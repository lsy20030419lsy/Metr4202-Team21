import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import tf2_ros
from tf2_ros import TransformException

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # --- 参数定义 ---
        self.declare_parameter('marker_size', 0.1)  # ArUco Marker的边长(米)
        self.declare_parameter('aruco_dictionary_name', 'DICT_6X6_250') # 使用的字典名称
        self.declare_parameter('camera_frame', 'camera_link') # 相机坐标系的名称
        self.declare_parameter('map_frame', 'map') # 地图坐标系的名称
        
        # 获取参数
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dictionary_name = self.get_parameter('aruco_dictionary_name').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        
        # --- 初始化 ---
        self.bridge = CvBridge()
        
        # 初始化 ArUco 检测器
        aruco_dict_enum = cv2.aruco.__getattribute__(dictionary_name)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_enum)
        self.aruco_parameters = cv2.aruco.DetectorParameters()

        # TF2 缓冲和监听器，用于坐标变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 存储相机内参
        self.camera_matrix = None
        self.distortion_coeffs = None
        
        # 存储已检测到的 markers，用于持久化和降噪
        # 格式: { marker_id: {'poses': [Pose1, Pose2, ...], 'average_pose': Pose} }
        self.detected_markers = {}
        self.pose_buffer_size = 15 # 用于移动平均的姿态样本数

        # --- QoS Profiles ---
        # 为传感器数据使用更可靠的QoS设置，防止消息丢失
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 订阅 ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, qos_profile)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile)

        # --- 发布 ---
        # 发布 MarkerArray 可以在 RViz 中一次性更新所有标记
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_markers', 10)
        
        # 定时发布已存储的 markers，确保其在RViz中持久显示
        self.publish_timer = self.create_timer(1.0, self.publish_stored_markers)
        
        self.get_logger().info("健壮版 ArUco Detector 节点已启动。")

    def camera_info_callback(self, msg):
        """获取并存储相机内参，此回调只执行一次"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info("成功接收到相机内参。")
            self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg):
        """处理每一帧图像，检测ArUco Marker"""
        if self.camera_matrix is None:
            self.get_logger().warn("正在等待相机内参...", throttle_duration_sec=5)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.distortion_coeffs)

                for i, marker_id in enumerate(ids):
                    self.process_marker_pose(int(marker_id[0]), rvecs[i], tvecs[i], msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"图像处理失败: {e}")

    def process_marker_pose(self, marker_id, rvec, tvec, timestamp):
        """将marker位姿转换到地图坐标系并存储"""
        # 1. 创建Marker在相机坐标系下的PoseStamped对象
        pose_in_camera_frame = PoseStamped()
        pose_in_camera_frame.header.stamp = timestamp
        pose_in_camera_frame.header.frame_id = self.camera_frame
        
        pose_in_camera_frame.pose.position = Point(x=tvec[0][0], y=tvec[0][1], z=tvec[0][2])
        # 将旋转向量转换为四元数
        (roll, pitch, yaw), _ = cv2.Rodrigues(rvec)
        # 简化的四元数转换，如果需要精确朝向会更复杂
        q = Quaternion(x=0.0, y=0.0, z=np.sin(yaw/2), w=np.cos(yaw/2))
        pose_in_camera_frame.pose.orientation = q

        # 2. **【关键】** 使用TF2将位姿转换到地图坐标系
        try:
            # 等待并获取从 camera_frame 到 map_frame 的变换
            pose_in_map_frame = self.tf_buffer.transform(
                pose_in_camera_frame, self.map_frame, timeout=Duration(seconds=0.5))
            
            # 3. 存储并更新marker位姿
            self.update_marker_store(marker_id, pose_in_map_frame.pose)
            
        except TransformException as e:
            self.get_logger().warn(f"坐标变换失败: 从 {self.camera_frame} 到 {self.map_frame}。错误: {e}", throttle_duration_sec=5)

    def update_marker_store(self, marker_id, new_pose):
        """使用移动平均法更新并存储marker位姿，以实现降噪和持久化"""
        if marker_id not in self.detected_markers:
            # 首次发现该ID的marker
            self.detected_markers[marker_id] = {'poses': [], 'average_pose': new_pose}
        
        poses_buffer = self.detected_markers[marker_id]['poses']
        poses_buffer.append(new_pose.position)
        
        if len(poses_buffer) > self.pose_buffer_size:
            poses_buffer.pop(0) # 维持缓冲区大小
            
        # 计算位置的移动平均值
        avg_x = sum([p.x for p in poses_buffer]) / len(poses_buffer)
        avg_y = sum([p.y for p in poses_buffer]) / len(poses_buffer)
        avg_z = sum([p.z for p in poses_buffer]) / len(poses_buffer)
        
        avg_pose = Pose()
        avg_pose.position = Point(x=avg_x, y=avg_y, z=avg_z)
        avg_pose.orientation = new_pose.orientation # 朝向直接使用最新的
        
        self.detected_markers[marker_id]['average_pose'] = avg_pose
        self.get_logger().info(f"已更新 Marker ID {marker_id} 在地图上的位置: ({avg_x:.2f}, {avg_y:.2f})")

    def publish_stored_markers(self):
        """定时发布所有已发现marker的可视化标记，确保在RViz中持久显示"""
        marker_array = MarkerArray()
        
        for marker_id, data in self.detected_markers.items():
            avg_pose = data['average_pose']
            
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detected_aruco_markers"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = avg_pose
            
            # 设置标记的尺寸和颜色
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = 0.02
            marker.color.a = 0.9
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 0.1
            
            # lifetime设置为0表示永久存在，直到节点关闭
            marker.lifetime = Duration(seconds=0).to_msg()
            
            marker_array.markers.append(marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()