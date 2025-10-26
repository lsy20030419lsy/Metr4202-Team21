import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException

class ArucoMarkerLocator(Node):
    def __init__(self):
        super().__init__('aruco_marker_locator')
        # 声明参数：ArUco字典和标记边长（米）
        self.declare_parameter('marker_size', 0.05)  # 假设边长5厘米，可根据实际修改
        self.declare_parameter('aruco_dict_name', 'DICT_5X5_250')
        marker_size = float(self.get_parameter('marker_size').get_parameter_value().double_value)
        aruco_dict_name = self.get_parameter('aruco_dict_name').get_parameter_value().string_value

        # 获取OpenCV内置的字典常量
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.__getattribute__(aruco_dict_name))
        except Exception as e:
            self.get_logger().error(f"Unsupported aruco_dict_name: {aruco_dict_name}, defaulting to DICT_5X5_250")
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = marker_size

        # 图像话题订阅（假定相机命名空间为/camera）
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        # 可视化Marker发布
        self.marker_pub = self.create_publisher(Marker, 'aruco_markers', 10)
        # CvBridge用于ROS图像消息与OpenCV图像转换
        self.bridge = CvBridge()
        # 存储相机内参矩阵和畸变系数
        self.camera_matrix = None
        self.dist_coeffs = None

        # tf2变换缓存与监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 已处理的marker ID集合
        self.processed_ids = set()

    def camera_info_callback(self, info_msg: CameraInfo):
        # 提取相机内参矩阵K和畸变D
        k = info_msg.k  # 3x3 row-major
        self.camera_matrix = np.array(k, dtype=np.float64).reshape(3, 3)
        # D长度可能为5或更多，OpenCV需要numpy数组
        self.dist_coeffs = np.array(info_msg.d, dtype=np.float64)
        # 取消订阅camera_info，一旦获取内参就够了
        self.destroy_subscription(self.camera_info_sub)
        self.get_logger().info("Camera intrinsics received and stored.")

    def image_callback(self, image_msg: Image):
        # 等待相机内参准备好
        if self.camera_matrix is None:
            return  # 未获得相机参数前不处理图像

        # 将ROS图像消息转换为OpenCV图像
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        # 转换为灰度图用于检测
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 检测ArUco标记
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            return  # 没有检测到任何标记

        # 如果检测到多个marker，逐个处理
        for corner_pts, marker_id in zip(corners, ids.flatten()):
            # 跳过已处理的Marker ID
            if marker_id in self.processed_ids:
                continue

            # 利用相机内参和实际尺寸估计姿态（rvec为旋转向量，tvec为平移向量）
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner_pts, self.marker_length,
                                                               self.camera_matrix, self.dist_coeffs)
            if tvec is None:
                continue
            tvec = tvec[0][0]  # 提取平移向量 (单个marker的情况)
            rvec = rvec[0][0]  # 提取旋转向量

            # 将marker从相机坐标系转换到地图坐标系
            try:
                # 查找 map 到 相机光学坐标系 的变换。假定摄像头光学坐标系frame_id与图像消息一致
                cam_frame = image_msg.header.frame_id  # 通常类似于"camera_link"或"camera_optical_frame"
                # 获取当前时间的变换
                transform_map_cam = self.tf_buffer.lookup_transform('map', cam_frame, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f"TF transform lookup failed: {ex}")
                continue

            # 提取tf变换中的平移和旋转
            trans = transform_map_cam.transform.translation
            rot = transform_map_cam.transform.rotation
            # 将Quaternion转换为旋转矩阵
            # geometry_msgs中的四元数为(x,y,z,w)
            qw, qx, qy, qz = rot.w, rot.x, rot.y, rot.z
            # 构造方向余弦矩阵 (3x3)
            rot_matrix = np.array([
                [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
                [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
                [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            # 变换的平移
            translation = np.array([trans.x, trans.y, trans.z])

            # marker在地图坐标系下的位置计算: p_map = R_map_cam * p_cam + t_map_cam
            # 注意：OpenCV ArUco返回的tvec是在相机坐标系下，坐标系z轴朝前、x右、y下（OpenCV标准相机坐标）。
            # 假设camera_frame为光学坐标系，与OpenCV一致。
            marker_pos_map = translation + rot_matrix.dot(tvec)

            # 发布可视化Marker（绿色立方体）
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'aruco_cube'
            marker.id = int(marker_id)  # 使用marker ID作为标识
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(marker_pos_map[0])
            marker.pose.position.y = float(marker_pos_map[1])
            marker.pose.position.z = float(marker_pos_map[2])
            # 方向可以设置为单位四元数(不特别旋转，使方块与地图坐标对齐)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # 10cm的立方体边长
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0  # 绿色
            marker.color.b = 0.0
            marker.color.a = 1.0  # 不透明
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 永久显示
            self.marker_pub.publish(marker)

            # 发布可视化Marker（文本标签）
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = marker.header.stamp
            label.ns = 'aruco_text'
            label.id = int(marker_id)  # 文本使用相同ID，在不同命名空间下
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(marker_pos_map[0])
            label.pose.position.y = float(marker_pos_map[1])
            # 文本稍微上移，让它显示在方块上方
            label.pose.position.z = float(marker_pos_map[2] + 0.1)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.15  # 文本字体大小（米）
            label.color.r = 0.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.color.a = 1.0
            label.text = str(marker_id)
            label.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            self.marker_pub.publish(label)

            # 将该marker的ID加入已处理集合
            self.processed_ids.add(marker_id)
            self.get_logger().info(f"Marker {marker_id} localized at map position {marker_pos_map}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
