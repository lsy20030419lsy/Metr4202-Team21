import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math

class ArucoMapMarker(Node):
    """
    一个集成的ROS 2节点，用于：
    1. 从摄像头图像中检测ArUco二维码。
    2. 计算其在全局`map`坐标系中的位置。
    3. 当第一次检测到某个ID的二维码时，在终端打印其位置。
    4. 在RViz中发布一个永久的、带ID标签的绿色方块作为可视化标记。
    """
    def __init__(self):
        super().__init__('aruco_map_marker')

        # --- 需要你根据实际情况修改的参数 ---
        # 你的相机内参矩阵 (需要通过相机标定获得)
        self.camera_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        # 你的相机畸变系数 (需要通过相机标定获得)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        # 你在仿真或现实世界中使用的ArUco二维码的边长（单位：米）
        self.marker_length_meters = 0.1

        # --- 内部状态变量 ---
        self.robot_pose = None
        self.bridge = CvBridge()
        self.detected_marker_ids = set() # 用一个集合来存储已发现的二维码ID，防止重复处理

        # --- ArUco检测器初始化 ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        # --- ROS 2 订阅者与发布者 ---
        # 订阅里程计信息，获取机器人当前位姿
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        # 订阅摄像头原始图像
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # <-- 确保这个话题名是正确的！
            self.image_callback,
            10)
        # 发布标记到RViz
        self.marker_publisher = self.create_publisher(Marker, '/aruco_markers', 10)

        self.get_logger().info('二维码地图标记节点已启动。')

    def odom_callback(self, msg):
        """里程计回调函数，用于更新机器人的当前位姿。"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # 从四元数计算偏航角 (yaw)
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_pose = {'x': pos.x, 'y': pos.y, 'theta': yaw}

    def image_callback(self, msg):
        """图像回调函数，处理摄像头图像并检测二维码。"""
        if self.robot_pose is None:
            self.get_logger().warn("正在等待里程计(odom)数据...", throttle_duration_sec=5)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return
            
        # 检测图像中的ArUco码
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        # 如果检测到了任何二维码
        if ids is not None:
            # 估算每个二维码的位姿（相对于相机）
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length_meters, self.camera_matrix, self.dist_coeffs)

            # 遍历所有检测到的二维码
            for i, marker_id_array in enumerate(ids):
                marker_id = int(marker_id_array[0])

                # 如果这个ID的二维码是第一次被发现
                if marker_id not in self.detected_marker_ids:
                    # --- 坐标系变换 ---
                    # 1. 获取二维码在相机坐标系下的位置 (tvec)
                    #    - ROS标准相机坐标系: Z轴向前, X轴向右, Y轴向下
                    tvec = tvecs[i][0]
                    x_marker_in_cam = tvec[0]
                    z_marker_in_cam = tvec[2] # 我们主要关心X(左右)和Z(前后)

                    # 2. 将相机坐标系下的位置转换到机器人坐标系下
                    #    - 假设相机安装在机器人正前方
                    #    - 相机的Z轴(向前)对应机器人的X轴(向前)
                    #    - 相机的X轴(向右)对应机器人的-Y轴(向左)
                    x_marker_in_robot = z_marker_in_cam
                    y_marker_in_robot = -x_marker_in_cam
                    
                    # 3. 将机器人坐标系下的位置转换到全局地图坐标系下
                    x_robot = self.robot_pose['x']
                    y_robot = self.robot_pose['y']
                    theta_robot = self.robot_pose['theta']
                    
                    x_marker_in_map = x_robot + x_marker_in_robot * math.cos(theta_robot) - y_marker_in_robot * math.sin(theta_robot)
                    y_marker_in_map = y_robot + x_marker_in_robot * math.sin(theta_robot) + y_marker_in_robot * math.cos(theta_robot)
                    
                    # 记录这个ID，并发布标记
                    self.detected_marker_ids.add(marker_id)
                    self.get_logger().info(f"--> 发现新二维码! ID: {marker_id}, 地图坐标: x={x_marker_in_map:.2f}, y={y_marker_in_map:.2f}")
                    
                    # 在计算出的地图位置发布一个可视化的标记
                    self.publish_marker(marker_id, x_marker_in_map, y_marker_in_map)

    def publish_marker(self, marker_id, x_pos, y_pos):
        """为新发现的二维码创建一个绿点，并附上其ID作为文字标签。"""
        
        # --- 创建绿点 (方块) ---
        point_marker = Marker()
        point_marker.header.frame_id = "map"  # <-- 【关键修改】使用 'map' 坐标系
        point_marker.header.stamp = self.get_clock().now().to_msg()
        point_marker.ns = "aruco_points"
        point_marker.id = marker_id
        point_marker.type = Marker.CUBE # 使用方块，也可以用 SPHERE
        point_marker.action = Marker.ADD
        
        point_marker.pose.position.x = x_pos
        point_marker.pose.position.y = y_pos
        point_marker.pose.position.z = 0.05 # 稍微抬高，避免与地面重叠
        point_marker.pose.orientation.w = 1.0
        
        point_marker.scale.x = 0.1 # 点的大小
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1

        point_marker.color.g = 1.0 # 绿色
        point_marker.color.a = 1.0 # 不透明
        
        point_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # 0秒代表永久存在
        
        # --- 创建文字标签 ---
        text_marker = Marker()
        text_marker.header.frame_id = "map" # <-- 【关键修改】同样使用 'map' 坐标系
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "aruco_labels"
        text_marker.id = marker_id # ID可以和点一样
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = x_pos
        text_marker.pose.position.y = y_pos
        text_marker.pose.position.z = 0.2 # 放在点的上方
        
        text_marker.scale.z = 0.15 # 字体大小
        
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"ID: {marker_id}"

        text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # 永久存在

        # 发布两个标记
        self.marker_publisher.publish(point_marker)
        self.marker_publisher.publish(text_marker)
        self.get_logger().info(f"    已为ID: {marker_id} 在地图上发布绿色标记。")

def main(args=None):
    rclpy.init(args=args)
    aruco_map_marker_node = ArucoMapMarker()
    try:
        rclpy.spin(aruco_map_marker_node)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_map_marker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
