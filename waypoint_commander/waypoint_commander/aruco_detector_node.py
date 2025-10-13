import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
from tf_transformations import quaternion_from_euler

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # 声明并获取参数：标记在真实世界中的尺寸（单位：米）
        # 你的项目PDF里写的是 100mm x 100mm，所以是 0.1 米
        self.declare_parameter('marker_size', 0.1)
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        # 初始化CV Bridge，用于ROS图像和OpenCV图像的转换
        self.bridge = CvBridge()
        
        # 初始化摄像头参数为空，等待从话题中接收
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 初始化ArUco字典和检测器
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 创建订阅者，订阅摄像头的参数信息
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # 创建订阅者，订阅摄像头的原始图像
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # 创建发布者，用于发布检测到的二维码的3D姿态
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_aruco_pose', 10)
        
        self.get_logger().info('机器人的虚拟眼睛 (ArUco Detector Node) 已启动！正在等待摄像头参数...')

    def info_callback(self, msg):
        """回调函数：只执行一次，用于从/camera/camera_info话题获取摄像头的内部参数。"""
        if self.camera_matrix is None:
            # 将ROS消息中的参数转换成Numpy数组格式
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('成功接收到摄像头参数！现在可以开始检测二维码了。')
            # 成功获取后，销毁这个订阅者，因为它只需要一次信息
            self.destroy_subscription(self.info_sub)

    def image_callback(self, msg):
        """回调函数：每次收到新的图像帧时执行，用于检测二维码。"""
        # 如果还没收到摄像头参数，就直接返回，不处理图像
        if self.camera_matrix is None:
            self.get_logger().warn('尚未收到摄像头参数，无法处理图像。', throttle_duration_sec=5)
            return

        # 将ROS图像消息转换为OpenCV格式 (BGR8)
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 检测二维码
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        # 如果检测到了二维码
        if ids is not None:
            # 估算每个二维码的3D姿态（旋转向量rvecs和平移向量tvecs）
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # 遍历所有检测到的二维码
            for i, marker_id in enumerate(ids):
                # 创建一个 PoseStamped 消息来存储姿态信息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = msg.header.frame_id  # 坐标系应为相机的坐标系, e.g., 'camera_link'
                
                # 从平移向量 tvecs 获取位置信息
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]
                
                # 从旋转向量 rvecs 获取姿态信息，并转换为四元数
                rotation_matrix = cv2.Rodrigues(rvecs[i][0])[0]
                # 这里我们假设一个简化的从旋转矩阵到欧拉角的转换，然后到四元数
                # 在真实项目中可能需要更鲁棒的转换
                q = quaternion_from_euler(0, 0, np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]))
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                self.get_logger().info(f"在模拟器中发现ID {marker_id[0]}! 正在发布其相对于相机的位置。")
                self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
