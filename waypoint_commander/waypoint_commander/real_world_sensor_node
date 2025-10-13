import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco

class RealWorldSensorNode(Node):
    def __init__(self):
        super().__init__('real_world_sensor_node')
        
        # 创建一个发布者，用于发送“我看到了”的信号
        self.publisher_ = self.create_publisher(String, '/real_world_target', 10)
        
        # 尝试打开你电脑的默认摄像头 (通常是第0个)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("致命错误：无法打开你的真实世界摄像头！请检查摄像头是否被占用。")
            return
            
        # 初始化ArUco检测器
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
        
        # 创建一个计时器，每0.1秒检查一次摄像头画面
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 创建一个标志位，确保信号只发送一次
        self.signal_sent = False
        
        self.get_logger().info("现实世界传感器已启动！请将ArUco二维码展示给你电脑的摄像头。")

    def timer_callback(self):
        # 如果信号已经发送过了，就没必要继续浪费CPU资源了
        if self.signal_sent:
            # 停止计时器，这个节点的主要任务已经完成
            self.timer.cancel()
            self.get_logger().info("信号已发送，节点进入待机状态。")
            return

        # 从摄像头读取一帧画面
        ret, frame = self.cap.read()
        if ret:
            # 在画面中检测二维码
            corners, ids, _ = self.detector.detectMarkers(frame)
            
            # 如果检测到了
            if ids is not None:
                detected_id = ids[0][0] # 获取第一个检测到的ID
                self.get_logger().info(f"在现实世界中检测到ID: {detected_id}！正在向机器人发送信号...")
                
                # 创建一个字符串消息
                msg = String()
                msg.data = str(detected_id)
                
                # 发布消息
                self.publisher_.publish(msg)
                
                # 将标志位设为True，这样就不会重复发送了
                self.signal_sent = True

    def __del__(self):
        # 节点关闭时，确保摄像头资源被释放
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = RealWorldSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
