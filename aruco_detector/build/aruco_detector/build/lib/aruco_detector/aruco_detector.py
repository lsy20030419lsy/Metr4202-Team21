import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from nav_msgs.msg import Odometry


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.robot_pose = None

    def odom_callback(self, msg):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extract the quaternion and convert it to a yaw angle
        orientation_q = msg.pose.pose.orientation
        theta = self.get_yaw_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Store the robot's pose
        self.robot_pose = {'x': x, 'y': y, 'theta': theta}
        self.get_logger().info(f"Robot Pose - x: {x:.2f}, y: {y:.2f}, theta: {math.degrees(theta):.2f} degrees")

    def get_yaw_from_quaternion(self, x, y, z, w):
        # Convert quaternion to euler (yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


class ArucoPositionCalculator(OdomSubscriber):
    def __init__(self, OdomSubscriber):
        self.odom_subscriber = OdomSubscriber

    def calculate_marker_position(self, x_marker_cam, z_marker_cam):
        # Make sure the robot's pose has been received
        if self.odom_subscriber.robot_pose is None:
            print("Odom data not yet available. OdomSubscriber 2")
            return None 
            

        # Get the robot's current position and orientation from odom
        x_robot = self.odom_subscriber.robot_pose['x']
        y_robot = self.odom_subscriber.robot_pose['y']
        theta_robot = self.odom_subscriber.robot_pose['theta']

        # Transformation using the robot's pose (odometry data)
        x_marker_map = x_robot + (x_marker_cam * math.cos(theta_robot) - z_marker_cam * math.sin(theta_robot))
        y_marker_map = y_robot + (x_marker_cam * math.sin(theta_robot) + z_marker_cam * math.cos(theta_robot))

        return x_marker_map, y_marker_map


class ArucoDetector(Node):
    def __init__(self, odom_subscriber):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.aruco_position_calculator = ArucoPositionCalculator(odom_subscriber)

        # Camera calibration parameters (from googleing rasperry pi camera module)
        fx = 600
        fy = 600
        cx = 320
        cy = 240

        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        k1 = 0  # assume no distortion in simulation 
        k2 = 0
        k3 = 0
        p1 = 0
        p2 = 0
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        # Define the marker size in meters
        self.marker_length = 0.1

        # Wait for odom data before proceeding
        while self.aruco_position_calculator.odom_subscriber.robot_pose is None:
            self.get_logger().info("Waiting for odometry data...")
            rclpy.spin_once(self.aruco_position_calculator.odom_subscriber)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            # Pose estimation:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner, self.marker_length, self.camera_matrix, self.dist_coeffs)

                # Calculate marker position in the map frame using odometry data
                result = self.aruco_position_calculator.calculate_marker_position(tvec[0][0][0], tvec[0][0][2])
                if result is not None:
                    x_marker, y_marker = result
                    print(f"Marker ID: {ids[i][0]}, Position on map: x={x_marker:.2f}, y={y_marker:.2f}")
                # Draw axis
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        cv2.imshow("Aruco Detector", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the OdomSubscriber node
    odom_subscriber = OdomSubscriber()
    # Pass the OdomSubscriber instance to the ArucoDetector node
    aruco_detector = ArucoDetector(odom_subscriber)

    rclpy.spin(aruco_detector)

    # Clean up
    aruco_detector.destroy_node()
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
