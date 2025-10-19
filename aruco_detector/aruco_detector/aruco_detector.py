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
    An integrated ROS 2 node for:
    1. Detecting ArUco markers from a camera image.
    2. Calculating their position in the global `map` coordinate frame.
    3. Printing the position to the console upon the first detection of a specific ID.
    4. Publishing a permanent visualization marker (a green cube with an ID label) in RViz.
    """
    def __init__(self):
        super().__init__('aruco_map_marker')

        # --- Parameters to modify based on your setup ---
        # Your camera's intrinsic matrix (obtained from calibration)
        self.camera_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        # Your camera's distortion coefficients (obtained from calibration)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        # The side length of the ArUco marker you are using (in meters)
        self.marker_length_meters = 0.1

        # --- Internal state variables ---
        self.robot_pose = None
        self.bridge = CvBridge()
        # A set to store the IDs of detected markers to avoid reprocessing
        self.detected_marker_ids = set() 

        # --- ArUco detector initialization ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        # --- ROS 2 Subscribers and Publishers ---
        # Subscribe to odometry information to get the robot's current pose
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        # Subscribe to the raw camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # <-- Make sure this topic name is correct!
            self.image_callback,
            10)
        # Publisher for visualization markers to RViz
        self.marker_publisher = self.create_publisher(Marker, '/aruco_markers', 10)

        self.get_logger().info('ArUco map marker node has started.')

    def odom_callback(self, msg):
        """Callback function for odometry, used to update the robot's current pose."""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Calculate the yaw angle from the quaternion
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_pose = {'x': pos.x, 'y': pos.y, 'theta': yaw}

    def image_callback(self, msg):
        """Callback function for images, processes the camera feed and detects markers."""
        if self.robot_pose is None:
            self.get_logger().warn("Waiting for odometry data...", throttle_duration_sec=5)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
            
        # Detect ArUco markers in the image
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        # If any markers are detected
        if ids is not None:
            # Estimate the pose of each marker (relative to the camera)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length_meters, self.camera_matrix, self.dist_coeffs)

            # Iterate through all detected markers
            for i, marker_id_array in enumerate(ids):
                marker_id = int(marker_id_array[0])

                # If this marker ID is detected for the first time
                if marker_id not in self.detected_marker_ids:
                    # --- Coordinate Transformation ---
                    # 1. Get the marker's position in the camera coordinate frame (tvec)
                    #    - ROS standard camera coordinates: Z forward, X right, Y down
                    tvec = tvecs[i][0]
                    x_marker_in_cam = tvec[0]
                    z_marker_in_cam = tvec[2] # We are mainly interested in X (left/right) and Z (forward/backward)

                    # 2. Transform the position from the camera frame to the robot's frame
                    #    - Assuming the camera is mounted on the front of the robot
                    #    - The camera's Z-axis (forward) corresponds to the robot's X-axis (forward)
                    #    - The camera's X-axis (right) corresponds to the robot's -Y-axis (left)
                    x_marker_in_robot = z_marker_in_cam
                    y_marker_in_robot = -x_marker_in_cam
                    
                    # 3. Transform the position from the robot's frame to the global map frame
                    x_robot = self.robot_pose['x']
                    y_robot = self.robot_pose['y']
                    theta_robot = self.robot_pose['theta']
                    
                    x_marker_in_map = x_robot + x_marker_in_robot * math.cos(theta_robot) - y_marker_in_robot * math.sin(theta_robot)
                    y_marker_in_map = y_robot + x_marker_in_robot * math.sin(theta_robot) + y_marker_in_robot * math.cos(theta_robot)
                    
                    # Record this ID and publish the marker
                    self.detected_marker_ids.add(marker_id)
                    self.get_logger().info(f"--> New marker found! ID: {marker_id}, Map Coordinates: x={x_marker_in_map:.2f}, y={y_marker_in_map:.2f}")
                    
                    # Publish a visualization marker at the calculated map position
                    self.publish_marker(marker_id, x_marker_in_map, y_marker_in_map)

    def publish_marker(self, marker_id, x_pos, y_pos):
        """Creates a green point (cube) and a text label for a newly discovered marker."""
        
        # --- Create Green Point (Cube) ---
        point_marker = Marker()
        point_marker.header.frame_id = "map"  # <-- [CRITICAL CHANGE] Use the 'map' frame
        point_marker.header.stamp = self.get_clock().now().to_msg()
        point_marker.ns = "aruco_points"
        point_marker.id = marker_id
        point_marker.type = Marker.CUBE # Using a CUBE, can also be a SPHERE
        point_marker.action = Marker.ADD
        
        point_marker.pose.position.x = x_pos
        point_marker.pose.position.y = y_pos
        point_marker.pose.position.z = 0.05 # Slightly elevated to avoid overlapping with the ground
        point_marker.pose.orientation.w = 1.0
        
        point_marker.scale.x = 0.1 # Size of the point
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1

        point_marker.color.g = 1.0 # Green color
        point_marker.color.a = 1.0 # Opaque
        
        point_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # A lifetime of 0 means it exists forever
        
        # --- Create Text Label ---
        text_marker = Marker()
        text_marker.header.frame_id = "map" # <-- [CRITICAL CHANGE] Also use the 'map' frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "aruco_labels"
        text_marker.id = marker_id # ID can be the same as the point's
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = x_pos
        text_marker.pose.position.y = y_pos
        text_marker.pose.position.z = 0.2 # Position it above the point
        
        text_marker.scale.z = 0.15 # Font size
        
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"ID: {marker_id}"

        text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # Exists forever

        # Publish both markers
        self.marker_publisher.publish(point_marker)
        self.marker_publisher.publish(text_marker)
        self.get_logger().info(f"   Published green marker on the map for ID: {marker_id}.")

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
