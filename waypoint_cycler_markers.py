import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import random

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Subscribe to the global costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)

        # Subscribe to behavior tree logs
        self.bt_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_callback,
            10)

        # Subscribe to odometry for robot’s position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Publisher for navigation goals
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Publisher for ArUco markers
        self.marker_pub = self.create_publisher(Marker, '/markers', 10)

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Robot status flags
        self.callback_first = True
        self.is_idle = True
        self.reached_first_goal = False

        # Current waypoint and visited points
        self.current_waypoint = None
        self.visited_points = []

        # Marker ID counter
        self.marker_id = 0

        # Timer to place marker every 10 seconds
        self.create_timer(10.0, self.publish_marker_at_robot)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def costmap_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        frontier_grid, frontiers = self.detect_frontiers(data, width, height)

        if frontiers:
            target = None
            if self.callback_first and self.is_idle:
                target = self.locate_farthest_frontier(frontiers, origin_x, origin_y, resolution)
                self.callback_first = False
                self.is_idle = False
                print("First farthest frontier detected")
            elif self.reached_first_goal:
                target = self.locate_nearest_frontier(frontiers, origin_x, origin_y, resolution)
                print("Closest frontier detected")
            self.send_waypoint(target, origin_x, origin_y, resolution)
            self.current_waypoint = target
            if target is None:
                return

    def bt_log_callback(self, msg):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery':
                if event.current_status == 'IDLE':
                    self.is_idle = True
                    if not self.callback_first:
                        self.reached_first_goal = True
                elif event.current_status in ('RUNNING', 'ACTIVE'):
                    self.is_idle = False

                self.get_logger().info("NavigateRecovery status: {}".format(event.current_status))

    def detect_frontiers(self, data, width, height):
        frontier_map = [["  "] * width for _ in range(height)]
        candidates = []

        for idx, value in enumerate(data):
            if 0 <= value < 70:
                x, y = idx % width, idx // width
                if self._is_frontier_cell(x, y, data, width, height):
                    frontier_map[y][x] = " +"
                    candidates.append((x, y))
        return frontier_map, candidates

    def _is_frontier_cell(self, x, y, data, width, height):
        if y > 0 and data[(y - 1) * width + x] == -1:
            return True
        if y < height - 1 and data[(y + 1) * width + x] == -1:
            return True
        if x > 0 and data[y * width + (x - 1)] == -1:
            return True
        if x < width - 1 and data[y * width + (x + 1)] == -1:
            return True
        return False

    def locate_nearest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        closest_point = None
        shortest_dist = float('inf')

        for cell_x, cell_y in frontier_cells:
            world_x = map_origin_x + cell_x * map_resolution
            world_y = map_origin_y + cell_y * map_resolution
            dx = world_x - self.robot_x
            dy = world_y - self.robot_y
            dist = (dx ** 2 + dy ** 2) ** 0.5

            if (world_x, world_y) not in self.visited_points and dist < shortest_dist:
                shortest_dist = dist
                closest_point = (world_x, world_y)

        return closest_point

    def locate_farthest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        farthest_point = None
        longest_dist = -1.0

        for cell_x, cell_y in frontier_cells:
            world_x = map_origin_x + cell_x * map_resolution
            world_y = map_origin_y + cell_y * map_resolution
            dx = world_x - self.robot_x
            dy = world_y - self.robot_y
            dist = (dx ** 2 + dy ** 2) ** 0.5

            if dist > longest_dist:
                longest_dist = dist
                farthest_point = (world_x, world_y)

        return farthest_point

    def send_waypoint(self, target_point, map_origin_x, map_origin_y, map_resolution):
        if target_point in self.visited_points:
            self.get_logger().info(f"Skipping already visited location: {target_point}")
            return
        if target_point is None:
            self.get_logger().warn("No valid frontier point to send.")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_point[0]
        goal_msg.pose.position.y = target_point[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.publisher.publish(goal_msg)
        self.visited_points.append(target_point)
        self.get_logger().info(f"New waypoint sent → ({target_point[0]:.2f}, {target_point[1]:.2f})")

    def publish_marker_at_robot(self):
        """Places a blue sphere marker at the robot's current position every 10 seconds."""

        # --- Blue sphere marker ---
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "aruco_sim"
        sphere_marker.id = self.marker_id
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = self.robot_x
        sphere_marker.pose.position.y = self.robot_y
        sphere_marker.pose.position.z = 0.2
        sphere_marker.pose.orientation.w = 1.0
        sphere_marker.scale.x = 0.15
        sphere_marker.scale.y = 0.15
        sphere_marker.scale.z = 0.15
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0

        # --- Floating text label ---
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "aruco_sim_text"
        text_marker.id = self.marker_id + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.robot_x
        text_marker.pose.position.y = self.robot_y
        text_marker.pose.position.z = 0.4
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.15
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"ID: {self.marker_id}"

        # Publish markers
        self.marker_pub.publish(sphere_marker)
        self.marker_pub.publish(text_marker)

        self.marker_id += 1

        self.get_logger().info(
            f"📍 Marker placed at robot position ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

