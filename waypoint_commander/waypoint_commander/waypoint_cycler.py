import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Subscribe to the global costmap — used for finding new frontiers.
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)

        # Subscribe to behavior tree logs (to check robot’s state like idle or running).
        self.bt_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_callback,
            10)

        # Subscribe to odometry for robot’s position updates.
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Publisher that sends next navigation goal.
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Store current position of the robot.
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Used to check if this is the first run.
        self.callback_first = True

        # Robot status flags.
        self.is_idle = True
        self.reached_first_goal = False

        # Keep current and visited waypoints.
        self.current_waypoint = None
        self.visited_points = []

    def odom_callback(self, msg):
        """
        Called whenever new odometry data arrives.
        Updates the robot’s current x and y position based on its movement.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def costmap_callback(self, msg):
        """
        Runs when the global costmap updates.
        Checks the map for new frontiers and chooses the next spot to explore.
        """

        # Extract costmap information
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # Detect all potential frontiers on the map
        frontier_grid, frontiers = self.detect_frontiers(data, width, height)

        if frontiers:
            target = None
            if self.callback_first and self.is_idle:  # first run (initial exploration step)
                target = self.locate_farthest_frontier(frontiers, origin_x, origin_y, resolution)
                self.callback_first = False
                self.is_idle = False
                print("First farthest frontier detected")
            elif self.reached_first_goal:  # after reaching the first goal
                target = self.locate_nearest_frontier(frontiers, origin_x, origin_y, resolution)
                print("closest frontier detected")
            self.send_waypoint(target, origin_x, origin_y, resolution)
            self.current_waypoint = target
            if target is None:
                return

    def bt_log_callback(self, msg):
        """
        This function listens to updates from the Behavior Tree log.
        It basically tells us what the robot is doing — whether it’s idle, moving, or trying to recover.
        """

        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery':
                # If the robot just finished a task and became idle
                if event.current_status == 'IDLE':
                    self.is_idle = True
                    # Once it's not the first time anymore, mark that the first goal was reached
                    if not self.callback_first:
                        self.reached_first_goal = True
                # If the robot is currently on the move or executing navigation
                elif event.current_status in ('RUNNING', 'ACTIVE'):
                    self.is_idle = False

            # Print out the current status of the navigation node for debugging
            if event.node_name == 'NavigateRecovery':
                self.get_logger().info("NavigateRecovery status: {}".format(event.current_status))

    def detect_frontiers(self, data, width, height):
        """
        Scans through the costmap to find frontiers.
        A frontier is basically free space (0–70) that's right next to an unexplored area (-1).
        """
        frontier_map = [["  "] * width for _ in range(height)]  # Make a blank grid for marking frontiers
        candidates = []  # Store the (x, y) positions of detected frontier cells

        # Go through every single cell in the costmap
        for idx, value in enumerate(data):
            if 0 <= value < 70:  # If the cell is free to move through
                x, y = idx % width, idx // width  # Convert the 1D index to 2D coordinates
                # Check if any of the nearby cells are still unknown (-1)
                if self._is_frontier_cell(x, y, data, width, height):
                    frontier_map[y][x] = " +"  # Mark this spot as a frontier
                    candidates.append((x, y))  # Save the cell’s coordinates as a frontier point

        return frontier_map, candidates  # Return both the map (for display) and the list of frontiers

    def _is_frontier_cell(self, x, y, data, width, height):
        """
        Checks if the given cell is right next to any unknown area (-1).
        Basically, we just look around in four directions (up, down, left, right)
        to see if there’s unexplored space beside it.
        """
        # Look up
        if y > 0 and data[(y - 1) * width + x] == -1:
            return True
        # Look down
        if y < height - 1 and data[(y + 1) * width + x] == -1:
            return True
        # Look left
        if x > 0 and data[y * width + (x - 1)] == -1:
            return True
        # Look right
        if x < width - 1 and data[y * width + (x + 1)] == -1:
            return True

        return False  # Not a frontier if no unknown neighbor is found

    def print_frontier_grid(self, frontier_grid, width, height):
        """
        Displays the detected frontiers in a simple text-based map.
        Each '+' symbol shows a frontier cell, while empty spaces mean no frontier.
        This is mainly for debugging or quick visualization in the terminal.
        """
        # Print a title for the map display
        print("\n=== Frontier Map ===")

        # Print X-axis numbers (they just repeat 0–9 to match map width)
        print("   " + "".join([f"{x % 10}" for x in range(width)]))

        # Go through each row from top to bottom so it looks like a real map view
        for y in reversed(range(height)):  # reverse to show the top of the map first
            row = "".join(frontier_grid[y])  # Turn each row (list) into one string for printing
            # Print row index (2 digits), row content, and vertical borders for neatness
            print(f"{y:02d}|{row}|")

        # Draw the bottom border line under the map
        print("=" * (width + 5))

    def locate_nearest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        """
        Finds the closest frontier point to the robot's current position.
        Basically, it checks all the detected frontiers and picks the one
        with the smallest distance that hasn’t been visited yet.
        """
        closest_point = None
        shortest_dist = float('inf')  # start with a really large number

        # Loop through every frontier candidate
        for cell_x, cell_y in frontier_cells:
            # Convert grid coordinates to actual world coordinates
            world_x = map_origin_x + cell_x * map_resolution
            world_y = map_origin_y + cell_y * map_resolution

            # Calculate straight-line (Euclidean) distance from the robot
            dx = world_x - self.robot_x
            dy = world_y - self.robot_y
            dist = (dx ** 2 + dy ** 2) ** 0.5

            # Check if this frontier is closer and hasn’t been visited yet
            if (world_x, world_y) not in self.visited_points and dist < shortest_dist:
                shortest_dist = dist
                closest_point = (world_x, world_y)

        # Return the closest frontier (or None if nothing found)
        return closest_point

    def locate_farthest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        """
        Finds the frontier point that’s the farthest away from the robot.
        Used mainly at the start so the robot heads toward the biggest unexplored area first.
        """
        farthest_point = None
        longest_dist = -1.0  # start low so the first valid point replaces it

        # Go through all detected frontier cells
        for cell_x, cell_y in frontier_cells:
            # Convert from grid cell coordinates to actual world coordinates
            world_x = map_origin_x + cell_x * map_resolution
            world_y = map_origin_y + cell_y * map_resolution

            # Calculate the straight-line (Euclidean) distance
            dx = world_x - self.robot_x
            dy = world_y - self.robot_y
            dist = (dx ** 2 + dy ** 2) ** 0.5

            # If this one’s farther than what we’ve seen so far, store it
            if dist > longest_dist:
                longest_dist = dist
                farthest_point = (world_x, world_y)

        # Return the farthest frontier found (or None if nothing’s valid)
        return farthest_point

    def send_waypoint(self, target_point, map_origin_x, map_origin_y, map_resolution):
        """
        Publishes the chosen frontier point as a navigation goal (PoseStamped message).
        Basically, this tells the robot where to go next.
        """
        # Don’t resend a waypoint that’s already been visited
        if target_point in self.visited_points:
            self.get_logger().info(f"Skipping already visited location: {target_point}")
            return

        # Skip if there’s no valid target to move toward
        if target_point is None:
            self.get_logger().warn("No valid frontier point to send.")
            return

        # Create and fill out the PoseStamped message for the new waypoint
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  # everything is in the map frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()  # timestamp for ROS

        goal_msg.pose.position.x = target_point[0]
        goal_msg.pose.position.y = target_point[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # simple default orientation (no rotation)

        # Send out the waypoint and log it
        self.publisher.publish(goal_msg)
        self.visited_points.append(target_point)
        self.get_logger().info(f"New waypoint sent → ({target_point[0]:.2f}, {target_point[1]:.2f})")


def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()