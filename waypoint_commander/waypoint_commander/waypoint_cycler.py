# 文件路径: ~/your_ros2_ws/src/waypoint_commander/waypoint_commander/waypoint_cycler.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # 订阅全局代价地图
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)

        # 订阅行为树日志以获取机器人状态
        self.bt_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_callback,
            10)

        # 订阅里程计以获取机器人位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # 发布导航目标点
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # 存储机器人当前位置
        self.robot_x = 0.0
        self.robot_y = 0.0

        # 状态标志位
        self.callback_first = True
        self.is_idle = True
        self.reached_first_goal = False

        # 存储访问过的路径点
        self.visited_points = []
        
        self.get_logger().info("边界探索节点 (Frontier Detector) 已启动。")

    def odom_callback(self, msg):
        """更新机器人当前位置"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def costmap_callback(self, msg):
        """代价地图更新时的回调，用于寻找并发布新的探索目标"""
        # 提取地图信息
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # 检测所有边界点
        _, frontiers = self.detect_frontiers(data, width, height)

        if frontiers:
            target = None
            # 首次运行时，选择最远的边界点
            if self.callback_first and self.is_idle:
                target = self.locate_farthest_frontier(frontiers, origin_x, origin_y, resolution)
                self.callback_first = False
                self.is_idle = False
                self.get_logger().info("检测到初始最远边界点。")
            # 到达第一个目标后，开始选择最近的边界点
            elif self.reached_first_goal:
                target = self.locate_nearest_frontier(frontiers, origin_x, origin_y, resolution)
                self.get_logger().info("已到达首个目标，开始寻找最近边界点。")
            
            if target:
                self.send_waypoint(target)

    def bt_log_callback(self, msg):
        """监听行为树，判断机器人是否空闲"""
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery':
                if event.current_status == 'IDLE' and not self.is_idle:
                    self.is_idle = True
                    self.get_logger().info("机器人已到达目标，状态变为空闲。")
                    if not self.callback_first:
                        self.reached_first_goal = True
                elif event.current_status in ('RUNNING', 'ACTIVE') and self.is_idle:
                    self.is_idle = False
                    self.get_logger().info("机器人开始移动，状态变为运行中。")

    def detect_frontiers(self, data, width, height):
        """扫描代价地图寻找边界点（未知区域-1旁的空闲区域0-70）"""
        candidates = []
        for idx, value in enumerate(data):
            if 0 <= value < 70:
                x, y = idx % width, idx // width
                if self._is_frontier_cell(x, y, data, width, height):
                    candidates.append((x, y))
        return None, candidates

    def _is_frontier_cell(self, x, y, data, width, height):
        """检查一个空闲单元格是否与未知单元格相邻"""
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and data[ny * width + nx] == -1:
                return True
        return False

    def locate_nearest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        """从所有边界点中找到距离机器人最近的一个"""
        closest_point = None
        shortest_dist = float('inf')

        for cell_x, cell_y in frontier_cells:
            world_x = map_origin_x + (cell_x + 0.5) * map_resolution
            world_y = map_origin_y + (cell_y + 0.5) * map_resolution
            
            dist = math.hypot(world_x - self.robot_x, world_y - self.robot_y)

            if (world_x, world_y) not in self.visited_points and dist < shortest_dist:
                shortest_dist = dist
                closest_point = (world_x, world_y)
        return closest_point

    def locate_farthest_frontier(self, frontier_cells, map_origin_x, map_origin_y, map_resolution):
        """从所有边界点中找到距离机器人最远的一个"""
        farthest_point = None
        longest_dist = -1.0

        for cell_x, cell_y in frontier_cells:
            world_x = map_origin_x + (cell_x + 0.5) * map_resolution
            world_y = map_origin_y + (cell_y + 0.5) * map_resolution
            
            dist = math.hypot(world_x - self.robot_x, world_y - self.robot_y)
            
            if dist > longest_dist:
                longest_dist = dist
                farthest_point = (world_x, world_y)
        return farthest_point

    def send_waypoint(self, target_point):
        """将目标点封装成PoseStamped消息并发布"""
        if target_point in self.visited_points:
            self.get_logger().warn(f"目标点 {target_point} 已访问，跳过。")
            return
        if target_point is None:
            self.get_logger().warn("无有效边界点可发送。")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_point[0]
        goal_msg.pose.position.y = target_point[1]
        goal_msg.pose.orientation.w = 1.0

        self.publisher.publish(goal_msg)
        self.visited_points.append(target_point)
        self.get_logger().info(f"已发送新路径点 -> ({target_point[0]:.2f}, {target_point[1]:.2f})")

def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
