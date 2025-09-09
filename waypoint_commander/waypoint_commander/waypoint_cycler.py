import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    waypoint_cmder = WaypointCycler()
    rclpy.spin(waypoint_cmder)

if __name__ == '__main__':
    main()

class WaypointCycler(Node):
    def __init__(self):
        super().__init__('waypoint_cycler')
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.waypoint_counter = 0
        p0 = PoseStamped()
        p0.header.frame_id = "map"
        p0.pose.position.x = 1.7
        p0.pose.position.y = -0.5
        p0.pose.orientation.w = 1.0

        p1 = PoseStamped()
        p1.header.frame_id = "map"
        p1.pose.position.x = -0.6
        p1.pose.position.y = 1.8
        p1.pose.orientation.w = 1.0
        self.waypoints = [p0, p1]

    def bt_log_callback(self,msg:BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.send_waypoint()
    def send_waypoint(self):
        self.waypoint_counter += 1
        if self.waypoint_counter % 2:
            self.publisher_.publish(self.waypoints[1])
        else:
            self.publisher_.publish(self.waypoints[0])