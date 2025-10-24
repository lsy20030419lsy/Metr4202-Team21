#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
from collections import deque


class FrontierDetector(Node):
    """
    Simple frontier-based explorer with a deadlock escape:
    - Picks frontiers from the global costmap.
    - If we spend too long on one goal (30s by default), we try to bail out to a
      wider open area (low-cost, large connected component, with a small clear
      radius around the landing cell). If that fails, fall back to the nearest
      safe low-cost cell.
    """

    def __init__(self):
        super().__init__('frontier_detector')

        # -------------------- Tunables --------------------
        # Cost threshold considered "safe" to stand on / pass through.
        self.safe_cost_threshold = 30

        # If we've been working on the same goal longer than this, we call it "stuck".
        self.stuck_seconds = 30.0

        # We won't revisit goals within this distance (meters).
        self.visited_dist_thresh = 0.20

        # Frontier cells must be below this cost and adjacent to unknown (-1).
        self.frontier_free_ceiling = 70

        # When choosing a frontier, cluster the closest top ratio and take the
        # center of the largest cluster.
        self.topk_ratio = 0.30

        # For escape: ignore tiny "bubbles"; only consider open components with at least this many cells.
        self.open_area_min_cells = 200

        # For escape: candidate cell must have at least this free radius around it (meters).
        self.min_clear_radius_m = 0.4
        # -------------------------------------------------

        # Subscriptions / publishers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10
        )
        self.bt_sub = self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Periodic stuck check (lightweight)
        self.timer = self.create_timer(1.0, self.check_stuck)

        # Runtime state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_idle = True
        self.callback_first = True
        self.current_waypoint = None              # (x, y) or None
        self.current_goal_start_time = None       # rclpy.time.Time or None
        self.visited_points = []                  # list[(x, y)]
        self.latest_costmap = None                # OccupancyGrid

    # -------------------- Callbacks --------------------

    def odom_callback(self, msg: Odometry):
        """Track the robot's current position in map frame."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def bt_log_callback(self, msg: BehaviorTreeLog):
        """
        Watch Nav2 behavior tree transitions to understand when a navigation
        cycle has succeeded/ended or is actively running. Keeps our 'idle' flag honest.
        """
        for evt in msg.event_log:
            if evt.node_name in ('NavigateToPose', 'NavigateRecovery'):
                st = evt.current_status
                if st in ('SUCCEEDED', 'SUCCESS', 'SUCCEED', 'IDLE'):
                    self.is_idle = True
                    self.current_waypoint = None
                    self.current_goal_start_time = None
                    self.get_logger().info("Reached goal → ready for the next frontier")
                elif st in ('RUNNING', 'ACTIVE'):
                    self.is_idle = False

    def costmap_callback(self, msg: OccupancyGrid):
        """
        On each costmap update: detect frontiers and, if we're idle, pick a new target.
        """
        self.latest_costmap = msg

        width  = msg.info.width
        height = msg.info.height
        res    = msg.info.resolution
        ox     = msg.info.origin.position.x
        oy     = msg.info.origin.position.y
        data   = msg.data

        _, frontier_cells = self.detect_frontiers(data, width, height)
        if not (frontier_cells and self.is_idle):
            return

        # Filter out cells we already visited (within a small radius).
        valid_cells = []
        for cx, cy in frontier_cells:
            wx = ox + cx * res
            wy = oy + cy * res
            if not self.is_visited(wx, wy, self.visited_dist_thresh):
                valid_cells.append((cx, cy))

        if not valid_cells:
            self.get_logger().info("All frontiers visited. Exploration looks done.")
            return

        # First pick: go far to open up a big chunk fast; later picks: cluster local frontiers.
        if self.callback_first:
            target = self.locate_farthest_frontier(valid_cells, ox, oy, res)
            self.callback_first = False
        else:
            target = self.locate_smart_frontier(valid_cells, ox, oy, res)

        if target is None:
            self.get_logger().warn("No valid frontier found.")
            return

        self.send_waypoint(target)

    # -------------------- Frontier detection & selection --------------------

    def detect_frontiers(self, data, width, height):
        """
        A frontier cell is free (cost < ceiling) and has at least one 4-neighbor that's unknown (-1).
        """
        grid = [["  "] * width for _ in range(height)]
        cells = []
        for idx, val in enumerate(data):
            if 0 <= val < self.frontier_free_ceiling:
                x = idx % width
                y = idx // width
                if self._has_unknown_neighbor(x, y, data, width, height):
                    grid[y][x] = " +"
                    cells.append((x, y))
        return grid, cells

    def _has_unknown_neighbor(self, x, y, data, w, h):
        """Quick 4-neighborhood unknown check."""
        if y > 0   and data[(y - 1) * w + x] == -1: return True
        if y < h-1 and data[(y + 1) * w + x] == -1: return True
        if x > 0   and data[y * w + (x - 1)] == -1: return True
        if x < w-1 and data[y * w + (x + 1)] == -1: return True
        return False

    def locate_farthest_frontier(self, cells, ox, oy, res):
        """First move: head for the farthest frontier to carve out space quickly."""
        far_pt, far_d = None, -1.0
        for cx, cy in cells:
            wx = ox + cx * res
            wy = oy + cy * res
            d = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if d > far_d:
                far_d = d
                far_pt = (wx, wy)
        return far_pt

    def locate_smart_frontier(self, cells, ox, oy, res):
        """
        Later moves: take the closest slice (top-k by distance), cluster by 4-connectivity,
        and head for the center of the largest cluster. This keeps motion purposeful and avoids jitter.
        """
        if not cells:
            return None

        with_dist = []
        for cx, cy in cells:
            wx = ox + cx * res
            wy = oy + cy * res
            with_dist.append(((cx, cy), math.hypot(wx - self.robot_x, wy - self.robot_y)))

        with_dist.sort(key=lambda t: t[1])
        k = max(1, int(len(with_dist) * self.topk_ratio))
        cand = [t[0] for t in with_dist[:k]]

        visited = set()
        clusters = []
        cand_set = set(cand)

        for cell in cand:
            if cell in visited:
                continue
            q = [cell]
            visited.add(cell)
            comp = []
            while q:
                x, y = q.pop()
                comp.append((x, y))
                for nx, ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if (nx, ny) in cand_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        q.append((nx, ny))
            clusters.append(comp)

        if not clusters:
            return None

        largest = max(clusters, key=len)
        avg_cx = sum(c[0] for c in largest) / len(largest)
        avg_cy = sum(c[1] for c in largest) / len(largest)
        wx = ox + avg_cx * res
        wy = oy + avg_cy * res
        return (wx, wy)

    # -------------------- Visited & goal publishing --------------------

    def is_visited(self, x, y, threshold):
        """Small helper to avoid hammering the same neighborhood over and over."""
        for vx, vy in self.visited_points:
            if math.hypot(vx - x, vy - y) < threshold:
                return True
        return False

    def send_waypoint(self, target_xy):
        """Publish a goal to Nav2 and update our local bookkeeping."""
        x, y = target_xy
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0  # keep heading simple; local planner will handle it

        self.goal_pub.publish(msg)
        self.current_waypoint = (x, y)
        self.current_goal_start_time = self.get_clock().now()
        self.visited_points.append((x, y))
        self.is_idle = False

        self.get_logger().info(f"New waypoint sent → ({x:.2f}, {y:.2f})")

    # -------------------- Stuck detection & escape --------------------

    def check_stuck(self):
        """
        Lightweight watchdog: if we've been chasing the same goal longer than
        'stuck_seconds', assume we're wedged and try the escape routine.
        """
        if self.current_waypoint is None or self.current_goal_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.current_goal_start_time).nanoseconds / 1e9
        if elapsed > self.stuck_seconds:
            self.get_logger().warn(f"Stuck for > {self.stuck_seconds:.0f}s — trying to escape…")
            self.escape_from_stuck()

    def escape_from_stuck(self):
        """
        Escape recipe (kept simple and predictable):
        1) Build a boolean mask of "free enough" cells (cost <= safe threshold).
        2) Find 4-connected components and ignore tiny blobs.
        3) In the largest component, look for the nearest cell that has at least
           'min_clear_radius_m' of fully free space around it (a small clear disk).
        4) If none match, fall back to the nearest safe cell anywhere.
        """
        if self.latest_costmap is None:
            self.get_logger().warn("No costmap yet; can't escape.")
            return

        msg = self.latest_costmap
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        # 1) Free-enough mask
        free = [False] * (w * h)
        for idx, c in enumerate(data):
            if 0 <= c <= self.safe_cost_threshold:
                free[idx] = True

        # 2) Connected components (4-neighborhood)
        visited = [False] * (w * h)
        components = []

        def neighbors(ix, iy):
            if ix > 0:     yield ix - 1, iy
            if ix < w - 1: yield ix + 1, iy
            if iy > 0:     yield ix, iy - 1
            if iy < h - 1: yield ix, iy + 1

        for y in range(h):
            for x in range(w):
                i = y * w + x
                if not free[i] or visited[i]:
                    continue
                q = deque([(x, y)])
                visited[i] = True
                comp = []
                while q:
                    cx, cy = q.popleft()
                    comp.append((cx, cy))
                    for nx, ny in neighbors(cx, cy):
                        j = ny * w + nx
                        if 0 <= j < w * h and free[j] and not visited[j]:
                            visited[j] = True
                            q.append((nx, ny))
                components.append(comp)

        # Prefer big open components; if none are big, use what we have.
        big_components = [c for c in components if len(c) >= self.open_area_min_cells]
        comps_to_use = big_components if big_components else components

        best_goal = None
        best_dist = float('inf')

        if comps_to_use:
            largest = max(comps_to_use, key=len)
            # Convert clearance meters → cells once
            min_r_cells = max(1, int(math.ceil(self.min_clear_radius_m / res)))

            # 3) Among cells that meet the min-clearance disk, pick the nearest to us
            for cx, cy in largest:
                if not self._is_clear_disk(cx, cy, w, h, free, min_r_cells):
                    continue
                wx = ox + cx * res
                wy = oy + cy * res
                d = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if d < best_dist:
                    best_dist = d
                    best_goal = (wx, wy)

        if best_goal is not None:
            self.get_logger().info(
                f"Escape: open area (≥ {self.min_clear_radius_m:.2f} m clearance) "
                f"→ ({best_goal[0]:.2f}, {best_goal[1]:.2f})"
            )
            self.send_waypoint(best_goal)
            return

        # 4) Fallback: nearest safe cell anywhere
        fallback = self.find_nearest_safe_spot(data, w, h, ox, oy, res)
        if fallback is not None:
            self.get_logger().info(
                f"Escape: no clear cell met the radius; falling back to nearest safe cell "
                f"→ ({fallback[0]:.2f}, {fallback[1]:.2f})"
            )
            self.send_waypoint(fallback)
        else:
            self.get_logger().warn("Escape: no safe spot found at all.")

    # Small helper: is a disk of radius r_cells around (cx, cy) entirely free?
    def _is_clear_disk(self, cx, cy, w, h, free_mask, r_cells):
        r2 = r_cells * r_cells
        for dy in range(-r_cells, r_cells + 1):
            yy = cy + dy
            if yy < 0 or yy >= h:
                return False
            # Clamp horizontal span for this scanline to the circle
            max_dx = int(math.floor((r2 - dy * dy) ** 0.5))
            for dx in range(-max_dx, max_dx + 1):
                xx = cx + dx
                if xx < 0 or xx >= w:
                    return False
                if not free_mask[yy * w + xx]:
                    return False
        return True

    def find_nearest_safe_spot(self, data, w, h, ox, oy, res):
        """Fallback: nearest cell with cost <= safe_cost_threshold."""
        best, best_d = None, float('inf')
        for idx, cost in enumerate(data):
            if 0 <= cost <= self.safe_cost_threshold:
                cx = idx % w
                cy = idx // w
                wx = ox + cx * res
                wy = oy + cy * res
                d = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if d < best_d:
                    best_d = d
                    best = (wx, wy)
        return best


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
