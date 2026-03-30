#!/usr/bin/env python3
"""
Frontier-based autonomous exploration node.

Key fixes:
- Use the robot's real pose from TF instead of (0, 0).
- Do not mark a frontier as visited until the goal is actually reached.
- Avoid spinning 360° at every waypoint; use a shorter, slower scan spin.
- Add goal back-off so the robot stops slightly before the frontier boundary.
- Keep a small retry/skip history so the node does not get stuck on one frontier.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ExploreState(Enum):
    IDLE = 0
    EXPLORE = 1
    MOVING = 2
    SPINNING = 3
    DONE = 4


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ---------------- Parameters ----------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('min_frontier_size', 8)      # cells
        self.declare_parameter('goal_tolerance', 0.35)      # m
        self.declare_parameter('exploration_timeout', 900.0)  # s
        self.declare_parameter('explore_timer_period', 1.5)  # s

        # Frontier scoring / safety
        self.declare_parameter('frontier_approach_offset', 0.35)  # m back from centroid toward robot
        self.declare_parameter('frontier_retry_limit', 2)
        self.declare_parameter('frontier_size_weight', 0.01)      # larger frontier = slightly preferred
        self.declare_parameter('frontier_min_distance', 0.60)      # ignore frontiers too close to robot

        # Post-arrival scan
        self.declare_parameter('spin_on_arrival', True)
        self.declare_parameter('spin_yaw', 3.14)                    # 180 deg; faster than full 360°
        self.declare_parameter('spin_timeout', 20.0)                # s

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.exploration_timeout = float(self.get_parameter('exploration_timeout').value)
        self.explore_timer_period = float(self.get_parameter('explore_timer_period').value)
        self.frontier_approach_offset = float(self.get_parameter('frontier_approach_offset').value)
        self.frontier_retry_limit = int(self.get_parameter('frontier_retry_limit').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.frontier_min_distance = float(self.get_parameter('frontier_min_distance').value)
        self.spin_on_arrival = bool(self.get_parameter('spin_on_arrival').value)
        self.spin_yaw = float(self.get_parameter('spin_yaw').value)
        self.spin_timeout = float(self.get_parameter('spin_timeout').value)

        # ---------------- State ----------------
        self.state = ExploreState.IDLE
        self.map_data: OccupancyGrid | None = None
        self.exploration_start_time = None

        # visited/frontier bookkeeping
        self.visited_frontiers = set()
        self.failed_frontiers = {}
        self.active_goal_key = None

        # nav / spin handles
        self._nav_goal_handle = None
        self._spin_goal_handle = None

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- QoS for latched map ----------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, map_qos
        )
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        # ---------------- Services ----------------
        self.start_srv = self.create_service(Trigger, '/explore/start', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/explore/stop', self.stop_callback)

        # ---------------- Action clients ----------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        # ---------------- Timer ----------------
        self.explore_timer = self.create_timer(self.explore_timer_period, self.explore_loop)

        self.get_logger().info('FrontierExplorer ready. Call /explore/start to begin.')

    # =========================================================
    # Utils
    # =========================================================
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        half = yaw * 0.5
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Cannot get TF {self.map_frame}->{self.base_frame}: {ex}')
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        # quaternion -> yaw
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def reset_session(self):
        self.visited_frontiers.clear()
        self.failed_frontiers.clear()
        self.active_goal_key = None
        self._nav_goal_handle = None
        self._spin_goal_handle = None
        self.exploration_start_time = time.monotonic()

    def cancel_all_actions(self):
        try:
            self.nav_client.cancel_all_goals_async()
        except Exception:
            pass
        try:
            self.spin_client.cancel_all_goals_async()
        except Exception:
            pass

    # =========================================================
    # ROS callbacks
    # =========================================================
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def start_callback(self, request, response):
        if self.state not in (ExploreState.IDLE, ExploreState.DONE):
            response.success = False
            response.message = f'Already running in state: {self.state.name}'
            return response

        self.reset_session()
        self.state = ExploreState.EXPLORE
        response.success = True
        response.message = 'Exploration started!'
        self.get_logger().info('>>> Exploration STARTED')
        return response

    def stop_callback(self, request, response):
        self.cancel_all_actions()
        self.state = ExploreState.IDLE
        self.active_goal_key = None
        self.get_logger().info('>>> Exploration STOPPED by user.')
        response.success = True
        response.message = 'Exploration stopped.'
        return response

    # =========================================================
    # Main loop
    # =========================================================
    def explore_loop(self):
        if self.state not in (ExploreState.EXPLORE,):
            return

        if self.map_data is None:
            if not hasattr(self, '_last_wait_warn') or (time.monotonic() - self._last_wait_warn) > 5.0:
                self._last_wait_warn = time.monotonic()
                self.get_logger().warn('Waiting for map...')
            return

        if self.exploration_start_time is not None:
            elapsed = time.monotonic() - self.exploration_start_time
            if elapsed > self.exploration_timeout:
                self.get_logger().warn('Exploration timeout reached. Stopping.')
                self.state = ExploreState.DONE
                return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        frontiers = self.detect_frontiers(self.map_data)
        if not frontiers:
            self.get_logger().info('No frontiers found. Exploration COMPLETE.')
            self.state = ExploreState.DONE
            return

        self.publish_frontier_markers(frontiers)

        goal = self.select_best_frontier(frontiers, robot_pose)
        if goal is None:
            self.get_logger().info('All remaining frontiers were already tried. Exploration COMPLETE.')
            self.state = ExploreState.DONE
            return

        gx, gy, key = goal
        self.active_goal_key = key

        self.get_logger().info(f'Navigating to frontier: ({gx:.2f}, {gy:.2f})')
        self.state = ExploreState.MOVING
        self.send_nav_goal(gx, gy)

    # =========================================================
    # Frontier detection
    # =========================================================
    def detect_frontiers(self, map_msg: OccupancyGrid):
        """
        Frontier = free cell (0) with at least one 4-neighbor unknown (255 or -1).
        Returns [(wx, wy, size), ...]
        """
        info = map_msg.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = np.array(map_msg.data, dtype=np.int16).reshape((height, width))

        frontier_cells = []
        for r in range(1, height - 1):
            for c in range(1, width - 1):
                val = data[r, c]
                if val < 0 or val > 20:
                    continue
                
                neighbors = (data[r - 1, c], data[r + 1, c], data[r, c - 1], data[r, c + 1])
                if -1 in neighbors:
                    frontier_cells.append((r, c))

        if not frontier_cells:
            self.get_logger().warn(f'No frontier cells found! Unique values in costmap: {np.unique(data)}')
            return []
        cell_set = set(frontier_cells)
        visited = set()
        clusters = []

        for seed in frontier_cells:
            if seed in visited:
                continue

            cluster = []
            q = deque([seed])

            while q:
                cur = q.popleft()
                if cur in visited:
                    continue
                visited.add(cur)

                if cur not in cell_set:
                    continue

                cluster.append(cur)
                r, c = cur
                for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                    if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                        q.append((nr, nc))

            if len(cluster) < self.min_frontier_size:
                continue

            cr = float(np.mean([p[0] for p in cluster]))
            cc = float(np.mean([p[1] for p in cluster]))
            wx = cc * res + ox + res * 0.5
            wy = cr * res + oy + res * 0.5
            clusters.append((wx, wy, len(cluster)))

        return clusters

    # =========================================================
    # Frontier selection
    # =========================================================
    def select_best_frontier(self, frontiers, robot_pose):
        """
        Choose the nearest useful frontier that has not been visited or over-retried.
        Return (goal_x, goal_y, key) or None.
        """
        robot_x, robot_y, _ = robot_pose
        candidates = []

        distance_weight = 5.0

        for wx, wy, size in frontiers:
            dist = math.hypot(wx - robot_x, wy - robot_y)
            key = (round(wx, 2), round(wy, 2))

            if key in self.visited_frontiers:
                continue
            if self.failed_frontiers.get(key, 0) >= self.frontier_retry_limit:
                continue
            if dist < self.frontier_min_distance:
                continue

            score = (distance_weight * dist) - (self.frontier_size_weight * size)
            candidates.append((score, wx, wy, key))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        _, wx, wy, key = candidates[0]

        goal_x, goal_y = self.approach_frontier(robot_pose, (wx, wy))
        return goal_x, goal_y, key

    def approach_frontier(self, robot_pose, frontier_xy):
        """
        Move to a point slightly before the frontier centroid so the robot stays in
        known free space and does not try to enter unknown cells.
        """
        rx, ry, _ = robot_pose
        fx, fy = frontier_xy

        dx = fx - rx
        dy = fy - ry
        dist = math.hypot(dx, dy)
        backoff_dist = 0.6

        if dist > backoff_dist:
            gx = fx - (dx / dist) * backoff_dist
            gy = fy - (dy / dist) * backoff_dist
        else:
            gx = rx
            gy = ry
        return gx, gy

    # =========================================================
    # Actions
    # =========================================================
    def send_nav_goal(self, x: float, y: float):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.state = ExploreState.EXPLORE
            return

        rx, ry, _ = robot_pose
        yaw = math.atan2(y - ry, x - rx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('navigate_to_pose action server not available.')
            self.state = ExploreState.EXPLORE
            return

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2.')
            if self.active_goal_key is not None:
                self.failed_frontiers[self.active_goal_key] = self.failed_frontiers.get(self.active_goal_key, 0) + 1
            self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached.')

            if self.spin_on_arrival:
                self.state = ExploreState.SPINNING
                self.send_spin_goal()
                return

            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None

            self.state = ExploreState.EXPLORE
            return

        self.get_logger().warn(f'Goal failed (status={status}).')
        if self.active_goal_key is not None:
            self.failed_frontiers[self.active_goal_key] = self.failed_frontiers.get(self.active_goal_key, 0) + 1
            if self.failed_frontiers[self.active_goal_key] >= self.frontier_retry_limit:
                self.visited_frontiers.add(self.active_goal_key)
        self.active_goal_key = None
        self.state = ExploreState.EXPLORE

    def send_spin_goal(self):
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(self.spin_yaw)

        if not self.spin_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('spin action server not available. Continuing.')
            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        future = self.spin_client.send_goal_async(spin_goal)
        future.add_done_callback(self.spin_response_callback)

        # Spin timeout watchdog
        self._spin_started_at = time.monotonic()

    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected.')
            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        self._spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        _ = future.result()
        self.get_logger().info('Spin complete. Searching next frontier...')

        if self.active_goal_key is not None:
            self.visited_frontiers.add(self.active_goal_key)
            self.active_goal_key = None

        self.state = ExploreState.EXPLORE

    def nav_feedback_callback(self, feedback_msg):
        # Intentionally light to keep Jetson Xavier CPU load low.
        return

    # =========================================================
    # Visualization
    # =========================================================
    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (wx, wy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(wx), y=float(wy), z=0.10)
            m.pose.orientation.w = 1.0

            s = min(0.45, max(0.12, size * 0.01))
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s

            m.color.r = 1.0
            m.color.g = 0.55
            m.color.b = 0.0
            m.color.a = 0.85
            m.lifetime = Duration(seconds=2).to_msg()
            marker_array.markers.append(m)

        self.frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()