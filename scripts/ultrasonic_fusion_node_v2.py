#!/usr/bin/env python3
"""
ultrasonic_fusion_node.py — v4

Thay đổi so với v3:
  1. Cập nhật 8 sensors: us_top, us_mid_1, us_mid_2, us_bot (left/right)
  2. Tất cả sensors pitch = 0° (ngang hoàn toàn) → không còn floor baseline logic
  3. us_bot: drop detection bằng cách so sánh với expected floor range
     (ở pitch=0°, us_bot ở z=0.12m sẽ không chạm sàn → báo inf khi bình thường)
     Logic mới: nếu us_bot đọc > DROP_THRESHOLD → có thể có hố/drop-off
  4. Fix Python 3.8 compatibility: Tuple từ typing module
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from typing import Tuple
import math
import numpy as np
from collections import deque


# ── Tất cả sensors pitch = 0° (ngang hoàn toàn, rpy="0 0 0") ──────────────
SENSOR_PITCH = {
    'us_top_left':    0.0,
    'us_top_right':   0.0,
    'us_mid_1_left':  0.0,
    'us_mid_1_right': 0.0,
    'us_mid_2_left':  0.0,
    'us_mid_2_right': 0.0,
    'us_bot_left':    0.0,
    'us_bot_right':   0.0,
}

# ── Chiều cao các sensor so với mặt đất (z_joint + wheel_radius) ───────────
WHEEL_RADIUS = 0.08255  # m

SENSOR_HEIGHT = {
    'us_top_left':    0.48 + WHEEL_RADIUS,   # ≈ 0.563m
    'us_top_right':   0.48 + WHEEL_RADIUS,   # ≈ 0.563m
    'us_mid_1_left':  0.36 + WHEEL_RADIUS,   # ≈ 0.443m
    'us_mid_1_right': 0.36 + WHEEL_RADIUS,   # ≈ 0.443m
    'us_mid_2_left':  0.24 + WHEEL_RADIUS,   # ≈ 0.323m
    'us_mid_2_right': 0.24 + WHEEL_RADIUS,   # ≈ 0.323m
    'us_bot_left':    0.12 + WHEEL_RADIUS,   # ≈ 0.203m
    'us_bot_right':   0.12 + WHEEL_RADIUS,   # ≈ 0.203m
}


class UltrasonicFusionNode(Node):

    # ── Ngưỡng phát hiện vật cản (tất cả sensors ngang) ────────────────────
    OBSTACLE_THRESHOLD = 0.4  # m — nếu sensor đọc < 0.4m → có vật cản (cảnh báo nguy hiểm)
    SENSOR_X_OFFSET = 0.35

    # ── Drop detection cho us_bot ────────────────────────────────────────────
    # us_bot ở z≈0.20m, pitch=0° → tia bắn ngang, KHÔNG chạm sàn khi không có vật cản
    # Nếu đọc được khoảng cách ngắn ở us_bot → vật cản thấp (bậc thang, đồ vật thấp)
    # Drop detection phức tạp hơn với sensor ngang — cần thêm sensor chúi xuống nếu cần
    # Tạm thời: us_bot dùng obstacle threshold thấp để cảnh báo vật cản thấp
    DROP_THRESHOLD = 0.40  # m — nếu us_bot < 0.40m → vật cản sát đất/bậc thang

    FILTER_WINDOW   = 5
    CANCEL_COOLDOWN = 2.0

    def __init__(self):
        super().__init__('ultrasonic_fusion_node')

        self.sensor_names = [
            'us_top_left',    'us_top_right',
            'us_mid_1_left',  'us_mid_1_right',
            'us_mid_2_left',  'us_mid_2_right',
            'us_bot_left',    'us_bot_right',
        ]

        self.buffers = {n: deque(maxlen=self.FILTER_WINDOW) for n in self.sensor_names}
        self.latest  = {n: float('inf') for n in self.sensor_names}

        # ── Góc ngang (horizontal angle) và pitch của từng sensor ───────────
        # Tất cả pitch = 0.0 vì rpy="0 0 0"
        # Angle: left sensors ở +y → góc dương, right sensors ở -y → góc âm
        self.sensor_angles = {
            'us_top_left':    (math.radians(0), 0.0),
            'us_top_right':   (math.radians(0), 0.0),
            'us_mid_1_left':  (math.radians(0), 0.0),
            'us_mid_1_right': (math.radians(0), 0.0),
            'us_mid_2_left':  (math.radians(0), 0.0),
            'us_mid_2_right': (math.radians(0), 0.0),
            'us_bot_left':    (math.radians(0), 0.0),
            'us_bot_right':   (math.radians(0), 0.0),
        }

        self._last_cancel_time = 0.0
        self._danger_prev      = False
        self._robot_navigating = False
        self._estop_pub        = None

        # ── Subscribers ──────────────────────────────────────────────────────
        for name in self.sensor_names:
            self.create_subscription(
                Range,
                f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n),
                10
            )

        self.create_subscription(
            String, '/robot/state', self._on_robot_state, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.scan_pub   = self.create_publisher(LaserScan, '/ultrasonic_scan', 10)
        self.safety_pub = self.create_publisher(Bool,      '/safety_stop',     10)
        self.cmdvel_pub = self.create_publisher(Twist,     '/cmd_vel',         10)
        self._estop_pub = self.create_publisher(Bool,      '/robot/emergency_stop', 10)

        self.create_subscription(Twist, '/cmd_vel_nav', self._cmdvel_cb, 10)
        self.create_timer(0.1, self._publish_scan)

        # ── Log cấu hình để verify ───────────────────────────────────────────
        self.get_logger().info('=== Ultrasonic Fusion Node v4 ===')
        self.get_logger().info(f'Sensors: {self.sensor_names}')
        self.get_logger().info('All sensors horizontal (pitch=0°)')
        for name in self.sensor_names:
            self.get_logger().info(
                f'  {name}: height={SENSOR_HEIGHT[name]:.3f}m | '
                f'obstacle_threshold={self.OBSTACLE_THRESHOLD}m'
            )
        self.get_logger().info('ultrasonic_fusion_node v4 started ✓')

    # ────────────────────────────────────────────────────────────────────────
    def _on_robot_state(self, msg: String):
        self._robot_navigating = msg.data in ('NAVIGATING', 'RETURNING_HOME')

    # ────────────────────────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        """Lọc và lưu raw range vào buffer."""
        if msg.min_range <= msg.range <= msg.max_range:
            self.buffers[name].append(msg.range)
        elif msg.range >= msg.max_range:
            self.buffers[name].append(float('inf'))

        if self.buffers[name]:
            vals = [v for v in self.buffers[name] if v != float('inf')]
            if vals:
                self.latest[name] = float(np.median(vals))
            else:
                self.latest[name] = float('inf')

    # ────────────────────────────────────────────────────────────────────────
    def _check_obstacle(self, name: str) -> Tuple[bool, str]:
        raw = self.latest[name]
        if raw == float('inf'):
            return False, ''
        if raw < self.OBSTACLE_THRESHOLD:
            return True, f'OBSTACLE — {name}: {raw:.3f}m'
        return False, ''

    # ────────────────────────────────────────────────────────────────────────
    def _is_danger(self) -> bool:
        """
        Kiểm tra tất cả sensors theo thứ tự ưu tiên:
        1. us_bot: vật cản sát đất (nguy hiểm nhất)
        2. us_mid_2: tầm thấp-giữa
        3. us_mid_1: tầm giữa
        4. us_top: tầm cao
        """
        # Kiểm tra theo thứ tự ưu tiên (thấp → cao)
        priority_order = [
            ['us_bot_left',    'us_bot_right'],
            ['us_mid_2_left',  'us_mid_2_right'],
            ['us_mid_1_left',  'us_mid_1_right'],
            ['us_top_left',    'us_top_right'],
        ]

        for group in priority_order:
            for name in group:
                danger, reason = self._check_obstacle(name)
                if danger:
                    self.get_logger().warn(
                        f'[SAFETY] {reason}',
                        throttle_duration_sec=1.0)
                    return True

        return False

    # ────────────────────────────────────────────────────────────────────────
    def _publish_scan(self):
        num_rays = 360
        ranges   = [float('inf')] * num_rays
        spread   = 8

        angle_min_scan = -math.pi
        angle_increment = 2 * math.pi / num_rays

        for name, (angle_rad, pitch) in self.sensor_angles.items():
            raw = self.latest[name]
            if raw == float('inf'):
                continue

            dist = raw + self.SENSOR_X_OFFSET
            center_idx = int(round((angle_rad - angle_min_scan) / angle_increment)) % num_rays

            for offset in range(-spread, spread + 1):
                idx = (center_idx + offset) % num_rays
                cos_val = math.cos(math.radians(offset * (15.0 / spread)))
                if cos_val > 0:
                    ranges[idx] = min(ranges[idx], dist / cos_val)

        scan                 = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment = 2 * math.pi / num_rays
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = 0.01
        scan.range_max       = 1.5
        scan.ranges          = ranges

        self.scan_pub.publish(scan)

        # Safety check & emergency stop
        danger = self._is_danger()
        self.safety_pub.publish(Bool(data=danger))

        if danger and not self._danger_prev:
            self._try_cancel_goal()
        self._danger_prev = danger

    # ────────────────────────────────────────────────────────────────────────
    def _try_cancel_goal(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cancel_time < self.CANCEL_COOLDOWN:
            return
        self._last_cancel_time = now

        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)    
        self.get_logger().warn('[SAFETY] Published emergency_stop=True')

    # ────────────────────────────────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
        """Gate cmd_vel: chặn nếu đang có nguy hiểm."""
        if self._is_danger():
            self.cmdvel_pub.publish(Twist())
        else:
            self.cmdvel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()