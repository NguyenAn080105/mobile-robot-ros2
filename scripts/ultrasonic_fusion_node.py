#!/usr/bin/env python3
"""
ultrasonic_fusion_node.py

Nhận sensor_msgs/Range từ 6 HC-SR04:
  /ultrasonic/us_front, us_left, us_right, us_rear  → fuse thành LaserScan
  /ultrasonic/us_drop_left, us_drop_right           → drop-off detection

Output:
  /ultrasonic_scan  (sensor_msgs/LaserScan) → Nav2 local costmap
  /safety_stop      (std_msgs/Bool)         → emergency logic
  /cmd_vel          (geometry_msgs/Twist)   → safety relay từ /cmd_vel_nav
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math
import numpy as np
from collections import deque


class UltrasonicFusionNode(Node):

    # ── Thông số an toàn ────────────────────────────────────────────────────
    OBSTACLE_THRESHOLD = 0.35   # m — vật cản nguy hiểm phía ngang
    DROP_FLOOR_DIST    = 0.32   # m — khoảng cách sensor → sàn bình thường (calibrate thực tế)
    DROP_MARGIN        = 0.08   # m — nếu đọc > DROP_FLOOR_DIST + DROP_MARGIN → có cạnh
    FRONT_STOP_DIST    = 0.20   # m — dừng khẩn cấp nếu phía trước < giá trị này
    FILTER_WINDOW      = 5      # median filter window size

    def __init__(self):
        super().__init__('ultrasonic_fusion_node')

        # ── Median filter buffers ────────────────────────────────────────────
        self.sensor_names = [
            'us_top_left',  'us_top_right',
            'us_mid_left',  'us_mid_right',
            'us_bot_left',  'us_bot_right',
        ]
        self.buffers = {n: deque(maxlen=self.FILTER_WINDOW) for n in self.sensor_names}
        self.latest  = {n: float('inf') for n in self.sensor_names}

        # ── Góc của 4 sensors ngang trong LaserScan 360° ────────────────────
        self.sensor_angles = {
            'us_mid_left':  math.pi / 2,   # 90° trái
            'us_mid_right': -math.pi / 2,  # 90° phải
            # Top và Bot có góc pitch → không map được vào LaserScan 2D
            # chúng chỉ dùng cho safety logic
        }

        # ── Subscribers: 6 Range topics ─────────────────────────────────────
        for name in self.sensor_names:
            self.create_subscription(
                Range,
                f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n),
                10
            )

        # ── Publishers ───────────────────────────────────────────────────────
        self.scan_pub   = self.create_publisher(LaserScan, '/ultrasonic_scan', 10)
        self.safety_pub = self.create_publisher(Bool,      '/safety_stop',     10)
        self.cmdvel_pub = self.create_publisher(Twist,     '/cmd_vel',         10)

        # ── Subscriber: nhận cmd_vel từ Nav2, relay qua safety check ────────
        # Nav2 sẽ được remap publish ra /cmd_vel_nav (xem nav_launch.py)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmdvel_cb, 10)

        # ── Timer 10 Hz ──────────────────────────────────────────────────────
        self.create_timer(0.1, self._publish_scan)

        self.get_logger().info('ultrasonic_fusion_node started ✓')
        self.get_logger().info(
            f'Thresholds — front_stop: {self.FRONT_STOP_DIST}m | '
            f'drop: >{self.DROP_FLOOR_DIST + self.DROP_MARGIN}m'
        )

    # ────────────────────────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        """Lọc giá trị hợp lệ, áp median filter."""
        if msg.min_range <= msg.range <= msg.max_range:
            self.buffers[name].append(msg.range)
        if self.buffers[name]:
            self.latest[name] = float(np.median(list(self.buffers[name])))

    # ────────────────────────────────────────────────────────────────────────
    def _publish_scan(self):
        """
        Convert 4 sensors ngang → LaserScan 360° → /ultrasonic_scan
        Nav2 local costmap subscribe topic này như một observation source.
        """
        num_rays = 360
        ranges   = [float('inf')] * num_rays
        spread   = 8  # ±8° xung quanh tâm sensor (xấp xỉ cone 15°)

        for name, angle_rad in self.sensor_angles.items():
            dist = self.latest[name]
            if dist == float('inf'):
                continue
            center_idx = int(round(math.degrees(angle_rad))) % num_rays
            for offset in range(-spread, spread + 1):
                idx = (center_idx + offset) % num_rays
                # Khoảng cách tăng ở rìa cone
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
        scan.range_min       = 0.02
        scan.range_max       = 4.0
        scan.ranges          = ranges

        self.scan_pub.publish(scan)

        # Publish safety status mỗi lần scan
        self.safety_pub.publish(Bool(data=self._is_danger()))

    # ────────────────────────────────────────────────────────────────────────
    def _is_danger(self) -> bool:
        # Bot: phát hiện drop-off (đọc > ngưỡng = không có sàn)
        drop_threshold = self.DROP_FLOOR_DIST + self.DROP_MARGIN
        for name in ['us_bot_left', 'us_bot_right']:
            if self.latest[name] > drop_threshold:
                self.get_logger().warn(
                    f'[SAFETY] DROP — {name}: {self.latest[name]:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True

        # Top: phát hiện vật cản thấp xa phía trước
        for name in ['us_top_left', 'us_top_right']:
            if self.latest[name] < self.OBSTACLE_THRESHOLD:
                self.get_logger().warn(
                    f'[SAFETY] OBSTACLE TOP — {name}: {self.latest[name]:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True

        # Mid: phát hiện vật treo gầm bàn
        for name in ['us_mid_left', 'us_mid_right']:
            if self.latest[name] < self.OBSTACLE_THRESHOLD:
                self.get_logger().warn(
                    f'[SAFETY] OBSTACLE MID — {name}: {self.latest[name]:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True

        return False

    # ────────────────────────────────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
        """
        Safety relay:
          - Nếu an toàn    → forward cmd_vel từ Nav2 xuống robot
          - Nếu nguy hiểm  → publish Twist(0,0,0) để dừng robot ngay
        """
        if self._is_danger():
            self.cmdvel_pub.publish(Twist())  # zero velocity
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