#!/usr/bin/env python3
"""
ultrasonic_fusion_node.py — v2

Thay đổi so với v1:
  1. Safety: cancel Nav2 goal thay vì chèn Twist(0) (tránh race condition với controller_server)
  2. Tăng ngưỡng phát hiện: 0.6m thay vì 0.35m cho phép robot lớn có đủ thời gian replan
  3. Fix khoảng cách: compensate góc pitch của từng sensor (cos projection)
  4. Thêm us_top_left/right vào LaserScan (projected xuống mặt phẳng ngang)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import math
import numpy as np
from collections import deque


# ── Góc pitch thực tế từ URDF (rpy="0 pitch 0") ───────────────────────────
# us_top: rpy="0 0.4363 0"  → pitch = +25°  (ngóc lên → chiếu xuống ngang: *cos(25°))
# us_mid: rpy="0 -0.2618 0" → pitch = -15°  (chúc xuống → chiếu ra ngang: *cos(15°))
# us_bot: rpy="0 0.2618 0"  → pitch = +15°  (hướng xuống sàn, không dùng cho scan)
SENSOR_PITCH = {
    'us_top_left':   math.radians(25.0),
    'us_top_right':  math.radians(25.0),
    'us_mid_left':   math.radians(-15.0),
    'us_mid_right':  math.radians(-15.0),
    'us_bot_left':   math.radians(15.0),
    'us_bot_right':  math.radians(15.0),
}


class UltrasonicFusionNode(Node):

    # ── Thông số an toàn ────────────────────────────────────────────────────
    # Tăng ngưỡng vì robot lớn (chassis ~0.5m) cần phát hiện sớm để replan
    OBSTACLE_THRESHOLD = 0.60   # m — tăng từ 0.35 lên 0.60 (detection margin)
    DROP_FLOOR_DIST    = 0.32   # m — khoảng cách sensor → sàn bình thường
    DROP_MARGIN        = 0.08   # m — tolerance
    FILTER_WINDOW      = 5      # median filter

    # Thời gian cooldown (s) sau khi cancel goal, tránh spam cancel
    CANCEL_COOLDOWN    = 2.0

    def __init__(self):
        super().__init__('ultrasonic_fusion_node')

        self.sensor_names = [
            'us_top_left',  'us_top_right',
            'us_mid_left',  'us_mid_right',
            'us_bot_left',  'us_bot_right',
        ]
        self.buffers = {n: deque(maxlen=self.FILTER_WINDOW) for n in self.sensor_names}
        self.latest  = {n: float('inf') for n in self.sensor_names}
        # Guard: skip _is_danger() for sensors that have never sent a reading.
        # Without this, inf > drop_threshold triggers a false DROP immediately.
        self._received = {n: False for n in self.sensor_names}

        # ── Góc và pitch của từng sensor trong LaserScan ─────────────────────
        # (angle_in_laserscan, pitch_angle_from_urdf)
        # Top/Mid đều ở phía trước-trái và phía trước-phải nhưng lệch nhau
        # Từ URDF: sensors nằm ở x=0.35, y=±0.17 → góc ~±26° so với trục X robot
        # Ở đây dùng sensor theo 2 phía trái/phải chứ không phải thẳng trước
        self.sensor_angles = {
            # sensor_name: (góc trong LaserScan [rad], pitch [rad])
            'us_top_left':  ( math.radians(26),   SENSOR_PITCH['us_top_left']),
            'us_top_right': ( math.radians(-26),  SENSOR_PITCH['us_top_right']),
            'us_mid_left':  ( math.radians(26),   SENSOR_PITCH['us_mid_left']),
            'us_mid_right': ( math.radians(-26),  SENSOR_PITCH['us_mid_right']),
        }

        # ── Nav2 action client để cancel goal khi danger ─────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None
        self._last_cancel_time = 0.0
        self._danger_prev = False

        # ── Subscribers: 6 Range topics ─────────────────────────────────────
        for name in self.sensor_names:
            self.create_subscription(
                Range,
                f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n),
                10
            )

        # Goal handle subscriber — lắng nghe goal hiện tại của navigator
        # navigator.py publish goal qua action, ta cần lấy handle để cancel
        # Giải pháp đơn giản: subscribe /robot/state để biết khi nào đang navigate
        self._robot_navigating = False
        self.create_subscription(
            rclpy.impl.rcutils_logger.RcutilsLogger if False else __import__('std_msgs.msg', fromlist=['String']).String,
            '/robot/state',
            self._on_robot_state,
            10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.scan_pub   = self.create_publisher(LaserScan, '/ultrasonic_scan', 10)
        self.safety_pub = self.create_publisher(Bool,      '/safety_stop',     10)
        # Vẫn giữ cmd_vel relay làm backup cho trường hợp không dùng navigator.py
        self.cmdvel_pub = self.create_publisher(Twist,     '/cmd_vel',         10)

        # ── Subscriber: relay cmd_vel (chỉ hoạt động nếu không dùng navigator.py)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmdvel_cb, 10)

        # ── Timer 10 Hz ──────────────────────────────────────────────────────
        self.create_timer(0.1, self._publish_scan)

        self.get_logger().info('ultrasonic_fusion_node v2 started ✓')
        self.get_logger().info(
            f'Thresholds — obstacle: {self.OBSTACLE_THRESHOLD}m | '
            f'drop: >{self.DROP_FLOOR_DIST + self.DROP_MARGIN}m'
        )

    def _on_robot_state(self, msg):
        self._robot_navigating = msg.data in ('NAVIGATING', 'RETURNING_HOME')

    # ────────────────────────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        """Lọc giá trị hợp lệ, áp median filter, compensate pitch."""
        if msg.min_range <= msg.range <= msg.max_range:
            pitch = abs(SENSOR_PITCH.get(name, 0.0))
            # Khoảng cách ngang thực = range * cos(pitch)
            horizontal_dist = msg.range * math.cos(pitch)
            self.buffers[name].append(horizontal_dist)
            self._received[name] = True          # ← ADD THIS
        if self.buffers[name]:
            self.latest[name] = float(np.median(list(self.buffers[name])))

    # ────────────────────────────────────────────────────────────────────────
    def _publish_scan(self):
        """
        Convert sensors ngang → LaserScan 360°
        Đã compensate góc pitch → khoảng cách đúng hơn.
        """
        num_rays = 360
        ranges   = [float('inf')] * num_rays
        spread   = 8  # ±8° cone width

        for name, (angle_rad, pitch) in self.sensor_angles.items():
            dist = self.latest[name]
            if dist == float('inf'):
                continue
            center_idx = int(round(math.degrees(angle_rad))) % num_rays
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
        scan.range_min       = 0.02
        scan.range_max       = 4.0
        scan.ranges          = ranges

        self.scan_pub.publish(scan)

        # Safety check & cancel Nav2 goal nếu cần
        danger = self._is_danger()
        self.safety_pub.publish(Bool(data=danger))

        if danger and not self._danger_prev:
            # Edge: mới chuyển sang danger → cancel goal
            self._try_cancel_goal()
        self._danger_prev = danger

    # ────────────────────────────────────────────────────────────────────────
    def _try_cancel_goal(self):
        """Cancel Nav2 goal hiện tại. Dùng cooldown tránh spam."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cancel_time < self.CANCEL_COOLDOWN:
            return
        self._last_cancel_time = now

        # Cách 1: Dùng action client cancel all goals
        if self._nav_client.server_is_ready():
            self.get_logger().warn('[SAFETY] Cancelling Nav2 goal due to obstacle/drop detection')
            # cancel_all_goals_async() cancel tất cả goal đang pending
            self._nav_client._cancel_goal(None)  # fallback nếu không có handle

        # Cách 2 (reliable hơn): publish emergency stop lên navigator topic
        # để navigator tự cancel goal qua _on_estop
        from std_msgs.msg import Bool as BoolMsg
        if not hasattr(self, '_estop_pub'):
            self._estop_pub = self.create_publisher(BoolMsg, '/robot/emergency_stop', 10)
        msg = BoolMsg()
        msg.data = True
        self._estop_pub.publish(msg)
        self.get_logger().warn('[SAFETY] Published emergency_stop=True to navigator')

    # ────────────────────────────────────────────────────────────────────────
    def _is_danger(self) -> bool:
        """Kiểm tra tất cả sensors với ngưỡng đã nâng lên."""
        drop_threshold = self.DROP_FLOOR_DIST + self.DROP_MARGIN

        for name in ['us_bot_left', 'us_bot_right']:
            if not self._received[name]:          # ← ADD THIS GUARD
                continue
            if self.latest[name] > drop_threshold:
                self.get_logger().warn(
                    f'[SAFETY] DROP — {name}: {self.latest[name]:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True

        for name in ['us_top_left', 'us_top_right', 'us_mid_left', 'us_mid_right']:
            if not self._received[name]:          # ← ADD THIS GUARD
                continue
            if self.latest[name] < self.OBSTACLE_THRESHOLD:
                self.get_logger().warn(
                    f'[SAFETY] OBSTACLE — {name}: {self.latest[name]:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True

        return False

    # ────────────────────────────────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
        """Backup relay: chỉ dùng khi không có navigator.py."""
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