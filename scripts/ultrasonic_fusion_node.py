#!/usr/bin/env python3
"""
ultrasonic_fusion_node.py — v3

Thay đổi so với v2:
  1. us_top/us_bot: so sánh với baseline floor distance thay vì threshold tuyệt đối
     - Ngắn hơn baseline - margin → vật cản trên sàn → obstacle
     - Dài hơn baseline + margin → drop/hố → danger
  2. us_mid: giữ nguyên logic phát hiện vật cản phía trước (ngóc lên 15°)
  3. Fix SENSOR_PITCH cho đúng với URDF thực tế
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import math
import numpy as np
from collections import deque
from typing import Tuple


# ── Góc pitch thực tế từ URDF ──────────────────────────────────────────────
# Pitch dương (+rpy_y) = trục X chúi XUỐNG (right-hand rule quanh Y)
# us_top:  rpy="0 +0.2618 0" → +15° → chúi xuống
# us_mid:  rpy="0 -0.2618 0" → -15° → ngóc lên
# us_bot:  rpy="0 +0.2618 0" → +15° → chúi xuống
SENSOR_PITCH = {
    'us_top_left':   math.radians(15.0),   # chúi xuống
    'us_top_right':  math.radians(15.0),   # chúi xuống
    'us_mid_left':   math.radians(-15.0),  # ngóc lên
    'us_mid_right':  math.radians(-15.0),  # ngóc lên
    'us_bot_left':   math.radians(15.0),   # chúi xuống
    'us_bot_right':  math.radians(15.0),   # chúi xuống
}

# ── Baseline: khoảng cách sensor → sàn khi không có vật cản ────────────────
# Tính bằng: z_sensor / sin(pitch_angle)
WHEEL_RADIUS = 0.08255  # m — từ URDF

# z_chassis là z của sensor joint so với chassis link
Z_TOP = 0.49 + WHEEL_RADIUS   # = 0.5726m so với mặt đất
Z_BOT = 0.03 + WHEEL_RADIUS   # = 0.1126m so với mặt đất

PITCH_DEG = 15.0

FLOOR_BASELINE = {
    'us_top_left':  Z_TOP / math.sin(math.radians(PITCH_DEG)),  # 0.5726/sin(15°) ≈ 2.213m
    'us_top_right': Z_TOP / math.sin(math.radians(PITCH_DEG)),  # ≈ 2.213m
    'us_bot_left':  Z_BOT / math.sin(math.radians(PITCH_DEG)),  # 0.1126/sin(15°) ≈ 0.435m
    'us_bot_right': Z_BOT / math.sin(math.radians(PITCH_DEG)),  # ≈ 0.435m
}

class UltrasonicFusionNode(Node):

    # ── Thông số an toàn ────────────────────────────────────────────────────
    OBSTACLE_THRESHOLD = 0.60   # m — us_mid: phát hiện vật cản phía trước
    
    # us_top/bot: margin so với baseline
    FLOOR_OBSTACLE_MARGIN = 0.25  # m — ngắn hơn baseline X m → vật cản trên sàn
    DROP_MARGIN           = 0.15  # m — dài hơn baseline X m → drop/hố

    FILTER_WINDOW      = 5
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

        # ── Góc của từng sensor trong LaserScan ──────────────────────────────
        # Chỉ us_mid và us_top đưa vào LaserScan (us_bot chỉ drop detection)
        # us_top chiếu xuống sàn nên projected horizontal distance
        # us_mid ngóc lên → phát hiện vật thể cao
        self.sensor_angles = {
            'us_top_left':  ( math.radians(26),  SENSOR_PITCH['us_top_left']),
            'us_top_right': ( math.radians(-26), SENSOR_PITCH['us_top_right']),
            'us_mid_left':  ( math.radians(26),  SENSOR_PITCH['us_mid_left']),
            'us_mid_right': ( math.radians(-26), SENSOR_PITCH['us_mid_right']),
        }

        self._last_cancel_time = 0.0
        self._danger_prev      = False
        self._robot_navigating = False

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

        self.create_subscription(Twist, '/cmd_vel_nav', self._cmdvel_cb, 10)
        self.create_timer(0.1, self._publish_scan)

        # Log baseline để verify
        for name, baseline in FLOOR_BASELINE.items():
            self.get_logger().info(
                f'Baseline {name}: {baseline:.3f}m | '
                f'obstacle if < {baseline - self.FLOOR_OBSTACLE_MARGIN:.3f}m | '
                f'drop if > {baseline + self.DROP_MARGIN:.3f}m'
            )

        self.get_logger().info('ultrasonic_fusion_node v3 started ✓')

    def _on_robot_state(self, msg):
        self._robot_navigating = msg.data in ('NAVIGATING', 'RETURNING_HOME')

    # ────────────────────────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        """Lưu raw range (không compensate pitch ở đây, xử lý tại điểm dùng)."""
        if msg.min_range <= msg.range <= msg.max_range:
            self.buffers[name].append(msg.range)
        # Nếu đọc max_range (không có vật cản trong tầm) → lưu inf
        elif msg.range >= msg.max_range:
            self.buffers[name].append(float('inf'))
        
        if self.buffers[name]:
            vals = [v for v in self.buffers[name] if v != float('inf')]
            if vals:
                self.latest[name] = float(np.median(vals))
            else:
                self.latest[name] = float('inf')

    # ────────────────────────────────────────────────────────────────────────
    def _is_floor_sensor_obstacle(self, name: str) -> Tuple[bool, str]:
        """
        Kiểm tra us_top/us_bot có phát hiện vật cản không.
        
        Returns: (is_danger, reason)
        - range << baseline → vật cản trên sàn (có thứ gì đó chặn beam trước khi chạm đất)
        - range >> baseline → drop/hố (beam đi xa hơn bình thường)
        - range ≈ baseline → bình thường, bỏ qua
        """
        raw = self.latest[name]
        if raw == float('inf'):
            return False, ''
        
        baseline = FLOOR_BASELINE[name]
        
        # Vật cản trên sàn: beam bị chặn sớm hơn baseline
        if raw < baseline - self.FLOOR_OBSTACLE_MARGIN:
            return True, f'FLOOR_OBSTACLE — {name}: {raw:.3f}m (baseline={baseline:.3f}m)'
        
        # Drop/hố: beam đi xa hơn baseline
        if raw > baseline + self.DROP_MARGIN:
            return True, f'DROP — {name}: {raw:.3f}m (baseline={baseline:.3f}m)'
        
        return False, ''

    # ────────────────────────────────────────────────────────────────────────
    def _is_danger(self) -> bool:
        # Kiểm tra us_bot (drop detection + floor obstacle, độ cao thấp)
        for name in ['us_bot_left', 'us_bot_right']:
            danger, reason = self._is_floor_sensor_obstacle(name)
            if danger:
                self.get_logger().warn(f'[SAFETY] {reason}', throttle_duration_sec=1.0)
                return True

        # Kiểm tra us_top (floor obstacle ở tầm cao hơn)
        for name in ['us_top_left', 'us_top_right']:
            danger, reason = self._is_floor_sensor_obstacle(name)
            if danger:
                self.get_logger().warn(f'[SAFETY] {reason}', throttle_duration_sec=1.0)
                return True

        # Kiểm tra us_mid (vật cản phía trước, tầm trung-cao)
        for name in ['us_mid_left', 'us_mid_right']:
            raw = self.latest[name]
            if raw != float('inf') and raw < self.OBSTACLE_THRESHOLD:
                self.get_logger().warn(
                    f'[SAFETY] OBSTACLE — {name}: {raw:.3f}m',
                    throttle_duration_sec=1.0)
                return True

        return False

    # ────────────────────────────────────────────────────────────────────────
    def _publish_scan(self):
        """
        Chỉ đưa us_mid vào LaserScan để costmap xử lý obstacle phía trước.
        us_top đưa vào với điều kiện: chỉ khi phát hiện floor obstacle
        (tức range < baseline - margin), không phải khi đọc baseline bình thường.
        """
        num_rays = 360
        ranges   = [float('inf')] * num_rays
        spread   = 8

        for name, (angle_rad, pitch) in self.sensor_angles.items():
            raw = self.latest[name]
            if raw == float('inf'):
                continue

            # us_top: chỉ đưa vào scan khi có floor obstacle, không phải đọc sàn bình thường
            if name in ('us_top_left', 'us_top_right'):
                baseline = FLOOR_BASELINE[name]
                if raw >= baseline - self.FLOOR_OBSTACLE_MARGIN:
                    # Đọc bình thường (sàn) → bỏ qua, không đưa vào costmap
                    continue
                # Có vật cản → tính horizontal distance
                dist = raw * math.cos(abs(pitch))
            else:
                # us_mid: compensate pitch để lấy horizontal distance
                dist = raw * math.cos(abs(pitch))

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
        scan.range_min       = 0.01
        scan.range_max       = 2.5
        scan.ranges          = ranges

        self.scan_pub.publish(scan)

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

        if not hasattr(self, '_estop_pub'):
            from std_msgs.msg import Bool as BoolMsg
            self._estop_pub = self.create_publisher(BoolMsg, '/robot/emergency_stop', 10)

        from std_msgs.msg import Bool as BoolMsg
        msg = BoolMsg()
        msg.data = True
        self._estop_pub.publish(msg)
        self.get_logger().warn('[SAFETY] Published emergency_stop=True')

    # ────────────────────────────────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
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