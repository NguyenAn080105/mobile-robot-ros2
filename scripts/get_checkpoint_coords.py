#!/usr/bin/env python3
"""
get_checkpoint_coords.py
=========================
Tool lấy tọa độ checkpoint bằng cách click "2D Nav Goal" trong RViz2.

Cách dùng:
  1. Khởi động simulation + nav2:
       ros2 launch mobile_robot sim_nav_launch.py
  2. Chạy script này:
       ros2 run mobile_robot get_checkpoint_coords.py
  3. Trong RViz2, dùng "2D Nav Goal" (phím N) click vào từng vị trí theo thứ tự:
       [0] Home → [1] Thư Viện → [2] Phòng Họp → [3] Phòng Hiệu Trưởng → [4] CTSV
  4. File checkpoints.yaml sẽ được tạo tự động trong thư mục hiện tại.
     Copy file này vào: ros2_ws/src/mobile_robot/config/checkpoints.yaml

Topic lắng nghe: /goal_pose  (do RViz2 Nav2 plugin publish — khớp với bt_navigator)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml, os, math


# ------------------------------------------------------------------
# Đặt tên checkpoint của bạn ở đây — theo đúng thứ tự click
# ------------------------------------------------------------------
CHECKPOINT_NAMES = [
    ("home",             "Home (Mặc định)"),
    ("library",          "Thư Viện"),
    ("meeting_room",     "Phòng Họp"),
    ("principal_office", "Phòng Hiệu Trưởng"),
    ("student_affairs",  "Phòng Công Tác Sinh Viên"),
]


def quat_to_yaw(z: float, w: float) -> float:
    """Chuyển quaternion (z, w) sang góc yaw (degrees)."""
    return math.degrees(2.0 * math.atan2(z, w))


class CheckpointRecorder(Node):

    def __init__(self):
        super().__init__('checkpoint_recorder')

        # use_sim_time=True để khớp với Gazebo simulation
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.checkpoints    = {}
        self.current_index  = 0

        # /goal_pose là topic do RViz2 Nav2Goal plugin publish
        # Khớp với bt_navigator đang lắng nghe NavigateToPose action
        self.sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self._print_instructions()

    def _print_instructions(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("CHECKPOINT RECORDER")
        self.get_logger().info("Trong RViz2: dùng '2D Nav Goal' để click vào vị trí")
        self.get_logger().info("Thứ tự click:")
        for i, (name, display) in enumerate(CHECKPOINT_NAMES):
            marker = ">>>" if i == self.current_index else "   "
            self.get_logger().info(f"  {marker} [{i}] {display}")
        self.get_logger().info("=" * 60)

    def goal_callback(self, msg: PoseStamped):
        if self.current_index >= len(CHECKPOINT_NAMES):
            self.get_logger().warn("Đã lưu đủ tất cả checkpoints! Ctrl+C để thoát.")
            return

        name, display = CHECKPOINT_NAMES[self.current_index]
        pos = msg.pose.position
        ori = msg.pose.orientation
        yaw = quat_to_yaw(ori.z, ori.w)

        self.checkpoints[self.current_index] = {
            'id':           self.current_index,
            'name':         name,
            'display_name': display,
            'position': {
                'x': round(pos.x, 4),
                'y': round(pos.y, 4),
                'z': 0.0,
            },
            'orientation': {
                'x': round(ori.x, 4),
                'y': round(ori.y, 4),
                'z': round(ori.z, 4),
                'w': round(ori.w, 4),
            }
        }

        self.get_logger().info(
            f"✅ [{self.current_index}] {display}\n"
            f"   x={pos.x:.4f}  y={pos.y:.4f}  yaw={yaw:.1f}°"
        )

        self.current_index += 1
        self._save_yaml()

        if self.current_index < len(CHECKPOINT_NAMES):
            _, next_display = CHECKPOINT_NAMES[self.current_index]
            self.get_logger().info(
                f"  => Tiếp theo: [{self.current_index}] {next_display}"
            )
        else:
            self.get_logger().info("🎉 Hoàn tất! File: checkpoints.yaml")
            self.get_logger().info(
                "Copy vào: ros2_ws/src/mobile_robot/config/checkpoints.yaml"
            )
            self.get_logger().info("Sau đó rebuild: colcon build --packages-select mobile_robot")

    def _save_yaml(self):
        """Lưu checkpoints.yaml — format tương thích với checkpoint_navigator.py"""
        data = {
            # frame_id = 'map' khớp với AMCL global_frame_id trong nav2_params.yaml
            'frame_id': 'map',
            'checkpoints': list(self.checkpoints.values())
        }

        out_path = os.path.join(os.getcwd(), 'checkpoints.yaml')
        with open(out_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(f"  💾 Saved → {out_path}")


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã dừng. Checkpoint file: checkpoints.yaml")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()