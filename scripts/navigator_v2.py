#!/usr/bin/env python3
"""
navigator.py  ── v2  (Pre-Rotate Edition)
==========================================
Thêm 2 trạng thái mới vào state machine:
  COMPUTING_PATH → PRE_ROTATING → NAVIGATING

Luồng mới:
  1. Khi nhận lệnh navigate, gọi ComputePathToPose action để lấy global path.
  2. Trích heading tiếp tuyến đoạn đầu path tại lookahead ≈ 0.5m.
  3. Nếu |heading_error| > 0.40 rad (~23°) → xoay tại chỗ bằng cmd_vel trực tiếp.
  4. Khi căn hướng xong (hoặc timeout) → gửi NavigateToPose như bình thường.

Fallback: Nếu planner thất bại / path quá ngắn → bỏ qua pre-rotate,
          điều hướng thẳng (hành vi giống v1).

Topics mới:
  pub: /cmd_vel (Twist) — chỉ trong trạng thái PRE_ROTATING
  (các topic khác giữ nguyên từ v1)
"""

import math
import os
import time
from enum import Enum

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
import yaml

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32, String


# ============================================================
#  TUNING PARAMETERS  — chỉnh ở đây, không cần sửa code bên dưới
# ============================================================

# Ngưỡng góc lệch để kích hoạt pre-rotate (rad).
# < ngưỡng này → bỏ qua, điều hướng thẳng.
PRE_ROTATE_THRESHOLD = 0.40     # rad  (~23°)

# Sai số góc coi là "đã căn xong" (rad).
PRE_ROTATE_STOP_THR  = 0.05    # rad  (~3°)

# Khoảng cách lookahead trên path để tính heading tiếp tuyến ban đầu (m).
PRE_ROTATE_LOOKAHEAD = 0.50    # m

# P gain cho bộ điều khiển xoay tại chỗ.
PRE_ROTATE_KP        = 1.5

# Vận tốc góc tối đa khi xoay tại chỗ (rad/s).
PRE_ROTATE_MAX_W     = 0.80    # rad/s

# Vận tốc góc tối thiểu khi xoay (để thắng ma sát tĩnh) (rad/s).
PRE_ROTATE_MIN_W     = 0.15    # rad/s

# Timeout cho pre-rotate — sau thời gian này dù chưa căn xong vẫn tiến tới nav (s).
PRE_ROTATE_TIMEOUT   = 12.0    # s

# ============================================================


class State(Enum):
    IDLE           = "IDLE"
    COMPUTING_PATH = "COMPUTING_PATH"   # ← NEW: chờ planner trả path
    PRE_ROTATING   = "PRE_ROTATING"     # ← NEW: xoay tại chỗ về hướng ban đầu của path
    NAVIGATING     = "NAVIGATING"
    AT_CHECKPOINT  = "AT_CHECKPOINT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RETURNING_HOME = "RETURNING_HOME"


class CheckpointNavigator(Node):

    def __init__(self):
        super().__init__("checkpoint_navigator")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("checkpoint_file",       "")
        self.declare_parameter("timeout_at_checkpoint", 30.0)
        self.declare_parameter("home_checkpoint_id",    0)

        self.timeout = self.get_parameter("timeout_at_checkpoint").value
        self.home_id = self.get_parameter("home_checkpoint_id").value

        self.checkpoints = self._load_checkpoints()
        if not self.checkpoints:
            self.get_logger().error("No checkpoints loaded. Navigator exiting.")
            return

        # ── State variables ──────────────────────────────────────────────────
        self.state        = State.IDLE
        self.current_cp   = -1
        self.target_cp    = None
        self.goal_handle  = None
        self.arrival_time = None

        # Pre-rotate specific
        self._target_yaw       = 0.0     # desired heading (rad, map frame)
        self._pre_rotate_start = None    # time.monotonic() when rotation began
        self._pending_cp_id    = None    # checkpoint waiting for rotation to finish

        # ── TF (needed to get current robot yaw for pre-rotate) ──────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Action clients ───────────────────────────────────────────────────
        self._nav     = ActionClient(self, NavigateToPose,    "navigate_to_pose")
        self._planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.get_logger().info("Waiting for Nav2 action servers...")
        self._nav.wait_for_server()
        self._planner.wait_for_server()
        self.get_logger().info("Nav2 action servers ready.")

        # ── Publishers ───────────────────────────────────────────────────────
        self._state_pub  = self.create_publisher(String, "/robot/state",              10)
        self._cp_pub     = self.create_publisher(Int32,  "/robot/current_checkpoint", 10)
        self._status_pub = self.create_publisher(String, "/robot/status_message",     10)
        # cmd_vel publisher — chỉ dùng trong PRE_ROTATING; Nav2 controller_server
        # sẽ không cạnh tranh vì chưa nhận NavigateToPose goal lúc này.
        self._cmdvel_pub = self.create_publisher(Twist,  "/cmd_vel",                  10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Int32, "/robot/navigate_to_checkpoint", self._on_nav_command, 10)
        self.create_subscription(
            Bool, "/robot/emergency_stop", self._on_estop, 10)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(0.1, self._state_machine)   # 10 Hz — main loop + pre-rotate tick
        self.create_timer(1.0, self._publish_state)   # 1 Hz  — state broadcast

        self.get_logger().info(
            f"CheckpointNavigator v2 ready. "
            f"{len(self.checkpoints)} checkpoints. "
            f"Pre-rotate threshold = {math.degrees(PRE_ROTATE_THRESHOLD):.1f}°  "
            f"lookahead = {PRE_ROTATE_LOOKAHEAD} m")

    # ================================================================
    #  CHECKPOINT LOADING  (unchanged from v1)
    # ================================================================
    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg  = get_package_share_directory("mobile_robot")
                path = os.path.join(pkg, "config", "checkpoints.yaml")
            except Exception:
                self.get_logger().error("Package 'mobile_robot' not found.")
                return {}

        if not os.path.exists(path):
            self.get_logger().error(f"Checkpoint file not found: {path}")
            return {}

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        result = {}
        for cp in data.get("checkpoints", []):
            pose = PoseStamped()
            pose.header.frame_id    = data.get("frame_id", "map")
            pose.pose.position.x    = float(cp["position"]["x"])
            pose.pose.position.y    = float(cp["position"]["y"])
            pose.pose.position.z    = float(cp["position"].get("z", 0.0))
            pose.pose.orientation.x = float(cp["orientation"].get("x", 0.0))
            pose.pose.orientation.y = float(cp["orientation"].get("y", 0.0))
            pose.pose.orientation.z = float(cp["orientation"].get("z", 0.0))
            pose.pose.orientation.w = float(cp["orientation"].get("w", 1.0))
            result[cp["id"]] = {
                "id":   cp["id"],
                "name": cp.get("display_name", cp["name"]),
                "pose": pose,
            }
        self.get_logger().info(f"Loaded {len(result)} checkpoints from {path}")
        return result

    # ================================================================
    #  TOPIC CALLBACKS
    # ================================================================
    def _on_nav_command(self, msg: Int32):
        cp_id = msg.data

        if cp_id not in self.checkpoints:
            self.get_logger().error(
                f"Checkpoint {cp_id} not found. Valid IDs: {list(self.checkpoints.keys())}")
            return

        # Block commands while robot is busy (includes new COMPUTING_PATH, PRE_ROTATING)
        busy_states = (
            State.NAVIGATING, State.RETURNING_HOME,
            State.COMPUTING_PATH, State.PRE_ROTATING
        )
        if self.state in busy_states:
            self.get_logger().warn(
                f"Command rejected: busy in state '{self.state.name}' "
                f"(navigating to [{self.target_cp}]).")
            self._pub_status(f"Busy [{self.state.name}]. Ignored goal {cp_id}.")
            return

        if self.state == State.EMERGENCY_STOP:
            self.get_logger().warn(
                "Emergency stop active. Publish False to /robot/emergency_stop first.")
            return

        if self.state in (State.IDLE, State.AT_CHECKPOINT) and self.current_cp == cp_id:
            self.get_logger().info(f"Already at checkpoint [{cp_id}].")
            return

        # All clear — start navigation pipeline
        self._request_navigation(cp_id)

    def _on_estop(self, msg: Bool):
        if msg.data:
            if self.state != State.EMERGENCY_STOP:
                self.get_logger().warn("!!! EMERGENCY STOP activated !!!")
                self.state      = State.EMERGENCY_STOP
                self.current_cp = -1
                self._stop_robot()
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                self._pub_status("EMERGENCY STOP")
        else:
            if self.state == State.EMERGENCY_STOP:
                self.get_logger().info("Emergency stop reset → IDLE.")
                self.state       = State.IDLE
                self.goal_handle = None
                self._pub_status("Ready.")

    # ================================================================
    #  STATE MACHINE  (10 Hz)
    # ================================================================
    def _state_machine(self):

        # ── AT_CHECKPOINT: check timeout to return home ────────────────────
        if self.state == State.AT_CHECKPOINT and self.arrival_time is not None:
            if time.time() - self.arrival_time >= self.timeout:
                self.get_logger().info(
                    f"Timeout at [{self.current_cp}]. Returning home.")
                self.state = State.RETURNING_HOME
                self._request_navigation(self.home_id)
            return

        # ── PRE_ROTATING: P-controller tick ───────────────────────────────
        if self.state == State.PRE_ROTATING:
            self._pre_rotate_tick()

        # Other states (IDLE, NAVIGATING, COMPUTING_PATH, EMERGENCY_STOP,
        # RETURNING_HOME) don't need active polling here.

    # ================================================================
    #  NAVIGATION ENTRY POINT
    # ================================================================
    def _request_navigation(self, cp_id: int):
        """
        Entry point for ALL navigation requests (both external commands and
        internal timeout-triggered home return).

        Instead of sending NavigateToPose immediately, we first ask the planner
        for a path so we can extract the initial heading tangent.
        """
        self._pending_cp_id = cp_id
        self.target_cp      = cp_id
        self.state          = State.COMPUTING_PATH

        cp_name = self.checkpoints[cp_id]["name"]
        self.get_logger().info(
            f"[COMPUTING_PATH] Requesting global path to [{cp_id}] {cp_name}...")
        self._pub_status(f"Computing path to [{cp_id}] {cp_name}...")

        goal_pose           = self.checkpoints[cp_id]["pose"]
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        compute_goal            = ComputePathToPose.Goal()
        compute_goal.pose       = goal_pose
        compute_goal.planner_id = ""   # "" → use default planner (NavFn / A*)
        # NOTE: In Foxy, ComputePathToPose.Goal does NOT have a 'start' field
        # (that was added in later Nav2 versions). The planner uses the
        # current TF position of the robot automatically.

        future = self._planner.send_goal_async(compute_goal)
        future.add_done_callback(self._on_path_goal_response)

    # ── Step 1: planner accepted/rejected the goal ─────────────────────────
    def _on_path_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "ComputePathToPose goal REJECTED. "
                "Falling back to direct NavigateToPose (no pre-rotate).")
            self._send_nav_goal(self._pending_cp_id)
            return
        goal_handle.get_result_async().add_done_callback(self._on_path_result)

    # ── Step 2: planner returned a path ────────────────────────────────────
    def _on_path_result(self, future):
        result = future.result()

        # Planner failed
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"ComputePathToPose failed (status={result.status}). "
                "Falling back to direct nav (no pre-rotate).")
            self._send_nav_goal(self._pending_cp_id)
            return

        path: Path = result.result.path

        # Path too short to extract a meaningful heading
        if len(path.poses) < 2:
            self.get_logger().warn(
                "Path has < 2 poses (goal very close?). Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Get current robot pose from TF
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                "TF unavailable — cannot compute heading error. Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        rx, ry, ryaw = robot_pose

        # Extract the initial heading tangent from the path
        initial_heading = self._extract_initial_heading(path, rx, ry)
        heading_error   = self._normalize_angle(initial_heading - ryaw)

        self.get_logger().info(
            f"Path initial heading: {math.degrees(initial_heading):.1f}° | "
            f"Robot yaw: {math.degrees(ryaw):.1f}° | "
            f"Error: {math.degrees(heading_error):.1f}°")

        # Below threshold — no need to pre-rotate
        if abs(heading_error) < PRE_ROTATE_THRESHOLD:
            self.get_logger().info(
                f"Heading error {math.degrees(heading_error):.1f}° < "
                f"threshold {math.degrees(PRE_ROTATE_THRESHOLD):.1f}°. "
                "Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Start pre-rotation
        self._target_yaw       = initial_heading
        self._pre_rotate_start = time.monotonic()
        self.state             = State.PRE_ROTATING

        self.get_logger().info(
            f"[PRE_ROTATING] Rotating {math.degrees(heading_error):.1f}° "
            f"to align with path tangent...")
        self._pub_status(
            f"Pre-rotating {math.degrees(heading_error):.0f}° "
            f"→ [{self._pending_cp_id}] {self.checkpoints[self._pending_cp_id]['name']}")

    # ================================================================
    #  PRE-ROTATE CONTROL LOOP
    #  Called at 10 Hz from _state_machine while state == PRE_ROTATING
    # ================================================================
    def _pre_rotate_tick(self):
        # ── Timeout guard ─────────────────────────────────────────────────
        elapsed = time.monotonic() - self._pre_rotate_start
        if elapsed > PRE_ROTATE_TIMEOUT:
            self.get_logger().warn(
                f"Pre-rotate TIMEOUT ({PRE_ROTATE_TIMEOUT:.0f}s). "
                "Proceeding to navigate anyway.")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        # ── Get current heading ────────────────────────────────────────────
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            # TF not ready yet — keep previous cmd_vel and wait
            return

        _, _, ryaw    = robot_pose
        heading_error = self._normalize_angle(self._target_yaw - ryaw)

        # ── Convergence check ─────────────────────────────────────────────
        if abs(heading_error) < PRE_ROTATE_STOP_THR:
            self.get_logger().info(
                f"[PRE_ROTATING] Converged. "
                f"Residual error: {math.degrees(heading_error):.2f}° "
                f"after {elapsed:.1f}s.")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        # ── P controller with velocity clamping ───────────────────────────
        # raw_w = Kp * error (can be any magnitude)
        # clamp to [MIN_W, MAX_W] while preserving sign
        raw_w = PRE_ROTATE_KP * heading_error
        sign  = 1.0 if raw_w >= 0.0 else -1.0
        w     = sign * max(PRE_ROTATE_MIN_W, min(PRE_ROTATE_MAX_W, abs(raw_w)))

        # Publish pure rotation command (no linear velocity!)
        twist             = Twist()
        twist.linear.x    = 0.0
        twist.angular.z   = w
        self._cmdvel_pub.publish(twist)

        # Debug log every ~1 s (every 10 ticks)
        tick_count = int(elapsed / 0.1)
        if tick_count % 10 == 0:
            self.get_logger().debug(
                f"[PRE_ROTATING] error={math.degrees(heading_error):.1f}°  "
                f"w={w:.2f} rad/s  elapsed={elapsed:.1f}s")

    # ================================================================
    #  SEND NAV2 GOAL  (called after pre-rotate finishes or is skipped)
    # ================================================================
    def _send_nav_goal(self, cp_id: int):
        """
        Sends NavigateToPose to Nav2. At this point the robot is already
        approximately aligned with the initial path tangent, so the
        controller should move forward cleanly.
        """
        self.target_cp = cp_id
        self.state     = State.NAVIGATING

        pose              = self.checkpoints[cp_id]["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"[NAVIGATING] → [{cp_id}] {self.checkpoints[cp_id]['name']} "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}")
        self._pub_status(
            f"Navigating to [{cp_id}] {self.checkpoints[cp_id]['name']}")

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal REJECTED by Nav2.")
            self.state = State.IDLE
            return
        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.current_cp   = self.target_cp
            self.arrival_time = time.time()
            name = self.checkpoints[self.current_cp]["name"]

            if self.state == State.RETURNING_HOME or self.current_cp == self.home_id:
                self.state = State.IDLE
                self.get_logger().info(
                    f"Arrived at Home [{self.current_cp}] '{name}'. State: IDLE.")
                self._pub_status(f"At Home [{self.current_cp}]. IDLE.")
            else:
                self.state = State.AT_CHECKPOINT
                self.get_logger().info(
                    f"Arrived at [{self.current_cp}] '{name}'. "
                    f"Returning home in {self.timeout:.0f}s.")
                self._pub_status(
                    f"At [{self.current_cp}] '{name}'. "
                    f"Returning home in {self.timeout:.0f}s.")

        elif status == GoalStatus.STATUS_CANCELED:
            self.current_cp = -1
            if self.state != State.EMERGENCY_STOP:
                self.state = State.IDLE
                self.get_logger().info("Goal canceled. State: IDLE.")

        elif status == GoalStatus.STATUS_ABORTED:
            self.current_cp = -1
            self.get_logger().error(
                f"Navigation ABORTED to [{self.target_cp}]. "
                "Check costmap / path planner.")
            self.state = State.IDLE
            self._pub_status(f"Aborted. Could not reach [{self.target_cp}].")

    # ================================================================
    #  UTILITY METHODS
    # ================================================================
    def _extract_initial_heading(self, path: Path, rx: float, ry: float) -> float:
        """
        Walks along the path poses and returns the heading (in map frame)
        from the robot to the first pose that is at least PRE_ROTATE_LOOKAHEAD
        meters away.

        Edge cases handled:
          - Path entirely within lookahead (very close goal): use last pose.
          - Only 1 pose: use heading robot→that pose.
        """
        poses = path.poses

        if len(poses) == 1:
            # degenerate path
            px = poses[0].pose.position.x
            py = poses[0].pose.position.y
            return math.atan2(py - ry, px - rx)

        # Walk the path until distance from robot exceeds lookahead
        for pose_stamped in poses:
            px   = pose_stamped.pose.position.x
            py   = pose_stamped.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist >= PRE_ROTATE_LOOKAHEAD:
                return math.atan2(py - ry, px - rx)

        # All path points are within lookahead → use last point
        last_pose = poses[-1].pose.position
        self.get_logger().debug(
            "All path points within lookahead — using last path point for heading.")
        return math.atan2(last_pose.y - ry, last_pose.x - rx)

    def _get_robot_pose(self):
        """
        Returns (x, y, yaw_rad) of base_footprint in map frame.
        Returns None if TF is not yet available.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "map",            # target frame
                "base_footprint", # source frame
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(
                f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f"TF extrapolation failed: {e}", throttle_duration_sec=2.0)
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        # quaternion → yaw  (standard formula)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw       = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to (−π, π]."""
        while angle >  math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _stop_robot(self):
        """Publishes a zero-velocity Twist to stop any in-place rotation."""
        self._cmdvel_pub.publish(Twist())

    # ================================================================
    #  STATE PUBLISHERS
    # ================================================================
    def _publish_state(self):
        m = String(); m.data = self.state.value;  self._state_pub.publish(m)
        m = Int32();  m.data = self.current_cp;   self._cp_pub.publish(m)

    def _pub_status(self, message: str):
        m = String(); m.data = message;           self._status_pub.publish(m)


# ================================================================
#  MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CheckpointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
