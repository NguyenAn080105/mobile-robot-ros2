#!/usr/bin/env python3
"""
navigator.py
========================
Background navigation state machine node.
Does not handle any user input — controlled via ROS topics only.

Topics:
    /robot/navigate_to_checkpoint  (Int32 - input)
    /robot/emergency_stop          (Bool  - input)
    /robot/state                   (String - output)
    /robot/current_checkpoint      (Int32  - output)
    /robot/status_message          (String - output)

Behavior added:
    - If the checkpoint is significantly behind / off-heading relative to the robot,
      the robot first performs an in-place pre-rotation toward the checkpoint.
    - After pre-rotation succeeds, the node sends the original checkpoint goal.
"""

import math
import os
import time
import yaml
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool, Int32

from ament_index_python.packages import get_package_share_directory


class State(Enum):
    IDLE           = "IDLE"
    PRE_ROTATING   = "PRE_ROTATING"
    NAVIGATING     = "NAVIGATING"
    AT_CHECKPOINT  = "AT_CHECKPOINT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RETURNING_HOME = "RETURNING_HOME"


class CheckpointNavigator(Node):

    def __init__(self):
        super().__init__("checkpoint_navigator")

        self.declare_parameter("checkpoint_file", "")
        self.declare_parameter("timeout_at_checkpoint", 30.0)
        self.declare_parameter("home_checkpoint_id", 0)

        # New parameters for pre-rotate behavior
        self.declare_parameter("enable_pre_rotate", True)
        self.declare_parameter("pre_rotate_angle_threshold", 1.0)   # rad
        self.declare_parameter("tf_lookup_timeout", 0.3)            # s

        self.timeout  = float(self.get_parameter("timeout_at_checkpoint").value)
        self.home_id  = int(self.get_parameter("home_checkpoint_id").value)

        self.enable_pre_rotate = bool(self.get_parameter("enable_pre_rotate").value)
        self.pre_rotate_angle_threshold = float(
            self.get_parameter("pre_rotate_angle_threshold").value
        )
        self.tf_lookup_timeout = float(self.get_parameter("tf_lookup_timeout").value)

        self.checkpoints = self._load_checkpoints()
        if not self.checkpoints:
            self.get_logger().error("No checkpoints loaded.")
            return

        # Main state
        self.state        = State.IDLE
        self.current_cp   = -1
        self.target_cp    = None
        self.goal_handle  = None
        self.arrival_time = None

        # Internal execution flow
        self._goal_phase = "idle"        # idle | pre_rotate | navigate
        self._mission_type = "normal"    # normal | return_home
        self._ignore_nav_results = False

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Nav2 action client
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 action server...")
        self._nav.wait_for_server()
        self.get_logger().info("Nav2 ready.")

        # Publishers
        self._state_pub  = self.create_publisher(String, "/robot/state", 10)
        self._cp_pub     = self.create_publisher(Int32,  "/robot/current_checkpoint", 10)
        self._status_pub = self.create_publisher(String, "/robot/status_message", 10)

        # Subscribers
        self.create_subscription(
            Int32, "/robot/navigate_to_checkpoint", self._on_nav_command, 10
        )
        self.create_subscription(
            Bool, "/robot/emergency_stop", self._on_estop, 10
        )

        # Timers
        self.create_timer(0.1, self._state_machine)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"Navigator ready. Loaded {len(self.checkpoints)} checkpoints."
        )
        self.get_logger().info(
            f"Pre-rotate: {'enabled' if self.enable_pre_rotate else 'disabled'} | "
            f"threshold={self.pre_rotate_angle_threshold:.2f} rad"
        )

    # ================================================================
    #  LOAD CHECKPOINTS
    # ================================================================
    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg = get_package_share_directory("mobile_robot")
                path = os.path.join(pkg, "config", "checkpoints.yaml")
            except Exception:
                self.get_logger().error("Package mobile_robot not found.")
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
    #  ANGLE / TF HELPERS
    # ================================================================
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _yaw_from_quat(self, q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _quat_from_yaw(self, yaw: float):
        z = math.sin(yaw / 2.0)
        w = math.cos(yaw / 2.0)
        return 0.0, 0.0, z, w

    def _get_robot_pose_in_map(self):
        tf = self._tf_buffer.lookup_transform(
            "map",
            "base_footprint",
            Time(),
            timeout=Duration(seconds=self.tf_lookup_timeout)
        )
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = self._yaw_from_quat(tf.transform.rotation)
        return x, y, yaw

    # ================================================================
    #  TOPIC CALLBACKS
    # ================================================================
    def _on_nav_command(self, msg: Int32):
        cp_id = msg.data

        # 1. Check if the checkpoint ID exists
        if cp_id not in self.checkpoints:
            self.get_logger().error(
                f"Checkpoint {cp_id} not found. "
                f"Valid IDs: {list(self.checkpoints.keys())}"
            )
            self._pub_status(f"Checkpoint {cp_id} not found.")
            return

        # 2. Reject new command while busy
        if self.state in [State.PRE_ROTATING, State.NAVIGATING, State.RETURNING_HOME]:
            self.get_logger().warn(
                f"Command rejected: Robot is busy navigating to checkpoint {self.target_cp}. "
                f"Current state = {self.state.value}"
            )
            self._pub_status(
                f"Busy. Ignored goal {cp_id}. Current state: {self.state.value}."
            )
            return

        # 3. Check emergency stop state
        if self.state == State.EMERGENCY_STOP:
            self.get_logger().warn(
                "Emergency stop active. Send False to /robot/emergency_stop first."
            )
            self._pub_status("Emergency stop active. Reset first.")
            return

        # 4. Already at requested checkpoint
        if self.state in (State.IDLE, State.AT_CHECKPOINT) and self.current_cp == cp_id:
            self.get_logger().info(f"Already at checkpoint [{cp_id}].")
            self._pub_status(f"Already at checkpoint [{cp_id}].")
            return

        self._start_goal(cp_id, mission_type="normal")

    def _on_estop(self, msg: Bool):
        if msg.data:
            if self.state != State.EMERGENCY_STOP:
                self.get_logger().warn("EMERGENCY STOP activated.")
                self._ignore_nav_results = True
                self.state = State.EMERGENCY_STOP
                self.current_cp = -1
                self.arrival_time = None
                self._goal_phase = "idle"
                self._mission_type = "normal"

                if self.goal_handle:
                    try:
                        self.goal_handle.cancel_goal_async()
                    except Exception:
                        pass

                self.goal_handle = None
                self.target_cp = None
                self._pub_status("EMERGENCY STOP")
        else:
            if self.state == State.EMERGENCY_STOP:
                self.get_logger().info("Emergency stop reset. State: IDLE.")
                self._ignore_nav_results = False
                self.state = State.IDLE
                self.goal_handle = None
                self.target_cp = None
                self.arrival_time = None
                self._goal_phase = "idle"
                self._mission_type = "normal"
                self._pub_status("Ready.")

    # ================================================================
    #  STATE MACHINE  (10 Hz)
    # ================================================================
    def _state_machine(self):
        if self.state != State.AT_CHECKPOINT or self.arrival_time is None:
            return

        if time.time() - self.arrival_time >= self.timeout:
            self.get_logger().info(
                f"Timeout at checkpoint {self.current_cp}. Returning home."
            )
            self._start_goal(self.home_id, mission_type="return_home")

    # ================================================================
    #  NAVIGATION FLOW
    # ================================================================
    def _start_goal(self, cp_id: int, mission_type: str = "normal"):
        self.target_cp = cp_id
        self.goal_handle = None
        self.arrival_time = None
        self._mission_type = mission_type
        self._ignore_nav_results = False

        if not self.enable_pre_rotate:
            self._send_goal(cp_id)
            return

        try:
            rx, ry, ryaw = self._get_robot_pose_in_map()
        except Exception as e:
            self.get_logger().warn(
                f"Cannot read robot pose from TF. Fallback direct navigation: {e}"
            )
            self._send_goal(cp_id)
            return

        goal_pose = self.checkpoints[cp_id]["pose"]
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        # If already extremely close to goal position, skip pre-rotate.
        if dist < 0.05:
            self._send_goal(cp_id)
            return

        heading_to_goal = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(heading_to_goal - ryaw)

        # Only pre-rotate if heading error is large enough
        if abs(yaw_error) < self.pre_rotate_angle_threshold:
            self._send_goal(cp_id)
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = rx
        pose.pose.position.y = ry
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = self._quat_from_yaw(heading_to_goal)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._goal_phase = "pre_rotate"
        self.state = State.PRE_ROTATING

        if mission_type == "return_home":
            self.get_logger().info(
                f"Pre-rotating before returning home [{cp_id}] "
                f"(yaw error = {yaw_error:.2f} rad)"
            )
            self._pub_status(f"Pre-rotating before returning home [{cp_id}]")
        else:
            self.get_logger().info(
                f"Pre-rotating toward checkpoint [{cp_id}] "
                f"(yaw error = {yaw_error:.2f} rad)"
            )
            self._pub_status(f"Pre-rotating toward checkpoint [{cp_id}]")

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _send_goal(self, cp_id: int):
        self.target_cp = cp_id
        self.goal_handle = None
        self.arrival_time = None
        self._goal_phase = "navigate"

        pose = self.checkpoints[cp_id]["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if self._mission_type == "return_home":
            self.state = State.RETURNING_HOME
            self.get_logger().info(
                f"Returning home to [{cp_id}] {self.checkpoints[cp_id]['name']} "
                f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}"
            )
            self._pub_status(
                f"Returning home to [{cp_id}] {self.checkpoints[cp_id]['name']}"
            )
        else:
            self.state = State.NAVIGATING
            self.get_logger().info(
                f"Navigating to [{cp_id}] {self.checkpoints[cp_id]['name']} "
                f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}"
            )
            self._pub_status(
                f"Navigating to [{cp_id}] {self.checkpoints[cp_id]['name']}"
            )

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        if self._ignore_nav_results or self.state == State.EMERGENCY_STOP:
            return

        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            self.state = State.IDLE
            self._goal_phase = "idle"
            self._mission_type = "normal"
            self._pub_status("Failed to send goal.")
            return

        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self.state = State.IDLE
            self._goal_phase = "idle"
            self._mission_type = "normal"
            self._pub_status("Goal rejected by Nav2.")
            return

        # If estop happened after send_goal_async but before acceptance
        if self._ignore_nav_results or self.state == State.EMERGENCY_STOP:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            return

        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        # Ignore late callbacks after estop/reset path
        if self._ignore_nav_results or self.state == State.EMERGENCY_STOP:
            return

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"Failed to get navigation result: {e}")
            self.state = State.IDLE
            self._goal_phase = "idle"
            self._mission_type = "normal"
            self.goal_handle = None
            self.target_cp = None
            self._pub_status("Navigation result error.")
            return

        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            # Phase 1 done: pre-rotation succeeded -> send final checkpoint goal
            if self._goal_phase == "pre_rotate":
                cp_id = self.target_cp
                self.get_logger().info(
                    f"Pre-rotation complete. Sending final goal to checkpoint {cp_id}."
                )
                self._send_goal(cp_id)
                return

            # Final goal succeeded
            self.current_cp = self.target_cp
            self.arrival_time = time.time()
            name = self.checkpoints[self.current_cp]["name"]

            if self._mission_type == "return_home" or self.current_cp == self.home_id:
                self.state = State.IDLE
                self._goal_phase = "idle"
                self._mission_type = "normal"
                self.get_logger().info(
                    f"Arrived at Home [{self.current_cp}] {name}. State: IDLE."
                )
                self._pub_status(f"At Home [{self.current_cp}] {name}. IDLE.")
            else:
                self.state = State.AT_CHECKPOINT
                self._goal_phase = "idle"
                self.get_logger().info(
                    f"Arrived at [{self.current_cp}] {name}. "
                    f"Timeout in {self.timeout:.0f}s."
                )
                self._pub_status(
                    f"At [{self.current_cp}] {name}. "
                    f"Returning home in {self.timeout:.0f}s."
                )

        elif status == GoalStatus.STATUS_CANCELED:
            self.current_cp = -1
            self.arrival_time = None
            self.target_cp = None
            self._goal_phase = "idle"
            self._mission_type = "normal"

            if self.state != State.EMERGENCY_STOP:
                self.state = State.IDLE
                self.get_logger().info("Goal canceled. State: IDLE.")
                self._pub_status("Goal canceled. IDLE.")

        elif status == GoalStatus.STATUS_ABORTED:
            self.current_cp = -1
            failed_cp = self.target_cp
            self.arrival_time = None
            self.target_cp = None
            self.state = State.IDLE
            self._goal_phase = "idle"
            self._mission_type = "normal"

            self.get_logger().error(
                f"Navigation aborted to checkpoint {failed_cp}."
            )
            self._pub_status(
                f"Aborted. Could not reach checkpoint {failed_cp}."
            )

        else:
            self.current_cp = -1
            failed_cp = self.target_cp
            self.arrival_time = None
            self.target_cp = None
            self.state = State.IDLE
            self._goal_phase = "idle"
            self._mission_type = "normal"

            self.get_logger().warn(
                f"Navigation finished with unexpected status {status} for checkpoint {failed_cp}."
            )
            self._pub_status(
                f"Navigation ended with status {status} for checkpoint {failed_cp}."
            )

    # ================================================================
    #  PUBLISHERS
    # ================================================================
    def _publish_state(self):
        m = String()
        m.data = self.state.value
        self._state_pub.publish(m)

        m = Int32()
        m.data = self.current_cp
        self._cp_pub.publish(m)

    def _pub_status(self, message: str):
        m = String()
        m.data = message
        self._status_pub.publish(m)


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