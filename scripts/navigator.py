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
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool, Int32

import yaml
import os
import time
from enum import Enum
from ament_index_python.packages import get_package_share_directory


class State(Enum):
    IDLE           = "IDLE"
    NAVIGATING     = "NAVIGATING"
    AT_CHECKPOINT  = "AT_CHECKPOINT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RETURNING_HOME = "RETURNING_HOME"


class CheckpointNavigator(Node):

    def __init__(self):
        super().__init__("checkpoint_navigator")

        self.declare_parameter("checkpoint_file",       "")
        self.declare_parameter("timeout_at_checkpoint", 30.0)
        self.declare_parameter("home_checkpoint_id",    0)

        self.timeout  = self.get_parameter("timeout_at_checkpoint").value
        self.home_id  = self.get_parameter("home_checkpoint_id").value

        self.checkpoints = self._load_checkpoints()
        if not self.checkpoints:
            self.get_logger().error("No checkpoints loaded.")
            return

        self.state        = State.IDLE
        self.current_cp   = -1
        # self.current_cp   = self.home_id
        self.target_cp    = None
        self.goal_handle  = None
        self.arrival_time = None

        # Nav2 action client
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 action server...")
        self._nav.wait_for_server()
        self.get_logger().info("Nav2 ready.")

        # Publishers
        self._state_pub  = self.create_publisher(String, "/robot/state",              10)
        self._cp_pub     = self.create_publisher(Int32,  "/robot/current_checkpoint", 10)
        self._status_pub = self.create_publisher(String, "/robot/status_message",     10)

        # Subscribers
        self.create_subscription(
            Int32, "/robot/navigate_to_checkpoint", self._on_nav_command, 10)
        self.create_subscription(
            Bool, "/robot/emergency_stop", self._on_estop, 10)

        # Timers
        self.create_timer(0.1, self._state_machine)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"Navigator ready. Loaded {len(self.checkpoints)} checkpoints.")

    # ================================================================
    #  LOAD CHECKPOINTS
    # ================================================================
    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg  = get_package_share_directory("mobile_robot")
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
    #  TOPIC CALLBACKS
    # ================================================================
    def _on_nav_command(self, msg: Int32):
        cp_id = msg.data
        # 1. Check if the checkpoint ID exists
        if cp_id not in self.checkpoints:
            self.get_logger().error(
                f"Checkpoint {cp_id} not found. "
                f"Valid IDs: {list(self.checkpoints.keys())}")
            return

        # 2. Check if the robot is currently moving; if so, REJECT new commands
        if self.state in [State.NAVIGATING, State.RETURNING_HOME]:
            self.get_logger().warn(
                f"Command rejected: Robot is currently navigating to checkpoint {self.target_cp}. "
                f"Please wait for completion or send a Stop command."
            )
            self._pub_status(f"Busy. Ignored goal {cp_id}. Navigating to {self.target_cp}.")
            return

        # 3. Check for emergency stop state
        if self.state == State.EMERGENCY_STOP:
            self.get_logger().warn(
                "Emergency stop active. Send False to /robot/emergency_stop first.")
            return

        # 4. Check if already at the requested checkpoint
        if self.state in (State.IDLE, State.AT_CHECKPOINT) and self.current_cp == cp_id:
            self.get_logger().info(f"Already at checkpoint [{cp_id}].")
            return

        # Goal Preemption: allow interrupting the current navigation with a new goal.
        # if self.goal_handle and self.state in (State.NAVIGATING, State.RETURNING_HOME):
        #     self.goal_handle.cancel_goal_async()

        self._send_goal(cp_id)

    def _on_estop(self, msg: Bool):
        if msg.data:
            if self.state != State.EMERGENCY_STOP:
                self.get_logger().warn("EMERGENCY STOP activated.")
                self.state = State.EMERGENCY_STOP
                self.current_cp = -1
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                self._pub_status("EMERGENCY STOP")
        else:
            if self.state == State.EMERGENCY_STOP:
                self.get_logger().info("Emergency stop reset. State: IDLE.")
                self.state       = State.IDLE
                self.goal_handle = None
                self._pub_status("Ready.")

    # ================================================================
    #  STATE MACHINE  (10 Hz)
    # ================================================================
    def _state_machine(self):
        if self.state != State.AT_CHECKPOINT or self.arrival_time is None:
            return
        if time.time() - self.arrival_time >= self.timeout:
            self.get_logger().info(
                f"Timeout at checkpoint {self.current_cp}. Returning home.")
            self.state = State.RETURNING_HOME
            self._send_goal(self.home_id)

    # ================================================================
    #  NAVIGATION
    # ================================================================
    def _send_goal(self, cp_id: int):
        self.target_cp = cp_id
        self.state     = State.NAVIGATING

        pose = self.checkpoints[cp_id]["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"Navigating to [{cp_id}] {self.checkpoints[cp_id]['name']} "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}")
        self._pub_status(
            f"Navigating to [{cp_id}] {self.checkpoints[cp_id]['name']}")

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
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
                self.get_logger().info(f"Arrived at Home [{self.current_cp}] {name}. State: IDLE.")
                self._pub_status(f"At Home [{self.current_cp}]. IDLE.")
            else:
                self.state = State.AT_CHECKPOINT
                self.get_logger().info(
                    f"Arrived at [{self.current_cp}] {name}. "
                    f"Timeout in {self.timeout:.0f}s.")
                self._pub_status(
                    f"At [{self.current_cp}] {name}. "
                    f"Returning home in {self.timeout:.0f}s.")

        elif status == GoalStatus.STATUS_CANCELED:
            self.current_cp = -1
            if self.state != State.EMERGENCY_STOP:
                self.state = State.IDLE
                self.get_logger().info("Goal canceled. State: IDLE.")

        elif status == GoalStatus.STATUS_ABORTED:
            self.current_cp = -1
            self.get_logger().error(
                f"Navigation aborted to checkpoint {self.target_cp}.")
            self.state = State.IDLE
            self._pub_status(f"Aborted. Could not reach checkpoint {self.target_cp}.")

    # ================================================================
    #  PUBLISHERS
    # ================================================================
    def _publish_state(self):
        m = String(); m.data = self.state.value;  self._state_pub.publish(m)
        m = Int32();  m.data = self.current_cp;   self._cp_pub.publish(m)

    def _pub_status(self, message: str):
        m = String(); m.data = message;           self._status_pub.publish(m)


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