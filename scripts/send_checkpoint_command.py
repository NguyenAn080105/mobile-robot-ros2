#!/usr/bin/env python3
"""
send_checkpoint_command.py
===========================
Interactive CLI to control the checkpoint navigator.

Usage:
    ros2 run mobile_robot send_checkpoint_command.py

Commands:
    go <id>   Navigate to checkpoint
    stop      Emergency stop
    reset     Reset emergency stop
    status    Show current state
    list      Show all checkpoints
    help      Show this help
    quit      Exit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
import sys
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory


CHECKPOINTS = {}


def load_checkpoints():
    try:
        pkg  = get_package_share_directory("mobile_robot")
        path = os.path.join(pkg, "config", "checkpoints.yaml")
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        for cp in data.get("checkpoints", []):
            CHECKPOINTS[cp["id"]] = cp.get("display_name", cp["name"])
    except Exception:
        pass


class CommandSender(Node):

    def __init__(self):
        super().__init__("checkpoint_command_sender")

        self._nav_pub   = self.create_publisher(
            Int32, "/robot/navigate_to_checkpoint", 10)
        self._estop_pub = self.create_publisher(
            Bool, "/robot/emergency_stop", 10)

        self._state       = "unknown"
        self._current_cp  = -1
        self._status_msg  = ""

        self.create_subscription(
            String, "/robot/state",
            lambda m: setattr(self, "_state", m.data), 10)
        self.create_subscription(
            Int32, "/robot/current_checkpoint",
            lambda m: setattr(self, "_current_cp", m.data), 10)
        self.create_subscription(
            String, "/robot/status_message",
            lambda m: setattr(self, "_status_msg", m.data), 10)

    def send_go(self, cp_id: int):
        if CHECKPOINTS and cp_id not in CHECKPOINTS:
            print(f"Checkpoint {cp_id} not found. Valid: {list(CHECKPOINTS.keys())}")
            return
        m = Int32(); m.data = cp_id
        self._nav_pub.publish(m)
        name = CHECKPOINTS.get(cp_id, str(cp_id))
        print(f"Sent: go to [{cp_id}] {name}")

    def send_estop(self, activate: bool):
        m = Bool(); m.data = activate
        self._estop_pub.publish(m)
        print("Sent: EMERGENCY STOP" if activate else "Sent: reset emergency stop")

    def print_status(self):
        # Spin briefly to get latest state
        rclpy.spin_once(self, timeout_sec=0.3)
        name = CHECKPOINTS.get(self._current_cp, str(self._current_cp))
        print(f"State      : {self._state}")
        print(f"Current CP : [{self._current_cp}] {name}")
        if self._status_msg:
            print(f"Message    : {self._status_msg}")

    def print_checkpoints(self):
        if not CHECKPOINTS:
            print("No checkpoint info available.")
            return
        print("Checkpoints:")
        for cid, name in sorted(CHECKPOINTS.items()):
            print(f"  [{cid}] {name}")


def print_help():
    print("\n--- Checkpoint Command Sender ---")
    print("  go <id>   Navigate to checkpoint")
    print("  stop      Emergency stop")
    print("  reset     Reset emergency stop")
    print("  status    Show current state")
    print("  list      Show all checkpoints")
    print("  help      Show this help")
    print("  quit      Exit")
    print("---------------------------------\n")


def main(args=None):
    rclpy.init(args=args)
    load_checkpoints()
    node = CommandSender()

    # Brief spin to connect subscribers
    rclpy.spin_once(node, timeout_sec=0.5)

    print_help()
    if CHECKPOINTS:
        node.print_checkpoints()
        print()

    try:
        while rclpy.ok():
            try:
                raw = input("cmd> ").strip()
            except EOFError:
                break
            if not raw:
                continue

            parts = raw.lower().split()
            cmd   = parts[0]

            if cmd == "go":
                if len(parts) != 2:
                    print("Usage: go <id>")
                    continue
                try:
                    node.send_go(int(parts[1]))
                except ValueError:
                    print("ID must be an integer.")

            elif cmd == "stop":
                node.send_estop(True)

            elif cmd == "reset":
                node.send_estop(False)

            elif cmd == "status":
                node.print_status()

            elif cmd in ("list", "ls"):
                node.print_checkpoints()

            elif cmd == "help":
                print_help()

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print(f"Unknown command '{raw}'. Type 'help'.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()