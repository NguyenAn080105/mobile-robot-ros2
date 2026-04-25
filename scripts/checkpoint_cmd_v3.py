#!/usr/bin/env python3
"""
checkpoint_cmd_v3.py
====================
Simple CLI for navigator_v3.py.

Accepted commands only:
    go <id>
    stop
    continue
    reset

ROS interface:
    Pub: /robot/command         std_msgs/String
         "go:<id>", "stop", "continue", "reset"
    Sub: /robot/state           std_msgs/String
    Sub: /robot/current_checkpoint std_msgs/Int32
    Sub: /robot/status_message  std_msgs/String

Exit: Ctrl+C or EOF.
"""

import os
import sys
import time
import threading
from typing import Dict

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Int32


# ============================================================
#  Terminal color helper
# ============================================================
class C:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"

    @staticmethod
    def bold(s: str) -> str:
        return f"{C.BOLD}{s}{C.RESET}"

    @staticmethod
    def dim(s: str) -> str:
        return f"{C.DIM}{s}{C.RESET}"

    @staticmethod
    def ok(s: str) -> str:
        return f"{C.GREEN}{s}{C.RESET}"

    @staticmethod
    def warn(s: str) -> str:
        return f"{C.YELLOW}{s}{C.RESET}"

    @staticmethod
    def err(s: str) -> str:
        return f"{C.RED}{s}{C.RESET}"

    @staticmethod
    def info(s: str) -> str:
        return f"{C.CYAN}{s}{C.RESET}"


STATE_HINTS = {
    "IDLE": "go <id>",
    "COMPUTING_PATH": "stop",
    "PRE_ROTATING": "stop",
    "NAVIGATING": "stop",
    "STOPPED": "continue | reset",
    "WAITING_RESET": "go <id> or wait home",
    "RETURNING_HOME": "wait",
}

STATE_COLOR = {
    "IDLE": C.ok,
    "COMPUTING_PATH": C.info,
    "PRE_ROTATING": C.info,
    "NAVIGATING": C.info,
    "STOPPED": C.warn,
    "WAITING_RESET": C.warn,
    "RETURNING_HOME": C.info,
}

CHECKPOINTS: Dict[int, str] = {}


# ============================================================
#  Checkpoint loader
# ============================================================
def load_checkpoints() -> str:
    """Load config/checkpoints_v2.yaml from installed mobile_robot package."""
    try:
        pkg = get_package_share_directory("mobile_robot")
        path = os.path.join(pkg, "config", "checkpoints_v2.yaml")

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        CHECKPOINTS.clear()
        for cp in data.get("checkpoints", []):
            cp_id = int(cp["id"])
            cp_name = cp.get("display_name", cp.get("name", str(cp_id)))
            CHECKPOINTS[cp_id] = cp_name

        return path

    except Exception as exc:
        print(C.warn(f"warning: cannot load checkpoints_v2.yaml: {exc}"))
        CHECKPOINTS.clear()
        return ""


# ============================================================
#  ROS 2 command sender
# ============================================================
class CommandSender(Node):
    def __init__(self):
        super().__init__("checkpoint_command_sender_v3")

        self._cmd_pub = self.create_publisher(String, "/robot/command", 10)

        self._state = "unknown"
        self._current_cp = -1
        self._status_msg = ""
        self._last_printed_status = ""
        self._lock = threading.Lock()

        self.create_subscription(String, "/robot/state", self._on_state, 10)
        self.create_subscription(Int32, "/robot/current_checkpoint", self._on_checkpoint, 10)
        self.create_subscription(String, "/robot/status_message", self._on_status, 10)

    # ------------------------- callbacks -------------------------
    def _on_state(self, msg: String):
        with self._lock:
            old_state = self._state
            self._state = msg.data

        if msg.data != old_state:
            color = STATE_COLOR.get(msg.data, lambda s: s)
            hint = STATE_HINTS.get(msg.data, "")
            if hint:
                print(f"\nstate: {color(msg.data)} | next: {hint}", flush=True)
            else:
                print(f"\nstate: {color(msg.data)}", flush=True)

    def _on_checkpoint(self, msg: Int32):
        with self._lock:
            self._current_cp = msg.data

    def _on_status(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        with self._lock:
            if text == self._last_printed_status:
                return
            self._status_msg = text
            self._last_printed_status = text

        print(f"msg  : {text}", flush=True)

    # ------------------------- helpers -------------------------
    def _warn_if_no_navigator(self):
        if self._cmd_pub.get_subscription_count() == 0:
            print(C.warn("warning: no subscriber on /robot/command. Is navigator_v3.py running?"))

    def wait_for_navigator(self, timeout_s: float = 2.0):
        """Wait briefly for navigator_v3 to appear on /robot/command."""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_s:
            if self._cmd_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return self._cmd_pub.get_subscription_count() > 0

    def publish_command(self, command: str):
        self._warn_if_no_navigator()
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)

    def send_go(self, cp_id: int):
        if CHECKPOINTS and cp_id not in CHECKPOINTS:
            print(C.err(f"error: checkpoint {cp_id} not found. valid: {sorted(CHECKPOINTS.keys())}"))
            return

        name = CHECKPOINTS.get(cp_id, str(cp_id))
        self.publish_command(f"go:{cp_id}")
        print(C.ok(f"sent : go {cp_id} ({name})"))

    def send_stop(self):
        self.publish_command("stop")
        print(C.warn("sent : stop"))

    def send_continue(self):
        self.publish_command("continue")
        print(C.ok("sent : continue"))

    def send_reset(self):
        self.publish_command("reset")
        print(C.warn("sent : reset"))


# ============================================================
#  Terminal UI
# ============================================================
def print_startup(checkpoint_file: str, navigator_connected: bool):
    print(C.bold("Mobile Robot Navigator v3"))
    print(C.bold("ROS topics"))
    print("  pub /robot/command")
    print("  sub /robot/state, /robot/status_message")
    print(f"  navigator: {'connected' if navigator_connected else 'not detected'}")
    if checkpoint_file:
        print(C.dim(f"  checkpoints: {checkpoint_file}"))
    print()

    print(C.bold("Valid state"))
    print("  IDLE / WAITING_RESET  -> go <id>")
    print("  MOVING                -> stop")
    print("  STOPPED               -> continue | reset")

    if CHECKPOINTS:
        print(C.bold("Checkpoints"))
        for cid, name in sorted(CHECKPOINTS.items()):
            print(f"  {cid:<2} {name}")
        print()

    print(C.bold("Commands"))
    print("  go <id>     navigate to checkpoint")
    print("  stop        stop current navigation")
    print("  continue    resume stopped navigation")
    print("  reset       cancel target; wait 30s then home")

    print(C.dim("Exit: Ctrl+C"))
    print()


def parse_and_send(raw: str, node: CommandSender):
    parts = raw.strip().lower().split()
    if not parts:
        return

    cmd = parts[0]

    if cmd == "go":
        if len(parts) != 2:
            print(C.err("usage: go <id>"))
            return
        try:
            node.send_go(int(parts[1]))
        except ValueError:
            print(C.err("error: id must be an integer"))
        return

    if cmd == "stop" and len(parts) == 1:
        node.send_stop()
        return

    if cmd == "continue" and len(parts) == 1:
        node.send_continue()
        return

    if cmd == "reset" and len(parts) == 1:
        node.send_reset()
        return

    print(C.err("invalid command. use: go <id> | stop | continue | reset"))


def spin_worker(node: Node):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception as exc:
        print(C.warn(f"spin warning: {exc}"), file=sys.stderr)


# ============================================================
#  Main
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    checkpoint_file = load_checkpoints()
    node = CommandSender()

    spin_thread = threading.Thread(target=spin_worker, args=(node,), daemon=True)
    spin_thread.start()

    navigator_connected = node.wait_for_navigator(timeout_s=2.0)
    print_startup(checkpoint_file, navigator_connected)

    try:
        while rclpy.ok():
            try:
                raw = input("cmd> ").strip()
            except EOFError:
                break

            parse_and_send(raw, node)

    except KeyboardInterrupt:
        print("\nexit")

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
