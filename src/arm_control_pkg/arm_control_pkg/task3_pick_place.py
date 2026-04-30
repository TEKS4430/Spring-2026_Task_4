"""
Task 3 — Pick and Place State Machine
=======================================
TEKS4430 Assignment 2

The arm moves through 5 states:
  HOME → APPROACH → GRASP → CLOSE_GRIP → LIFT

⚠️  Requires the driver running in Terminal 1:
    ros2 launch arm_control_pkg arm.launch.py

----------------------------------------------------------------------
YOUR TASKS:
  3a — Implement _send_joint_commands(): publish STATE_ANGLES for the
       current state to each joint topic.

  3b — Implement _check_transition(): return True only when ALL joint
       errors < POSITION_THRESHOLD AND arm has held for DWELL_TIME s.

  3c — Run twice:
       Run 1: DWELL_TIME = 0.0  → observe state skipping
       Run 2: DWELL_TIME = 1.5  → observe smooth sequence
       Explain the difference using the lecture's
       "discrete decisions vs continuous dynamics" concept.
----------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

# ── Configuration ────────────────────────────────────────────────────
POSITION_THRESHOLD = 0.05  # rad — "close enough" to advance state

# Task 3c: change to 1.5 for Run 2
DWELL_TIME = 0.0

# Pre-calculated joint angles [arm1, arm2, arm3, arm4, arm5]
# Joint limits from Youbot.proto:
#   arm1: -2.9496  to  2.9496
#   arm2: -1.1345  to  1.5708
#   arm3: -2.6355  to  2.5482
#   arm4: -1.7802  to  1.7802
#   arm5: -2.9234  to  2.9234
STATE_ANGLES = {
    "HOME": [0.00, 0.00, 0.00, 0.00, 0.00],
    "APPROACH": [0.00, 0.80, -1.00, 0.50, 0.00],
    "GRASP": [0.00, 1.10, -1.40, 0.60, 0.00],
    "CLOSE_GRIP": [0.00, 1.10, -1.40, 0.60, 0.00],
    "LIFT": [0.00, 0.50, -0.80, 0.50, 0.00],
}
STATE_SEQUENCE = ["HOME", "APPROACH", "GRASP", "CLOSE_GRIP", "LIFT"]

GRIPPER_OPEN = 0.0245
GRIPPER_CLOSE = 0.002
# ────────────────────────────────────────────────────────────────────


class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        self.state_index = 0
        self.dwell_start = None
        self.current_angles = [0.0] * 5

        self.joint_pubs = [
            self.create_publisher(Float64, f"/youBot/arm_joint_{i}/position", 10)
            for i in range(1, 6)
        ]

        self.grip_left_pub = self.create_publisher(
            Float64, "/youBot/gripper_left/position", 10
        )
        self.grip_right_pub = self.create_publisher(
            Float64, "/youBot/gripper_right/position", 10
        )

        for i in range(1, 6):
            self.create_subscription(
                Float64,
                f"/youBot/arm_joint_{i}/sensor",
                lambda msg, idx=i - 1: self._joint_callback(msg, idx),
                10,
            )

        self.create_timer(0.05, self._state_machine_loop)
        self.get_logger().info("Pick and place node started.")

    def _joint_callback(self, msg, index):
        self.current_angles[index] = msg.data

    def _state_machine_loop(self):
        if self.state_index >= len(STATE_SEQUENCE):
            self.get_logger().info("Sequence complete!", once=True)
            return

        state = STATE_SEQUENCE[self.state_index]
        self._send_joint_commands(state)
        self._handle_gripper(state)

        if self._check_transition(state):
            next_s = (
                STATE_SEQUENCE[self.state_index + 1]
                if self.state_index + 1 < len(STATE_SEQUENCE)
                else "DONE"
            )
            self.get_logger().info(f"{state} → {next_s}")
            self.state_index += 1
            self.dwell_start = None

    # ── TODO: implement these two methods ────────────────────────────

    def _send_joint_commands(self, state):
        """
        Publish the target joint angles for the current state.

        For each joint i (0 to 4):
          msg = Float64()
          msg.data = STATE_ANGLES[state][i]
          self.joint_pubs[i].publish(msg)
        """
        pass  # ← replace with your implementation

    def _check_transition(self, state):
        """
        Return True only when safe to move to the next state.

        Condition 1 — all joints close enough:
          abs(self.current_angles[i] - STATE_ANGLES[state][i]) < POSITION_THRESHOLD
          for all i in range(5)

        Condition 2 — dwell time elapsed:
          When condition 1 first becomes True, record time.time() in
          self.dwell_start.
          Return True only when time.time() - self.dwell_start >= DWELL_TIME.
          If condition 1 becomes False, reset self.dwell_start = None.
        """
        return False  # ← replace with your implementation

    # ── Provided: gripper ────────────────────────────────────────────

    def _handle_gripper(self, state):
        pos = GRIPPER_CLOSE if state == "CLOSE_GRIP" else GRIPPER_OPEN
        msg = Float64()
        msg.data = pos
        self.grip_left_pub.publish(msg)
        self.grip_right_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
