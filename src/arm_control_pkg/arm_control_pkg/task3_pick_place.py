"""
Task 3 — Pick and Place State Machine
=======================================
TEKS4430 Assignment 4

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
       Run 1: DWELL_TIME = 0.0  → the arm advances the moment it is
              within POSITION_THRESHOLD, before it has stopped moving.
              GRASP and CLOSE_GRIP have the same angles, so CLOSE_GRIP
              passes instantly and the arm lifts before the gripper
              has closed — the box is left behind.
       Run 2: DWELL_TIME = 1.5  → each state settles, the gripper closes,
              and the box is lifted.
       Explain the difference using the lecture's
       "discrete decisions vs continuous dynamics" concept.
----------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time  # noqa: F401 — you need it in _check_transition()


# ── Configuration ────────────────────────────────────────────────────
POSITION_THRESHOLD = 0.05  # rad — "close enough" to advance state

# Task 3c: change to 1.5 for Run 2
DWELL_TIME = 0.0

# Joint angles [arm1, arm2, arm3, arm4, arm5] for each state.
# Remember: 0 rad = arm straight up; NEGATIVE arm2/arm3/arm4 lean the arm
# FORWARD (towards the table), positive leans it backward.
#
# Box (4 cm cube) is at world (0.451, 0, 0.41); table top is at z = 0.39.
# Arm base frame is at world (0.156, 0, 0.099). The box is too high and
# too far for a straight-down grasp, so the gripper approaches at an
# angle (~30° below horizontal).
# Finger-frame positions from kinematics.py (world x, z):
#   HOME     → (0.189, 0.707)  arm straight up
#   APPROACH → (0.401, 0.452)  ~6 cm back from the box along the gripper
#   GRASP    → (0.451, 0.420)  at the box
#   LIFT     → (0.450, 0.530)  ~11 cm above the grasp point
# These are starting values — fine-tune them with Task 1 if the gripper
# misses the box in Webots.
STATE_ANGLES = {
    "HOME": [0.00, 0.00, 0.00, 0.00, 0.00],
    "APPROACH": [0.00, -0.15, -0.12, -1.77, 0.00],
    "GRASP": [0.00, -0.35, -0.14, -1.64, 0.00],
    "CLOSE_GRIP": [0.00, -0.35, -0.14, -1.64, 0.00],
    "LIFT": [0.00, -0.28, -0.08, -1.16, 0.00],
}
STATE_SEQUENCE = ["HOME", "APPROACH", "GRASP", "CLOSE_GRIP", "LIFT"]

GRIPPER_OPEN = 0.0245
GRIPPER_CLOSE = 0.002
# ────────────────────────────────────────────────────────────────────


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        self.state_index = 0
        self.dwell_start = None
        self.current_angles = [0.0] * 5

        self.joint_pubs = [
            self.create_publisher(Float64, f'/youBot/arm_joint_{i}/position', 10)
            for i in range(1, 6)
        ]

        self.grip_left_pub = self.create_publisher(
            Float64, '/youBot/gripper_left/position', 10)
        self.grip_right_pub = self.create_publisher(
            Float64, '/youBot/gripper_right/position', 10)

        for i in range(1, 6):
            self.create_subscription(
                Float64,
                f'/youBot/arm_joint_{i}/sensor',
                lambda msg, idx=i-1: self._joint_callback(msg, idx),
                10,
            )

        self.create_timer(0.05, self._state_machine_loop)
        self.get_logger().info('Pick and place node started.')

    def _joint_callback(self, msg, index):
        self.current_angles[index] = msg.data

    def _state_machine_loop(self):
        if self.state_index >= len(STATE_SEQUENCE):
            self.get_logger().info('Sequence complete!', once=True)
            return

        state = STATE_SEQUENCE[self.state_index]
        self._send_joint_commands(state)
        self._handle_gripper(state)

        if self._check_transition(state):
            next_s = (STATE_SEQUENCE[self.state_index + 1]
                      if self.state_index + 1 < len(STATE_SEQUENCE) else 'DONE')
            self.get_logger().info(f'{state} → {next_s}')
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
        pass   # ← replace with your implementation

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
        return False   # ← replace with your implementation

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


if __name__ == '__main__':
    main()
