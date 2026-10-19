"""
Task 1 — Forward Kinematics Explorer
=====================================
TEKS4430 Assignment 4

HOW TO USE:
  Edit joint_index and target_angle at the bottom of this file,
  rebuild (cb && src), then run:
    ros2 run arm_control_pkg task1_fk

  Every run first sends ALL joints to 0.0 rad (arm straight up), waits
  for the arm to settle, records "Start EE", then moves the one joint.
  So every row of the table really starts from 0.0.

YOUR TASKS:
  Task 1a — Run for each row and fill in the report table:
    | Joint | Start (rad) | End (rad) | EE start (x,y,z) | EE end (x,y,z) |
    |   1   |     0.0     |    1.0    |       ...        |      ...       |
    |   1   |     0.0     |    2.0    |       ...        |      ...       |
    |   2   |     0.0     |    0.5    |       ...        |      ...       |
    |   2   |     0.0     |    1.0    |       ...        |      ...       |
    |   3   |     0.0     |   -1.0    |       ...        |      ...       |
    |   4   |     0.0     |    1.0    |       ...        |      ...       |

  Task 1b — Answer in your report:
    "Why can't the arm reach (0.0, 0.0, 0.05) regardless of joint angles?"
    Hint: think about the arm's reachable workspace.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


RESET_SETTLE_TIME = 3.0  # s — time allowed for the arm to reach 0.0


class FKExplorer(Node):

    def __init__(self, joint_index: int, target_angle: float):
        super().__init__("fk_explorer")

        self.joint_index = joint_index
        self.target_angle = target_angle
        self.latest_ee = None
        self.start_ee = None
        self.phase = "WAIT"  # WAIT -> RESET -> MOVED
        self._ee_counter = 0

        # Use a name that does NOT collide with rclpy's internal
        # self._publishers list — rclpy uses that name internally.
        self._joint_cmd_pubs = {}
        for i in range(1, 6):
            topic = f"/youBot/arm_joint_{i}/position"
            self._joint_cmd_pubs[i] = self.create_publisher(Float64, topic, 10)

        self.create_subscription(
            Point, "/youBot/end_effector_position", self._ee_callback, 10
        )

        # 1 s: reset all joints, then move after the arm has settled
        self.create_timer(1.0, self._reset_all_joints)
        self.create_timer(1.0 + RESET_SETTLE_TIME, self._send_command)

        self.get_logger().info(
            f"FK Explorer — joint {joint_index} → "
            f"{target_angle:.3f} rad ({target_angle * 57.3:.1f} deg)"
        )

    def _publish(self, joint, angle):
        msg = Float64()
        msg.data = float(angle)
        self._joint_cmd_pubs[joint].publish(msg)

    def _ee_callback(self, msg: Point):
        self.latest_ee = (msg.x, msg.y, msg.z)
        if self.phase == "MOVED":
            self._ee_counter += 1
            if self._ee_counter % 10 == 0:  # ~3 Hz — readable without flooding
                self.get_logger().info(
                    f"Current EE: x={msg.x:.4f}  y={msg.y:.4f}  z={msg.z:.4f}"
                )

    def _reset_all_joints(self):
        if self.phase != "WAIT":
            return
        for j in range(1, 6):
            self._publish(j, 0.0)
        self.phase = "RESET"
        self.get_logger().info("Resetting all joints to 0.0 rad ...")

    def _send_command(self):
        if self.phase != "RESET":
            return
        if self.latest_ee is None:
            self.get_logger().warn(
                "No /youBot/end_effector_position yet — is the driver running?"
            )
            return  # timer fires again next period
        self.start_ee = self.latest_ee
        x, y, z = self.start_ee
        self.get_logger().info(f"Start EE:   x={x:.4f}  y={y:.4f}  z={z:.4f}")

        self._publish(self.joint_index, self.target_angle)
        self.phase = "MOVED"
        self.get_logger().info(
            f"Command sent: joint {self.joint_index} → {self.target_angle:.3f} rad"
        )
        self.get_logger().info(
            "Arm settling — when 'Current EE' stops changing, record it."
        )
        # ----------------------------------------------------------------
        # TODO (Task 1a): After arm settles (~2 s), record the
        # "Current EE" x/y/z values into your report table.
        # ----------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    # ── EDIT THESE for each experiment ──────────────────────────────
    joint_index = 1  # which joint to move (1–5)
    target_angle = 2  # target angle in radians
    # ────────────────────────────────────────────────────────────────

    node = FKExplorer(joint_index, target_angle)
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
