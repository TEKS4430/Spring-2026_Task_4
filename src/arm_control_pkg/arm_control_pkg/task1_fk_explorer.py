"""
Task 1 — Forward Kinematics Explorer
=====================================
TEKS4430 Assignment 2

HOW TO USE:
  Edit joint_index and target_angle at the bottom of this file,
  rebuild (cb && src), then run:
    ros2 run arm_control_pkg task1_fk

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


class FKExplorer(Node):

    def __init__(self, joint_index: int, target_angle: float):
        super().__init__("fk_explorer")

        self.joint_index = joint_index
        self.target_angle = target_angle
        self.start_ee = None
        self.moved = False

        # Use a name that does NOT collide with rclpy's internal
        # self._publishers list — rclpy uses that name internally.
        self._joint_cmd_pubs = {}
        for i in range(1, 6):
            topic = f"/youBot/arm_joint_{i}/position"
            self._joint_cmd_pubs[i] = self.create_publisher(Float64, topic, 10)

        self.create_subscription(
            Point, "/youBot/end_effector_position", self._ee_callback, 10
        )

        # Send command after 1 s so subscribers are connected
        self.create_timer(1.0, self._send_command)

        self.get_logger().info(
            f"FK Explorer — joint {joint_index} → "
            f"{target_angle:.3f} rad ({target_angle * 57.3:.1f} deg)"
        )

    def _ee_callback(self, msg: Point):
        if self.start_ee is None:
            self.start_ee = (msg.x, msg.y, msg.z)
            self.get_logger().info(
                f"Start EE:   x={msg.x:.4f}  y={msg.y:.4f}  z={msg.z:.4f}"
            )
        if self.moved:
            self._ee_counter = getattr(self, "_ee_counter", 0) + 1
            if self._ee_counter % 10 == 0:  # ~3 Hz — readable without flooding
                self.get_logger().info(
                    f"Current EE: x={msg.x:.4f}  y={msg.y:.4f}  z={msg.z:.4f}"
                )

    def _send_command(self):
        if self.moved:
            return
        msg = Float64()
        msg.data = self.target_angle
        self._joint_cmd_pubs[self.joint_index].publish(msg)
        self.moved = True
        self.get_logger().info(
            f"Command sent: joint {self.joint_index} → {self.target_angle:.3f} rad"
        )
        self.get_logger().info(
            "Arm settling — watch EE position above. Record in your table."
        )
        # ----------------------------------------------------------------
        # TODO (Task 1a): After arm settles (~2 s), record the
        # "Current EE" x/y/z values into your report table.
        # ----------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    # ── EDIT THESE for each experiment ──────────────────────────────
    joint_index = 1  # which joint to move (1–5)
    target_angle = 1.0  # target angle in radians
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
