"""
Task 2 — PID Joint Controller
==============================
TEKS4430 Assignment 2

Joint 2 (shoulder) must reach 1.0 rad from 0.0 rad.
Implement P → PD → PID and observe how each term improves the response.

⚠️  Requires the driver to be running in Terminal 1:
    ros2 launch arm_control_pkg arm.launch.py

Run workflow:
  T1: ros2 launch arm_control_pkg arm.launch.py
  T2: ros2 run arm_control_pkg task2_pid
  T3: ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only

After all three runs:
  ros2 run arm_control_pkg plotter

----------------------------------------------------------------------
YOUR TASKS:
  2a — Set Ki=0, Kd=0. Tune Kp until joint reaches near 1.0 rad.
       Save log as label:=P_only. Describe the overshoot.
  2b — Keep Kp. Add Kd to reduce overshoot.
       Save log as label:=PD.
  2c — Keep Kp+Kd. Add Ki to eliminate steady-state error.
       Save log as label:=PID. Report final Kp, Ki, Kd values.
----------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ── Tuning parameters — EDIT THESE ──────────────────────────────────
Kp = 1.0  # Task 2a: start here
Kd = 0.0  # Task 2b: add after Kp is working
Ki = 0.0  # Task 2c: add last
# ────────────────────────────────────────────────────────────────────

TARGET_ANGLE = 1.0  # radians — do not change
CONTROL_RATE = 50  # Hz


class PIDJointController(Node):

    def __init__(self):
        super().__init__("pid_joint_controller")

        self.current_angle = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 1.0 / CONTROL_RATE
        self._log_counter = 0

        self.cmd_pub = self.create_publisher(
            Float64, "/youBot/arm_joint_2/position", 10
        )

        self.error_pub = self.create_publisher(Float64, "/arm/joint2_error", 10)

        self.create_subscription(
            Float64, "/youBot/arm_joint_2/sensor", self._angle_callback, 10
        )

        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f"PID controller started | Target={TARGET_ANGLE} rad | "
            f"Kp={Kp}  Kd={Kd}  Ki={Ki}"
        )

    def _angle_callback(self, msg):
        self.current_angle = msg.data

    def _control_loop(self):
        error = TARGET_ANGLE - self.current_angle

        # ----------------------------------------------------------------
        # TODO Task 2a: Implement P term
        #   correction = Kp * error

        # TODO Task 2b: Add D term
        #   derivative = (error - self.prev_error) / self.dt
        #   correction += Kd * derivative

        # TODO Task 2c: Add I term
        #   self.integral += error * self.dt
        #   correction += Ki * self.integral
        # ----------------------------------------------------------------

        correction = 0.0  # replace with your implementation

        cmd = Float64()
        cmd.data = TARGET_ANGLE - correction
        self.cmd_pub.publish(cmd)

        err_msg = Float64()
        err_msg.data = error
        self.error_pub.publish(err_msg)

        self.prev_error = error

        # Log every 2 seconds (counter-based — avoids flooding the terminal)
        self._log_counter += 1
        if self._log_counter % (CONTROL_RATE * 2) == 0:
            self.get_logger().info(
                f"angle={self.current_angle:.4f}  "
                f"error={error:.4f}  "
                f"correction={correction:.4f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PIDJointController()
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
