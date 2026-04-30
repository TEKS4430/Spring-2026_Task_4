"""
Task 2 — PID Joint Controller (Sine Wave Tracking)
====================================================
TEKS4430 Assignment 2

The target angle for joint 2 (shoulder) moves as a sine wave.
Your PID controller must keep the arm following the moving target.

Without control the arm just sits still while the target moves away.
As you add each term, watch the arm in Webots start to follow better.

⚠️  Requires the driver running in Terminal 1:
    ros2 launch arm_control_pkg arm.launch.py

Run workflow:
  T1: ros2 launch arm_control_pkg arm.launch.py
  T2: ros2 run arm_control_pkg task2_pid
  T3: ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only

After all three runs generate the plot:
  ros2 run arm_control_pkg plotter

----------------------------------------------------------------------
YOUR TASKS:
----------------------------------------------------------------------
Task 2a — Implement P only.
  The arm will start chasing the sine wave but lag behind.
  Tune Kp until the arm follows reasonably. Too high = oscillation.
  Save log with label:=P_only.
  In your report: describe the lag you observe.

Task 2b — Add the D term.
  The lag should reduce noticeably.
  Save log with label:=PD.
  In your report: explain what the D term does to reduce lag.

Task 2c — Add the I term.
  Any constant offset between arm and target should disappear.
  Save log with label:=PID.
  Report your final Kp, Ki, Kd values.

For each task:
  1. Edit the gains below
  2. Run task2_pid + data_logger simultaneously
  3. Let it run for ~15 seconds (about 3 full sine cycles)
  4. Stop both with Ctrl+C
  5. Run plotter to see all three curves together
----------------------------------------------------------------------
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ── Sine wave target parameters — do not change ──────────────────────
AMPLITUDE = 0.3  # radians — how far the target swings each side
FREQUENCY = 0.05  # Hz     — one full cycle every 20 seconds
OFFSET = 0.5  # radians — centre position (arm2 mid-range)
# ─────────────────────────────────────────────────────────────────────

# ── Tuning parameters — EDIT THESE ───────────────────────────────────
Kp = 1.0  # Task 2a: start here, tune until arm follows the wave
Kd = 0.0  # Task 2b: add after Kp is working
Ki = 0.0  # Task 2c: add last
# ─────────────────────────────────────────────────────────────────────

CONTROL_RATE = 10  # Hz — do not change


class PIDJointController(Node):

    def __init__(self):
        super().__init__("pid_joint_controller")

        self.current_angle = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 1.0 / CONTROL_RATE
        self._log_counter = 0

        # Command publisher — sends position to joint 2
        self.cmd_pub = self.create_publisher(
            Float64, "/youBot/arm_joint_2/position", 10
        )

        # Error publisher — read by data_logger for plotting
        self.error_pub = self.create_publisher(Float64, "/arm/joint2_error", 10)

        # Current angle from the arm driver sensor
        self.create_subscription(
            Float64, "/youBot/arm_joint_2/sensor", self._angle_callback, 10
        )

        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f"PID tracking started | "
            f"Sine: amplitude={AMPLITUDE} rad  freq={FREQUENCY} Hz  "
            f"offset={OFFSET} rad | "
            f"Kp={Kp}  Kd={Kd}  Ki={Ki}"
        )
        self.get_logger().info(
            "Watch joint 2 in Webots — it should follow the moving target."
        )

    def _angle_callback(self, msg):
        self.current_angle = msg.data

    def _control_loop(self):
        # ── Moving target — provided, do not change ───────────────────
        t = self.get_clock().now().nanoseconds / 1e9
        target = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t) + OFFSET

        error = target - self.current_angle

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

        correction = 0.0  # ← replace with your implementation

        # Send command — target position + correction to close the gap
        cmd = Float64()
        cmd.data = target + correction
        self.cmd_pub.publish(cmd)

        # Publish error for the logger
        err_msg = Float64()
        err_msg.data = error
        self.error_pub.publish(err_msg)

        self.prev_error = error

        # Log every 2 seconds
        self._log_counter += 1
        if self._log_counter % (CONTROL_RATE * 2) == 0:
            self.get_logger().info(
                f"target={target:.3f}  "
                f"angle={self.current_angle:.3f}  "
                f"error={error:.3f}  "
                f"correction={correction:.3f}"
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
