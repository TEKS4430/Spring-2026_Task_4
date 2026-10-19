"""
Task 2 — PID Joint Controller (Sine Wave Tracking)
====================================================
TEKS4430 Assignment 4

The target angle for joint 2 (shoulder) moves as a sine wave.
Your controller sends a VELOCITY command (rad/s) to joint 2 on
/youBot/arm_joint_2/velocity. The joint only moves when your controller
tells it to, so feedback is the only thing that makes it follow:

  correction = 0  ->  the arm sits still while the target moves away.

The node first moves joint 2 to the centre of the wave (OFFSET) so every
run starts from the same place, then starts tracking. Errors are only
published (and logged) during tracking.

⚠️  Requires the driver running in Terminal 1:
    ros2 launch arm_control_pkg arm.launch.py

Run workflow:
  T1: ros2 launch arm_control_pkg arm.launch.py
  T2: ros2 run arm_control_pkg task2_pid
  T3: ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only

The logger saves the comparison plot automatically when you press Ctrl+C.
(You can regenerate it at any time with: ros2 run arm_control_pkg plotter)

----------------------------------------------------------------------
YOUR TASKS:
----------------------------------------------------------------------
Task 2a — Implement P only (Kd = Ki = 0).
  The arm follows the wave but lags behind it.
  Try several Kp values (e.g. 1, 2, 5, 10). Higher Kp -> less lag, but
  past some value the arm starts to oscillate. Keep a value below that.
  Save log with label:=P_only.
  In your report: describe the lag and how it changed with Kp.

Task 2b — Add the D term (keep Kp).
  Try Kd in 0.05–0.3. On this joint D changes little at moderate Kp;
  its effect is visible when Kp is near the oscillation limit, where it
  damps the oscillation. Save log with label:=PD.
  In your report: explain what D does and why its effect here is small.

Task 2c — Add the I term (keep Kp, Kd).
  Try Ki in 0.5–2.0. The I term removes most of the remaining lag.
  Save log with label:=PID.
  Report your final Kp, Ki, Kd values.

For each task:
  1. Edit the gains below and rebuild (cb && src)
  2. Run task2_pid + data_logger simultaneously
  3. Let it run for ~60 seconds (3 full sine cycles of 20 s)
  4. Stop both with Ctrl+C
  5. Open src/arm_control_pkg/logs/task2_plot.png
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
Kp = 2.0  # Task 2a: start here, tune until arm follows the wave
Kd = 0.0  # Task 2b: add after Kp is working
Ki = 0.0  # Task 2c: add last
# ─────────────────────────────────────────────────────────────────────

CONTROL_RATE = 10  # Hz — do not change
PREPOSITION_TIME = 3.0  # s — move to OFFSET before tracking starts
MAX_VELOCITY = 1.5  # rad/s — command is clipped to this


class PIDJointController(Node):

    def __init__(self):
        super().__init__("pid_joint_controller")

        self.current_angle = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 1.0 / CONTROL_RATE
        self._log_counter = 0
        self.start_time = None  # set when tracking starts

        # Position publisher — only used to pre-position the joint
        self.pos_pub = self.create_publisher(
            Float64, "/youBot/arm_joint_2/position", 10
        )
        # Velocity publisher — your controller output goes here
        self.cmd_pub = self.create_publisher(
            Float64, "/youBot/arm_joint_2/velocity", 10
        )
        self.node_start = self.get_clock().now().nanoseconds / 1e9

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
        now = self.get_clock().now().nanoseconds / 1e9

        # ── Pre-position to the centre of the wave — provided ─────────
        if now - self.node_start < PREPOSITION_TIME:
            msg = Float64()
            msg.data = OFFSET
            self.pos_pub.publish(msg)
            return
        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("At start position — tracking begins.")

        # ── Moving target — provided, do not change ───────────────────
        t = now - self.start_time
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

        # Send command — joint velocity in rad/s (clipped for safety)
        cmd = Float64()
        cmd.data = max(-MAX_VELOCITY, min(MAX_VELOCITY, correction))
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
