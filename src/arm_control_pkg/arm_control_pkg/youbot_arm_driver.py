"""
youbot_arm_driver.py — youBot arm driver for webots_ros2_driver
Students do NOT modify this file.

Device names come from the Youbot.proto file:
  Motors:  arm1, arm2, arm3, arm4, arm5
  Sensors: arm1sensor, arm2sensor, arm3sensor, arm4sensor, arm5sensor
  Gripper: finger::left, finger::right

Topics:
  Sub  /youBot/arm_joint_N/position   Float64  target angle (rad)
  Sub  /youBot/arm_joint_N/velocity   Float64  target speed (rad/s), Task 2
  Sub  /youBot/gripper_{left,right}/position  Float64  finger opening (m)
  Pub  /youBot/arm_joint_N/sensor     Float64  measured angle (rad)
  Pub  /youBot/end_effector_position  Point    finger frame in the arm base
                                               frame (see kinematics.py)
"""

import sys
import time
import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

from arm_control_pkg.kinematics import JOINT_LIMITS, forward_kinematics


def _log(msg):
    sys.stderr.write(f"[YouBotArmDriver] {msg}\n")
    sys.stderr.flush()


GRIPPER_LIMIT = (0.0, 0.0245)

# Device names exactly as defined in Youbot.proto
ARM_MOTOR_NAMES = ["arm1", "arm2", "arm3", "arm4", "arm5"]
ARM_SENSOR_NAMES = [
    "arm1sensor",
    "arm2sensor",
    "arm3sensor",
    "arm4sensor",
    "arm5sensor",
]
GRIP_MOTOR_NAMES = ["finger::left", "finger::right"]
GRIP_SENSOR_NAMES = ["finger::leftsensor", "finger::rightsensor"]

# Velocity mode (used by Task 2): if no velocity command arrives for this
# long, the joint is stopped so it never keeps drifting after Ctrl+C.
VELOCITY_TIMEOUT = 0.3  # s


class YouBotArmDriver:

    def init(self, webots_node, properties):
        _log("init() called")

        # Safe defaults — step() will no-op if _ready stays False
        self._ready = False
        self._robot = None
        self._node = None
        self._arm_motors = []
        self._arm_sensors = []
        self._gripper_motors = []
        self._arm_sensor_pubs = []
        self._ee_pub = None
        self._arm_targets = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Per joint: None = position mode, otherwise (velocity, stamp)
        self._arm_vel_cmds = [None] * 5
        self._max_vel = [1.5708] * 5
        self._grip_target = GRIPPER_LIMIT[1]

        # Webots robot handle
        try:
            self._robot = webots_node.robot
            self._timestep = int(self._robot.getBasicTimeStep())
            _log(f"timestep = {self._timestep} ms")
        except Exception as e:
            _log(f"ERROR getting robot: {e}")
            return

        # Arm motors
        for i, name in enumerate(ARM_MOTOR_NAMES):
            try:
                motor = self._robot.getDevice(name)
                if motor is None:
                    _log(f"ERROR: motor '{name}' not found")
                    return
                motor.setPosition(self._arm_targets[i])
                self._max_vel[i] = motor.getMaxVelocity()
                self._arm_motors.append(motor)
                _log(f"motor '{name}' OK")
            except Exception as e:
                _log(f"ERROR motor '{name}': {e}")
                return

        # Arm sensors
        for name in ARM_SENSOR_NAMES:
            try:
                sensor = self._robot.getDevice(name)
                if sensor is not None:
                    sensor.enable(self._timestep)
                    _log(f"sensor '{name}' OK")
                else:
                    _log(f"WARNING: sensor '{name}' not found — using 0.0")
                self._arm_sensors.append(sensor)
            except Exception as e:
                _log(f"WARNING sensor '{name}': {e}")
                self._arm_sensors.append(None)

        # Gripper motors
        for name in GRIP_MOTOR_NAMES:
            try:
                motor = self._robot.getDevice(name)
                if motor:
                    motor.setPosition(GRIPPER_LIMIT[1])
                    self._gripper_motors.append(motor)
                    _log(f"gripper '{name}' OK")
                else:
                    _log(f"WARNING: gripper '{name}' not found")
            except Exception as e:
                _log(f"WARNING gripper '{name}': {e}")

        # ROS2 node
        try:
            rclpy.init(args=None)
        except RuntimeError:
            _log("rclpy already initialised — skipping init()")

        try:
            self._node = rclpy.create_node("youbot_arm_driver")
        except Exception as e:
            _log(f"ERROR creating ROS2 node: {e}")
            return

        # Arm command subscribers  (topics use arm_joint_N naming for clarity)
        for i in range(5):
            self._node.create_subscription(
                Float64,
                f"/youBot/arm_joint_{i + 1}/position",
                lambda msg, idx=i: self._arm_cmd_cb(msg, idx),
                10,
            )

        # Arm velocity subscribers (Task 2). A message switches that joint
        # to velocity mode; a later /position message switches it back.
        for i in range(5):
            self._node.create_subscription(
                Float64,
                f"/youBot/arm_joint_{i + 1}/velocity",
                lambda msg, idx=i: self._arm_vel_cb(msg, idx),
                10,
            )

        # Gripper subscribers
        for side in ["left", "right"]:
            self._node.create_subscription(
                Float64,
                f"/youBot/gripper_{side}/position",
                lambda msg: self._grip_cb(msg),
                10,
            )

        # Sensor publishers
        for i in range(5):
            pub = self._node.create_publisher(
                Float64, f"/youBot/arm_joint_{i + 1}/sensor", 10
            )
            self._arm_sensor_pubs.append(pub)

        # End-effector publisher
        self._ee_pub = self._node.create_publisher(
            Point, "/youBot/end_effector_position", 10
        )

        self._ready = True
        _log("Driver ready — all topics live")

    # ── Callbacks ─────────────────────────────────────────────────────

    def _arm_cmd_cb(self, msg, idx):
        lo, hi = JOINT_LIMITS[idx]
        self._arm_targets[idx] = max(lo, min(hi, msg.data))
        self._arm_vel_cmds[idx] = None

    def _arm_vel_cb(self, msg, idx):
        vmax = self._max_vel[idx]
        self._arm_vel_cmds[idx] = (max(-vmax, min(vmax, msg.data)), time.time())

    def _grip_cb(self, msg):
        lo, hi = GRIPPER_LIMIT
        self._grip_target = max(lo, min(hi, msg.data))

    # ── Step ──────────────────────────────────────────────────────────

    def step(self):
        if not self._ready:
            return

        rclpy.spin_once(self._node, timeout_sec=0)

        angles = []
        for sensor in self._arm_sensors:
            angles.append(sensor.getValue() if sensor else 0.0)

        now = time.time()
        for i, motor in enumerate(self._arm_motors):
            cmd = self._arm_vel_cmds[i]
            if cmd is None:
                # Position mode (Tasks 1 and 3)
                motor.setVelocity(self._max_vel[i])
                motor.setPosition(self._arm_targets[i])
                continue
            # Velocity mode (Task 2)
            vel, stamp = cmd
            lo, hi = JOINT_LIMITS[i]
            if now - stamp > VELOCITY_TIMEOUT:
                vel = 0.0  # controller stopped: hold still
            if (angles[i] <= lo and vel < 0) or (angles[i] >= hi and vel > 0):
                vel = 0.0  # never drive into a joint limit
            # Webots velocity control: infinite target position, signed speed
            motor.setPosition(float("inf"))
            motor.setVelocity(vel)
            self._arm_targets[i] = angles[i]  # hold here if mode switches back

        for motor in self._gripper_motors:
            motor.setPosition(self._grip_target)

        for i, val in enumerate(angles):
            msg = Float64()
            msg.data = val
            self._arm_sensor_pubs[i].publish(msg)

        x, y, z = forward_kinematics(angles)
        pt = Point()
        pt.x, pt.y, pt.z = round(x, 4), round(y, 4), round(z, 4)
        self._ee_pub.publish(pt)
