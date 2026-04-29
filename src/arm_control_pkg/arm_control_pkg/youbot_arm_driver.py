"""
youbot_arm_driver.py — youBot arm driver for webots_ros2_driver
Students do NOT modify this file.

Device names come from the Youbot.proto file:
  Motors:  arm1, arm2, arm3, arm4, arm5
  Sensors: arm1sensor, arm2sensor, arm3sensor, arm4sensor, arm5sensor
  Gripper: finger::left, finger::right
"""

import sys
import math
import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


def _log(msg):
    sys.stderr.write(f"[YouBotArmDriver] {msg}\n")
    sys.stderr.flush()


# Exact limits from Youbot.proto
JOINT_LIMITS = [
    (-2.9496, 2.9496),  # arm1 — base rotation
    (-1.13446, 1.5708),  # arm2 — shoulder
    (-2.63545, 2.54818),  # arm3 — elbow
    (-1.78024, 1.78024),  # arm4 — wrist pitch
    (-2.92343, 2.92343),  # arm5 — wrist roll
]
GRIPPER_LIMIT = (0.0, 0.025)

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

# Link lengths for approximate FK (metres, from PROTO geometry)
L1 = 0.077 + 0.07  # arm1 anchor z + arm2 anchor z
L2 = 0.155  # arm2→arm3
L3 = 0.135  # arm3→arm4
L4 = 0.081  # arm4→arm5
L5 = 0.09  # arm5→finger tip


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

    def _grip_cb(self, msg):
        lo, hi = GRIPPER_LIMIT
        self._grip_target = max(lo, min(hi, msg.data))

    # ── Step ──────────────────────────────────────────────────────────

    def step(self):
        if not self._ready:
            return

        rclpy.spin_once(self._node, timeout_sec=0)

        for i, motor in enumerate(self._arm_motors):
            motor.setPosition(self._arm_targets[i])

        for motor in self._gripper_motors:
            motor.setPosition(self._grip_target)

        angles = []
        for i, sensor in enumerate(self._arm_sensors):
            val = sensor.getValue() if sensor else 0.0
            angles.append(val)
            msg = Float64()
            msg.data = val
            self._arm_sensor_pubs[i].publish(msg)

        self._ee_pub.publish(self._fk(angles))

    # ── Approximate FK ────────────────────────────────────────────────

    def _fk(self, angles):
        q1, q2, q3, q4 = angles[0], angles[1], angles[2], angles[3]
        a2 = q2
        a3 = q2 + q3
        a4 = q2 + q3 + q4
        r = L2 * math.cos(a2) + L3 * math.cos(a3) + (L4 + L5) * math.cos(a4)
        z = L1 + L2 * math.sin(a2) + L3 * math.sin(a3) + (L4 + L5) * math.sin(a4)
        pt = Point()
        pt.x = round(r * math.cos(q1), 4)
        pt.y = round(r * math.sin(q1), 4)
        pt.z = round(z, 4)
        return pt
