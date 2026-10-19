"""
kinematics.py — youBot arm forward kinematics (pure Python, no ROS).

Geometry is taken from Webots' Youbot.proto (R2025a):
  * All joints at 0 rad  -> the arm points STRAIGHT UP.
  * Joints 2, 3, 4 rotate about the -y axis, so a POSITIVE angle tilts the
    arm BACKWARD (over the robot body) and a NEGATIVE angle tilts it
    FORWARD (towards the table).
  * Joint 1 rotates the whole arm about the vertical axis.

Frame of the returned point: the arm base ("arm0") frame, which sits at
x = 0.156 m in front of the robot centre, at the top of the chassis.
  x = forward, y = left, z = up.

The returned point is the gripper "finger frame", 0.09 m beyond the
joint-5 axis (roughly where the fingers attach).
"""

import math

# Link offsets from Youbot.proto (metres)
BASE_TO_J2_Z = 0.077 + 0.07  # arm1 anchor z + arm2 anchor z
SHOULDER_X = 0.033  # arm2 anchor is 3.3 cm forward of the base axis
L2 = 0.155  # joint 2 -> joint 3
L3 = 0.135  # joint 3 -> joint 4
L4 = 0.081  # joint 4 -> joint 5
L5 = 0.09  # joint 5 -> finger frame

# Joint limits from Youbot.proto (rad)
JOINT_LIMITS = [
    (-2.9496, 2.9496),  # arm1 — base rotation
    (-1.13446, 1.5708),  # arm2 — shoulder
    (-2.63545, 2.54818),  # arm3 — elbow
    (-1.78024, 1.78024),  # arm4 — wrist pitch
    (-2.92343, 2.92343),  # arm5 — wrist roll
]

# Where the arm base frame is in the provided world (robot at the origin,
# chassis origin at z = 0.099 m). Used only for checks and comments.
ARM_BASE_IN_WORLD = (0.156, 0.0, 0.099)


def chain_points(angles):
    """Return the (x, y, z) of shoulder, elbow, wrist and finger frame."""
    q1, q2, q3, q4 = angles[0], angles[1], angles[2], angles[3]
    a2 = q2
    a3 = q2 + q3
    a4 = q2 + q3 + q4

    # Planar (r, z) positions in the vertical plane of the arm
    r, z = SHOULDER_X, BASE_TO_J2_Z
    planar = [(r, z)]
    for length, a in ((L2, a2), (L3, a3), (L4 + L5, a4)):
        r -= length * math.sin(a)
        z += length * math.cos(a)
        planar.append((r, z))

    c, s = math.cos(q1), math.sin(q1)
    return [(pr * c, pr * s, pz) for pr, pz in planar]


def forward_kinematics(angles):
    """Return (x, y, z) of the finger frame in the arm base frame."""
    return chain_points(angles)[-1]


def gripper_pitch(angles):
    """Gripper direction in rad: 0 = up, pi/2 = forward, pi = straight down."""
    return -(angles[1] + angles[2] + angles[3])


def to_world(point):
    """Convert an arm-base-frame point to world coordinates (robot at origin)."""
    return tuple(p + b for p, b in zip(point, ARM_BASE_IN_WORLD))
