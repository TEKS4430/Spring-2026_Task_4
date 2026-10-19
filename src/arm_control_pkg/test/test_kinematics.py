"""
Offline checks for the arm geometry and the Task 3 poses.

No ROS and no Webots needed. Run inside the container with:
    cd /ros2_ws/src/arm_control_pkg
    python3 -m pytest test/test_kinematics.py -v
"""

import ast
import math
import os
import re

import pytest

from arm_control_pkg import kinematics as K

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOX_CENTRE = (0.451, 0.0, 0.41)  # world, must match src/worlds/arm_assignment.wbt
TABLE_TOP_Z = 0.39
TABLE_FRONT_X = 0.37


def _state_angles():
    """Read STATE_ANGLES from task3 without importing rclpy."""
    path = os.path.join(PKG_DIR, "arm_control_pkg", "task3_pick_place.py")
    tree = ast.parse(open(path).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and node.targets[0].id == "STATE_ANGLES":
            return ast.literal_eval(node.value)
    raise AssertionError("STATE_ANGLES not found")


# ── Forward kinematics matches the Youbot.proto geometry ──────────────


def test_zero_pose_points_straight_up():
    x, y, z = K.forward_kinematics([0, 0, 0, 0, 0])
    assert x == pytest.approx(0.033, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert z == pytest.approx(0.147 + 0.155 + 0.135 + 0.081 + 0.09, abs=1e-6)


def test_negative_joint2_leans_forward_positive_leans_back():
    fwd = K.forward_kinematics([0, -0.5, 0, 0, 0])
    back = K.forward_kinematics([0, 0.5, 0, 0, 0])
    assert fwd[0] > 0.033 > back[0]


def test_joint1_rotates_without_changing_height():
    a = K.forward_kinematics([0.0, -0.5, -0.5, 0, 0])
    b = K.forward_kinematics([1.0, -0.5, -0.5, 0, 0])
    assert a[2] == pytest.approx(b[2])
    assert math.hypot(a[0], a[1]) == pytest.approx(math.hypot(b[0], b[1]))
    assert b[1] > 0  # positive joint 1 turns the arm to the left (+y)


def test_joint5_does_not_move_end_effector():
    a = K.forward_kinematics([0.3, -0.4, -0.6, -0.8, 0.0])
    b = K.forward_kinematics([0.3, -0.4, -0.6, -0.8, 2.0])
    assert a == pytest.approx(b)


def test_webots_demo_front_floor_pose_reaches_floor():
    # "FRONT_FLOOR" preset from Webots' youBot demo controller
    x, _, z = K.to_world(K.forward_kinematics([0, -0.97, -1.55, -0.61, 0]))
    assert x > 0.3  # in front of the robot
    assert z < 0.08  # near the floor


def test_webots_demo_back_plate_pose_reaches_behind():
    # "BACK_PLATE_HIGH" preset: reaches back onto the robot's plate
    x, _, _ = K.to_world(K.forward_kinematics([0, 0.678, 0.682, 1.74, 0]))
    assert x < 0.0


# ── Task 3 poses are valid for the provided world ─────────────────────


@pytest.fixture(scope="module")
def angles():
    return _state_angles()


def test_all_states_within_joint_limits(angles):
    for state, q in angles.items():
        for i, (lo, hi) in enumerate(K.JOINT_LIMITS):
            assert lo <= q[i] <= hi, f"{state} joint {i + 1} out of limits"


def test_close_grip_uses_grasp_angles(angles):
    assert angles["CLOSE_GRIP"] == angles["GRASP"]


def test_grasp_reaches_box(angles):
    x, y, z = K.to_world(K.forward_kinematics(angles["GRASP"]))
    dist = math.dist((x, y, z), BOX_CENTRE)
    assert dist < 0.02, f"GRASP finger frame is {dist * 100:.1f} cm from box"


def test_approach_is_near_but_clear_of_box(angles):
    p = K.to_world(K.forward_kinematics(angles["APPROACH"]))
    assert 0.03 < math.dist(p, BOX_CENTRE) < 0.10


def test_lift_raises_the_box(angles):
    grasp = K.to_world(K.forward_kinematics(angles["GRASP"]))
    lift = K.to_world(K.forward_kinematics(angles["LIFT"]))
    assert lift[2] - grasp[2] > 0.08


@pytest.mark.parametrize("state", ["APPROACH", "GRASP", "CLOSE_GRIP", "LIFT"])
def test_arm_links_stay_above_table(angles, state):
    # Elbow and wrist must not go into the table
    for x, _, z in [K.to_world(p) for p in K.chain_points(angles[state])][1:3]:
        if x > TABLE_FRONT_X:
            assert z > TABLE_TOP_Z, f"{state}: link inside the table"


# ── Packaging ─────────────────────────────────────────────────────────


def test_entry_points_point_to_existing_modules():
    setup = open(os.path.join(PKG_DIR, "setup.py")).read()
    for module in re.findall(r"arm_control_pkg\.(\w+):", setup):
        path = os.path.join(PKG_DIR, "arm_control_pkg", module + ".py")
        assert os.path.exists(path), f"entry point module missing: {module}"


def test_world_box_matches_test_constants():
    world = open(
        os.path.join(PKG_DIR, "..", "worlds", "arm_assignment.wbt")
    ).read()
    block = world[world.index("DEF TARGET_BOX"):]
    t = re.search(r"translation ([\d.\-e]+) ([\d.\-e]+) ([\d.\-e]+)", block)
    assert tuple(float(v) for v in t.groups()) == pytest.approx(BOX_CENTRE)
    assert "size 0.04 0.04 0.04" in block
