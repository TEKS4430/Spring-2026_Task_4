#!/usr/bin/env python3
"""
arm.launch.py — Assignment 4: Arm Control
==========================================
Prerequisites:
    - Webots is open with worlds/arm_assignment.wbt loaded
    - Simulation is running (press Play ▶ in Webots)

Usage:
    ros2 launch arm_control_pkg arm.launch.py
"""

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    pkg = get_package_share_directory("arm_control_pkg")

    # Pass the file path — webots_ros2_driver reads and parses it internally.
    # Do NOT read the file content here; that causes a deprecation warning
    # and prints the full URDF on process exit.
    robot_urdf = os.path.join(pkg, "resource", "youbot_arm.urdf")

    youbot_driver = WebotsController(
        robot_name="youBot",
        parameters=[
            {"robot_description": robot_urdf},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            youbot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=youbot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
