from setuptools import find_packages, setup
import os
from glob import glob

package_name = "arm_control_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="TEKS4430 Assignment 4 — Arm Control",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task1_fk     = arm_control_pkg.task1_fk_explorer:main",
            "task2_pid    = arm_control_pkg.task2_pid_joint:main",
            "task3_pick   = arm_control_pkg.task3_pick_place:main",
            "data_logger  = arm_control_pkg.data_logger:main",
            "plotter      = arm_control_pkg.plotter:main",
        ],
        "webots_ros2": [
            "youbot_arm_driver = arm_control_pkg.youbot_arm_driver:YouBotArmDriver",
        ],
    },
)
