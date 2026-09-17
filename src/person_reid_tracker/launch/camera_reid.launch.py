from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _preferred_python() -> str:
    """Use the active venv python when available.

    ROS2's generated console_scripts are often created with /usr/bin/python3,
    so packages installed in .venv such as torch are not visible. Running the
    node as `python -m ...` with VIRTUAL_ENV/bin/python fixes that.
    """
    venv = os.environ.get("VIRTUAL_ENV", "")
    if venv:
        candidate = os.path.join(venv, "bin", "python")
        if os.path.exists(candidate):
            return candidate
    return sys.executable


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([
            FindPackageShare("person_reid_tracker"),
            "config",
            "camera_reid.yaml",
        ]),
        description="Path to YAML config file",
    )

    python_exe = _preferred_python()

    return LaunchDescription([
        config_arg,
        ExecuteProcess(
            cmd=[
                python_exe,
                "-m",
                "person_reid_tracker.camera_reid_node",
                "--ros-args",
                "-r",
                "__node:=camera_reid_node",
                "--params-file",
                LaunchConfiguration("config"),
            ],
            output="screen",
        ),
    ])
