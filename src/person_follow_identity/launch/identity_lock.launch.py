from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _preferred_python() -> str:
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
            FindPackageShare("person_follow_identity"),
            "config",
            "identity_lock.yaml",
        ]),
        description="Path to identity lock YAML config file",
    )
    return LaunchDescription([
        config_arg,
        ExecuteProcess(
            cmd=[
                _preferred_python(),
                "-m",
                "person_follow_identity.node",
                "--ros-args",
                "-r",
                "__node:=identity_lock_node",
                "--params-file",
                LaunchConfiguration("config"),
            ],
            output="screen",
        ),
    ])
