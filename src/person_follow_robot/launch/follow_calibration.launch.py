from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _preferred_python() -> str:
    venv = os.environ.get("VIRTUAL_ENV", "")
    if venv:
        candidate = os.path.join(venv, "bin", "python")
        if os.path.exists(candidate):
            return candidate
    return sys.executable


def generate_launch_description():
    default_identity_config = PathJoinSubstitution([
        FindPackageShare("person_follow_robot"), "config", "identity_lock_kingsen.yaml"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("start_identity", default_value="true"),
        DeclareLaunchArgument("identity_config", default_value=default_identity_config),
        DeclareLaunchArgument(
            "camera_source",
            default_value="/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0",
        ),
        DeclareLaunchArgument("target_distance_m", default_value="1.0"),
        DeclareLaunchArgument("sample_seconds", default_value="10.0"),
        ExecuteProcess(
            cmd=[
                _preferred_python(),
                "-m",
                "person_follow_identity.node",
                "--ros-args",
                "-r",
                "__node:=identity_lock_node",
                "--params-file",
                LaunchConfiguration("identity_config"),
                "-p",
                ["camera_source:=", LaunchConfiguration("camera_source")],
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_identity")),
        ),
        Node(
            package="person_follow_robot",
            executable="follow_calibrator",
            name="follow_distance_calibrator",
            output="screen",
            parameters=[{
                "target_distance_m": ParameterValue(LaunchConfiguration("target_distance_m"), value_type=float),
                "sample_seconds": ParameterValue(LaunchConfiguration("sample_seconds"), value_type=float),
            }],
        ),
    ])
