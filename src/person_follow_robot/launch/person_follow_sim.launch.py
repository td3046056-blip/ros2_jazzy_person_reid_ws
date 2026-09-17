from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    default_controller_config = PathJoinSubstitution([
        FindPackageShare("person_follow_robot"), "config", "follow_controller_sim.yaml"
    ])

    args = [
        DeclareLaunchArgument("start_robot_sim", default_value="true"),
        DeclareLaunchArgument("start_identity", default_value="true"),
        DeclareLaunchArgument("start_controller", default_value="true"),
        DeclareLaunchArgument("identity_config", default_value=default_identity_config),
        DeclareLaunchArgument("controller_config", default_value=default_controller_config),
        DeclareLaunchArgument(
            "camera_source",
            default_value="/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0",
        ),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/diff_drive_controller/cmd_vel_unstamped"),
        DeclareLaunchArgument("target_distance_m", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robot_simulation"), "launch", "robot_sim.launch.py"])
        ]),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        condition=IfCondition(LaunchConfiguration("start_robot_sim")),
    )

    identity = ExecuteProcess(
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
    )

    controller = Node(
        package="person_follow_robot",
        executable="follow_controller",
        name="person_follow_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("controller_config"),
            {
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "target_distance_m": ParameterValue(LaunchConfiguration("target_distance_m"), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_controller")),
    )

    return LaunchDescription(args + [robot_sim, identity, controller])
