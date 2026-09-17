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
            FindPackageShare("person_reid_tracker"),
            "config",
            "usb_camera_reid.yaml",
        ]),
        description="Path to YAML config file",
    )
    camera_arg = DeclareLaunchArgument(
        "camera_source",
        default_value="/dev/video2",
        description="USB camera source, for example /dev/video2 or 2",
    )
    show_arg = DeclareLaunchArgument(
        "show_window",
        default_value="true",
        description="Open an OpenCV preview window",
    )
    draw_all_arg = DeclareLaunchArgument(
        "draw_all_tracks",
        default_value="true",
        description="Draw all DeepSORT IDs for debugging",
    )

    python_exe = _preferred_python()

    return LaunchDescription([
        config_arg,
        camera_arg,
        show_arg,
        draw_all_arg,
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
                "-p",
                ["camera_source:=", LaunchConfiguration("camera_source")],
                "-p",
                ["show_window:=", LaunchConfiguration("show_window")],
                "-p",
                ["draw_all_tracks:=", LaunchConfiguration("draw_all_tracks")],
            ],
            output="screen",
        ),
    ])
