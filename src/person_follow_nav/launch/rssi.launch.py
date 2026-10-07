"""
rssi.launch.py
==============
CHI 2 node RSSI: rssi_scanner_node (doc 3 board ESP32) + rssi_bearing_node (uoc luong huong nguoi).
KHONG co node nao ghi /cmd_vel. Chay CUNG voi calibrate.launch.py (lidar + driver) o terminal khac.

  ros2 launch person_follow_nav rssi.launch.py ports:="$PA,$PB,$PC"

  # giam sat
  ros2 topic echo /rssi/status  --field data --full-length
  ros2 topic echo /rssi/bearing --field data --full-length

ports: cong by-path cua 3 board, cach nhau dau phay. Board tu xung A/B/C nen thu tu khong quan trong.
KHONG dua cong LiDAR vao day (cung chip CH340) — node se tu choi mo cong dang bi sc_mini giu.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cfg = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "rssi.yaml"])
    tmpl = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "rssi_template.json"])

    args = [
        DeclareLaunchArgument("ports", default_value=""),
        DeclareLaunchArgument("template_file", default_value=tmpl),
    ]

    scanner = Node(
        package="person_follow_nav", executable="rssi_scanner", name="rssi_scanner_node", output="screen",
        parameters=[cfg, {"ports": LaunchConfiguration("ports")}],
    )
    bearing = Node(
        package="person_follow_nav", executable="rssi_bearing", name="rssi_bearing_node", output="screen",
        parameters=[cfg, {"template_file": LaunchConfiguration("template_file")}],
    )
    return LaunchDescription(args + [scanner, bearing])
