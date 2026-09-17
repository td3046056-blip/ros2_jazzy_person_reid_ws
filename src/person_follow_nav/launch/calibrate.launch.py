"""
calibrate.launch.py
===================
Chi khoi dong LIDAR + DRIVER XE. Khong co gi khac.

Dung cho ca hai cong cu hieu chinh:
  ros2 run person_follow_nav calibrate_center   (can /odom + /cmd_vel)
  ros2 run person_follow_nav calibrate_lidar    (chi can /scan)

TAI SAO KHONG DUNG test_avoid_only.launch.py:
  File do co chay follow_planner_node. Node do publish /cmd_vel lien tuc
  (Twist rong khi dang tat) o 15 Hz. calibrate_center cung publish /cmd_vel
  de xoay xe. Hai nguon ghi cung mot topic thi lenh xoay bi lenh rong chen
  vao, xe giat roi dung — khong xoay du mot vong duoc.

CACH DUNG
---------
  Terminal 1:
    ros2 launch person_follow_nav calibrate.launch.py

  Terminal 2:
    ros2 run person_follow_nav calibrate_center
    # hoac
    ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BASE_PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0"


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value=LIDAR_PORT),
        DeclareLaunchArgument("robot_port", default_value=BASE_PORT),
    ]

    lidar = Node(
        package="sc_mini", executable="sc_mini", name="sc_mini", output="screen",
        parameters=[{
            "port": LaunchConfiguration("lidar_port"),
            "baud_rate": 115200,
            "frame_id": "laser",
        }],
        condition=IfCondition(LaunchConfiguration("start_lidar")),
    )

    # TF tam thoi. Sau khi calibrate_center cho ket qua thi cap nhat so dau (x).
    laser_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_laser", output="log",
        arguments=["0.10", "0.00", "0.18", "0", "0", "0", "base_link", "laser"],
    )

    driver = Node(
        package="bw_dr03_ros2", executable="decoded_serial_node",
        name="bw_dr03_decoded_serial_node", output="screen",
        parameters=[{
            "port": LaunchConfiguration("robot_port"),
            "baudrate": 115200,
            # Toc do THAT o max_percent 60 — do bang scripts/measure_speed.py + thuoc
            # (xem CALIBRATION.md). KHONG dat lai 0.226 / 1.10: do la so calib o 30%,
            # dung voi 60% thi xe chay nhanh gap 2.16 lan lenh. Phai trung ca 3 launch.
            "max_linear": 0.49,
            "max_angular": 2.46,
            "max_percent": 60,
            "deadband": 0.02,
            "stop_brake": 30,
            "cmd_timeout": 1.0,
            "wheel_radius": 0.0632,
            "wheel_separation": 0.40,
            "encoder_ppr_default": 750,
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "publish_tf": True,
        }],
        condition=IfCondition(LaunchConfiguration("start_driver")),
    )

    info = [
        LogInfo(msg="=" * 68),
        LogInfo(msg=" CHE DO HIEU CHINH — chi lidar + driver, khong co planner"),
        LogInfo(msg=" Kiem tra o terminal khac:"),
        LogInfo(msg="   ros2 topic hz /scan      (SC-Mini chay ~10 Hz)"),
        LogInfo(msg="   ros2 topic hz /odom      (~20-50 Hz)"),
        LogInfo(msg=" Roi chay:"),
        LogInfo(msg="   ros2 run person_follow_nav calibrate_center"),
        LogInfo(msg="=" * 68),
    ]

    return LaunchDescription(args + info + [lidar, laser_tf, driver])
