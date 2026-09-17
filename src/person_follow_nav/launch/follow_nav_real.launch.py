"""
follow_nav_real.launch.py
=========================
Khoi dong TOAN BO he thong tren robot that.

  camera USB -> person_follow_identity -> /person_reid/target ─┐
  SC-Mini    -> sc_mini                -> /scan ──────────────┤
  NodeMCU    -> rssi_serial_node       -> /rssi/* ────────────┼-> target_tracker
  BW-DR03    -> decoded_serial_node    -> /odom ──────────────┘        │
                                                                       v
                                                              /follow/target
                                                                       │
                              /scan, /odom ──────────────────> follow_planner
                                                                       │
                                                                  /cmd_vel
                                                                       v
                                                              decoded_serial_node

CHAY:
  ros2 launch person_follow_nav follow_nav_real.launch.py

THU TU BAT (quan trong):
  1. ros2 service call /person_reid/start_enroll  std_srvs/srv/Trigger {}
  2. (di quanh camera ~20-30s cho no hoc) 
  3. ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}
  4. ros2 service call /follow/enable             std_srvs/srv/Trigger {}

DUNG KHAN:
  ros2 service call /follow/stop std_srvs/srv/Trigger {}
"""

from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _preferred_python() -> str:
    venv = os.environ.get("VIRTUAL_ENV", "")
    if venv:
        cand = os.path.join(venv, "bin", "python")
        if os.path.exists(cand):
            return cand
    return sys.executable


def generate_launch_description() -> LaunchDescription:
    cfg = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "follow_nav.yaml"])
    identity_cfg = PathJoinSubstitution(
        [FindPackageShare("person_follow_robot"), "config", "identity_lock_kingsen.yaml"]
    )

    args = [
        DeclareLaunchArgument("config", default_value=cfg),
        DeclareLaunchArgument("identity_config", default_value=identity_cfg),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_rssi", default_value="false"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
        DeclareLaunchArgument("robot_port", default_value="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0"),
        DeclareLaunchArgument("rssi_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if01-port0"),
        DeclareLaunchArgument(
            "camera_source",
            default_value="/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0",
        ),
        DeclareLaunchArgument("follow_distance_m", default_value="1.0"),
    ]

    # ── LiDAR ────────────────────────────────────────────────────────────
    lidar = Node(
        package="sc_mini",
        executable="sc_mini",
        name="sc_mini",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("lidar_port"),
            "baud_rate": 115200,
            "frame_id": "laser",
        }],
        condition=IfCondition(LaunchConfiguration("start_lidar")),
    )

    # TF tinh base_link -> laser.  x=0.10  y=0.00  z=0.18  (do tren xe that)
    # Goc yaw de 0 vi ta da xu ly bang lidar_yaw_offset_deg trong config.
    # LUU Y: z=0.18 la THAP. Lidar quet o 18cm nen thay chan ban, chan ghe,
    # ong quyen nguoi — nhung KHONG thay mat ban nho ra, bac them, day dien.
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        output="log",
        arguments=["0.10", "0.00", "0.18", "0", "0", "0", "base_link", "laser"],
    )

    # ── Driver xe (publish /odom + TF odom->base_link) ───────────────────
    driver = Node(
        package="bw_dr03_ros2",
        executable="decoded_serial_node",
        name="bw_dr03_decoded_serial_node",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("robot_port"),
            "baudrate": 115200,
            # Toc do THAT o max_percent 60 — do bang scripts/measure_speed.py + thuoc
            # (xem CALIBRATION.md). KHONG dat lai 0.226 / 1.10: do la so calib o 30%,
            # dung voi 60% thi xe chay nhanh gap 2.16 lan lenh. Phai trung ca 3 launch.
            "max_linear": 0.49,
            "max_angular": 2.46,
            # 30% khong du de banh quay o lenh cham — ghi chu cua ban ghi phai 60%
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

    # ── Camera ReID ──────────────────────────────────────────────────────
    identity = ExecuteProcess(
        cmd=[
            _preferred_python(), "-m", "person_follow_identity.node",
            "--ros-args", "-r", "__node:=identity_lock_node",
            "--params-file", LaunchConfiguration("identity_config"),
            "-p", ["camera_source:=", LaunchConfiguration("camera_source")],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # ── RSSI ─────────────────────────────────────────────────────────────
    rssi = Node(
        package="robot_rssi_ros2",
        executable="rssi_serial_node",
        name="rssi_serial_node",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("rssi_port"),
            "baudrate": 115200,
            "signal_timeout_sec": 3.0,
        }],
        condition=IfCondition(LaunchConfiguration("start_rssi")),
    )

    # ── Hai node moi ─────────────────────────────────────────────────────
    tracker = Node(
        package="person_follow_nav",
        executable="target_tracker",
        name="target_tracker_node",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    planner = Node(
        package="person_follow_nav",
        executable="follow_planner",
        name="follow_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"follow_distance_m": ParameterValue(
                LaunchConfiguration("follow_distance_m"), value_type=float)},
        ],
    )

    info = [
        LogInfo(msg="=" * 66),
        LogInfo(msg=" BAM NGUOI + NE VAT CAN — BW-DR03"),
        LogInfo(msg="   1) ros2 service call /person_reid/start_enroll  std_srvs/srv/Trigger {}"),
        LogInfo(msg="   2) ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}"),
        LogInfo(msg="   3) ros2 service call /follow/enable             std_srvs/srv/Trigger {}"),
        LogInfo(msg="   DUNG: ros2 service call /follow/stop            std_srvs/srv/Trigger {}"),
        LogInfo(msg=" Theo doi: ros2 topic echo /follow/planner_status"),
        LogInfo(msg="=" * 66),
    ]

    return LaunchDescription(
        args + info + [lidar, laser_tf, driver, identity, rssi, tracker, planner]
    )
