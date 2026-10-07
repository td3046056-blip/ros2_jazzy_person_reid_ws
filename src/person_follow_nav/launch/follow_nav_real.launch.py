"""
follow_nav_real.launch.py
=========================
Khoi dong TOAN BO he thong tren robot that.

  camera USB -> person_follow_identity -> /person_reid/target ─┐
  SC-Mini    -> sc_mini                -> /scan ──────────────┼-> target_tracker
  BW-DR03    -> decoded_serial_node    -> /odom ──────────────┘        │
                                                                       v
                                                              /follow/target
                                                                       │
                              /scan, /odom ──────────────────> follow_planner
  3 board ESP32 -> rssi_scanner_node -> /rssi/raw                      │   ^
                -> rssi_bearing_node -> /rssi/bearing ─────────────────┼───┘ (chi khi TIM LAI nguoi)
                                                                  /cmd_vel
                                                                       v
                                                              decoded_serial_node

CHAY:
  ros2 launch person_follow_nav follow_nav_real.launch.py

  Co RSSI (3 board quet + beacon deo tren nguoi, 07/10). LiDAR va board RSSI cung chip CH340 nen
  PHAI dung by-path cho ca hai (by-id trung ten — sc_mini co the mo nham board RSSI):
  ros2 launch person_follow_nav follow_nav_real.launch.py start_rssi:=true \
      lidar_port:=$PL ports:="$PA,$PB,$PC"
  RSSI chi dung khi MAT NGUOI (SEARCH): xe xoay tai cho ~1 vong de do huong beacon, quay camera ve
  huong do, di toi roi do lai. Dang bam thi camera + LiDAR (chinh xac hon RSSI ~15 lan).

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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def _check_ports(context, *args, **kwargs):
    """Co RSSI ma LiDAR van de by-id -> canh bao (board RSSI va LiDAR cung chip CH340, trung ten by-id)."""
    out = []
    if LaunchConfiguration("start_rssi").perform(context).lower() in ("true", "1"):
        if "/by-id/" in LaunchConfiguration("lidar_port").perform(context):
            out.append(LogInfo(msg="CANH BAO: start_rssi:=true nhung lidar_port van la by-id — board RSSI va LiDAR "
                                   "cung chip CH340 nen by-id trung ten, sc_mini co the mo nham board. "
                                   "Truyen lidar_port:=/dev/serial/by-path/... (xem CLAUDE.md muc 1)."))
        if not LaunchConfiguration("ports").perform(context).strip():
            out.append(LogInfo(msg="CANH BAO: start_rssi:=true nhung chua truyen ports:=\"$PA,$PB,$PC\" "
                                   "(cong by-path cua 3 board quet)."))
    return out


def generate_launch_description() -> LaunchDescription:
    cfg = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "follow_nav.yaml"])
    identity_cfg = PathJoinSubstitution(
        [FindPackageShare("person_follow_robot"), "config", "identity_lock_kingsen.yaml"]
    )
    rssi_tmpl = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "rssi_template.json"])

    args = [
        DeclareLaunchArgument("config", default_value=cfg),
        DeclareLaunchArgument("identity_config", default_value=identity_cfg),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_rssi", default_value="false"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
        DeclareLaunchArgument("robot_port", default_value="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0"),
        # Cong by-path cua 3 board RSSI quet, cach nhau dau phay (giong rssi.launch.py / rssi_follow.launch.py)
        DeclareLaunchArgument("ports", default_value=""),
        DeclareLaunchArgument("rssi_template_file", default_value=rssi_tmpl),
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

    # ── RSSI (07/10: 2 node moi cua nhanh rssi-thaihoa thay rssi_serial_node cu cua robot_rssi_ros2 —
    # node cu doc giao thuc firmware cu, board gio chay firmware "scanner") ─────────────────────────────
    # rssi_bearing_node phat /rssi/bearing (huong beacon trong khung odom, chi hop le sau khi xe xoay
    # tai cho >= 250 do). follow_planner_node dung no trong SEARCH. Hai node nay KHONG ghi /cmd_vel.
    rssi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("person_follow_nav"), "launch", "rssi.launch.py"])),
        launch_arguments={
            "ports": LaunchConfiguration("ports"),
            "template_file": LaunchConfiguration("rssi_template_file"),
        }.items(),
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
        LogInfo(msg=" RSSI (start_rssi:=true): chi dung khi MAT NGUOI — xe xoay ~1 vong do huong beacon"),
        LogInfo(msg="=" * 66),
    ]

    return LaunchDescription(
        args + info + [OpaqueFunction(function=_check_ports), lidar, laser_tf, driver, identity, rssi, tracker,
                       planner]
    )
