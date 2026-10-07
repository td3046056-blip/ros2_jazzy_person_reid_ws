"""
rssi_follow.launch.py
=====================
BAM THEO bang RSSI + LiDAR, KHONG camera: lidar + driver + 2 node RSSI + follow_planner.

KHAC cac launch khac: o day planner ghi lenh vao /cmd_vel_follow (KHONG phai /cmd_vel).
Nguon DUY NHAT ghi /cmd_vel la scripts/rssi_follow.py — no tu xoay xe de do huong beacon, va chuyen tiep
lenh cua planner khi dang bam. Khong chay rssi_follow.py thi xe KHONG nhuc nhich.

  T1: ros2 launch person_follow_nav rssi_follow.launch.py lidar_port:=$PL ports:="$PA,$PB,$PC"
  T2: python3 src/person_follow_nav/scripts/rssi_follow.py

  DUNG KHAN: Ctrl-C o T2, hoac (moi luc)  ros2 service call /follow/stop std_srvs/srv/Trigger {}
             O launch nay /follow/stop do rssi_follow.py nhan (planner bi doi ten service dung thanh
             /rssi_follow/planner_stop): luc script tu xoay do / lui thi planner von da tat, /follow/stop cua planner
             se KHONG dung duoc xe (mo phong 03/10: xe chay tiep 2 m). /rssi_follow/stop cung dung duoc.

Khong co target_tracker_node va khong co camera: /follow/target do rssi_follow.py phat (RSSI nhan chu,
LiDAR bam cum chan). Planner dung NGUYEN config/follow_nav.yaml, chi doi hai tham so va ten service dung ngay tai day:
  /follow/stop       doi ten thanh /rssi_follow/planner_stop (xem DUNG KHAN o tren)
  cmd_vel_topic      /cmd_vel_follow
  occluded_turn_deg  15 -> 60. Nguong "nguon khong phai camera ma nguoi lech qua ngan nay -> XOAY TAI CHO ve phia
                     nguoi truoc roi moi tien". 15 do la de quay CAMERA co dinh (FOV 62 do) ve phia nguoi. O day
                     khong co camera, LiDAR thay nguoi o moi huong (tru +-24 do thang sau duoi), ma nguon luon la
                     "rssi+lidar" nen nhanh nay bat SUOT: moi lan ne vat can can lech huong nguoi qua 15 do la xe
                     dung lai xoay nguoc ve phia nguoi, roi lai ne -> giang co, chuc mui vao vat can roi dung im
                     (mo phong 02/10, kich ban vong qua thung). 60 do = dung nguong pha "di toi" cua SEARCH trong
                     planner (cung ly do: nguong thap lam xe dung-xoay-di). Nguoi lech qua 60 do van xoay tai cho.
                     KHONG anh huong follow_nav_real.launch.py (van 15 do, co camera).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LIDAR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BASE_PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0"


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("person_follow_nav")
    cfg = PathJoinSubstitution([share, "config", "follow_nav.yaml"])
    tmpl = PathJoinSubstitution([share, "config", "rssi_template.json"])

    args = [
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value=LIDAR_PORT),
        DeclareLaunchArgument("robot_port", default_value=BASE_PORT),
        DeclareLaunchArgument("ports", default_value=""),
        DeclareLaunchArgument("template_file", default_value=tmpl),
        DeclareLaunchArgument("config", default_value=cfg),
        DeclareLaunchArgument("occluded_turn_deg", default_value="60.0"),
    ]

    # lidar + TF + driver: dung lai calibrate.launch.py de tham so driver (max_linear 0.49, ...) chi nam mot cho
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "calibrate.launch.py"])),
        launch_arguments={
            "start_lidar": LaunchConfiguration("start_lidar"),
            "start_driver": LaunchConfiguration("start_driver"),
            "lidar_port": LaunchConfiguration("lidar_port"),
            "robot_port": LaunchConfiguration("robot_port"),
        }.items(),
    )
    rssi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "rssi.launch.py"])),
        launch_arguments={
            "ports": LaunchConfiguration("ports"),
            "template_file": LaunchConfiguration("template_file"),
        }.items(),
    )

    planner = Node(
        package="person_follow_nav", executable="follow_planner", name="follow_planner_node", output="screen",
        # /follow/stop phai dung duoc xe MOI LUC: rssi_follow.py nhan ten do; service dung cua planner doi ten
        remappings=[("/follow/stop", "/rssi_follow/planner_stop")],
        parameters=[
            LaunchConfiguration("config"),
            {
                # Planner KHONG ghi thang /cmd_vel o che do nay — rssi_follow.py la nguon duy nhat
                "cmd_vel_topic": "/cmd_vel_follow",
                "occluded_turn_deg": ParameterValue(LaunchConfiguration("occluded_turn_deg"), value_type=float),
            },
        ],
    )

    info = [
        LogInfo(msg="=" * 68),
        LogInfo(msg=" BAM THEO BANG RSSI + LiDAR (khong camera)"),
        LogInfo(msg="   Planner ghi /cmd_vel_follow. Xe chi chay khi co scripts/rssi_follow.py"),
        LogInfo(msg="   DUNG KHAN: Ctrl-C o rssi_follow.py, hoac (moi luc)"),
        LogInfo(msg="              ros2 service call /follow/stop std_srvs/srv/Trigger {}"),
        LogInfo(msg="=" * 68),
    ]
    return LaunchDescription(args + info + [base, rssi, planner])
