"""
test_avoid_only.launch.py
=========================
Chay CHI LiDAR + driver + planner, KHONG camera, KHONG RSSI.

Dung de test rieng phan ne vat can truoc khi ghep camera vao — day la buoc
ban NEN lam dau tien vi no loai bo hoan toan bien so ReID khoi phuong trinh.

Cach test:
  T1:  ros2 launch person_follow_nav test_avoid_only.launch.py
  T2:  # Gia lap "nguoi" dung im cach xe 2.5m thang truoc mat:
       ros2 topic pub -r 10 /follow/target std_msgs/msg/String \
         "{data: '{\"valid\":true,\"confidence\":1.0,\"source\":\"fake\",
                   \"base_x\":2.5,\"base_y\":0.0,\"distance_m\":2.5,
                   \"bearing_rad\":0.0,\"in_camera_fov\":true}'}"
  T3:  ros2 service call /follow/enable std_srvs/srv/Trigger {}

  Roi dat thung carton vao giua xe va diem dich. Xe phai vong qua no.
  Doi vi tri thung sang trai/phai de xem xe co chon dung ben khong.

  Theo doi:  ros2 topic echo /follow/planner_status
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cfg = PathJoinSubstitution([FindPackageShare("person_follow_nav"), "config", "follow_nav.yaml"])

    args = [
        DeclareLaunchArgument("config", default_value=cfg),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
        DeclareLaunchArgument("robot_port", default_value="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0"),
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

    planner = Node(
        package="person_follow_nav", executable="follow_planner",
        name="follow_planner_node", output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription(args + [
        LogInfo(msg="TEST NE VAT CAN — khong camera. Xem huong dan trong file launch."),
        lidar, laser_tf, driver, planner,
    ])
