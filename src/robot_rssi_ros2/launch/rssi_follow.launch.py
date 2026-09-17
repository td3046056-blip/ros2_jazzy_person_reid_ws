"""
rssi_follow.launch.py
======================
Khởi động toàn bộ stack RSSI follow độc lập:
  1. rssi_serial_node  — đọc Serial NodeMCU, publish /rssi/*
  2. rssi_follow_node  — điều khiển robot theo RSSI
  3. (tuỳ chọn) bw_dr03_decoded_serial_node — driver robot

Cách dùng:
  ros2 launch robot_rssi_ros2 rssi_follow.launch.py

Truyền tham số:
  ros2 launch robot_rssi_ros2 rssi_follow.launch.py \
      rssi_port:=/dev/ttyACM0 \
      robot_port:=/dev/ttyUSB1 \
      linear_speed:=0.12 \
      angular_speed:=0.35 \
      start_enabled:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Launch Arguments ─────────────────────────────────────────────
    rssi_port_arg = DeclareLaunchArgument(
        'rssi_port',
        default_value='/dev/ttyUSB0',
        description='Cổng Serial của NodeMCU Master (vd: /dev/ttyUSB0 hoặc /dev/ttyACM0)',
    )
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='/dev/bw_dr03',
        description='Cổng Serial của robot BW-DR03',
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.15',
        description='Tốc độ tiến thẳng (m/s)',
    )
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.40',
        description='Tốc độ quay (rad/s)',
    )
    start_enabled_arg = DeclareLaunchArgument(
        'start_enabled',
        default_value='false',
        description='Bật follow ngay khi khởi động (true/false)',
    )
    use_proportional_arg = DeclareLaunchArgument(
        'use_proportional',
        default_value='false',
        description='Dùng angular tỉ lệ với góc thay vì tốc độ cố định',
    )
    min_confidence_arg = DeclareLaunchArgument(
        'min_confidence',
        default_value='0.10',
        description='Ngưỡng tin cậy RSSI tối thiểu để điều khiển robot',
    )
    target_distance_arg = DeclareLaunchArgument(
        'target_distance_m',
        default_value='0.80',
        description='Khoảng cách mục tiêu (mét) — robot dừng tiến khi sonar <= giá trị này',
    )
    emergency_stop_arg = DeclareLaunchArgument(
        'emergency_stop_m',
        default_value='0.35',
        description='Dừng khẩn cấp khi sonar < giá trị này (mét)',
    )
    slowdown_arg = DeclareLaunchArgument(
        'slowdown_m',
        default_value='1.20',
        description='Bắt đầu giảm tốc khi sonar < giá trị này (mét)',
    )

    # ── Nodes ────────────────────────────────────────────────────────

    rssi_serial_node = Node(
        package='robot_rssi_ros2',
        executable='rssi_serial_node',
        name='rssi_serial_node',
        output='screen',
        parameters=[{
            'port':               LaunchConfiguration('rssi_port'),
            'baudrate':           115200,
            'signal_timeout_sec': 3.0,
        }],
    )

    rssi_follow_node = Node(
        package='robot_rssi_ros2',
        executable='rssi_follow_node',
        name='rssi_follow_node',
        output='screen',
        parameters=[{
            'linear_speed':       LaunchConfiguration('linear_speed'),
            'angular_speed':      LaunchConfiguration('angular_speed'),
            'use_proportional':   True,
            'min_confidence':     LaunchConfiguration('min_confidence'),
            'start_enabled':      LaunchConfiguration('start_enabled'),
            'target_distance_m':  LaunchConfiguration('target_distance_m'),
            'emergency_stop_m':   LaunchConfiguration('emergency_stop_m'),
            'slowdown_m':         LaunchConfiguration('slowdown_m'),
            'slow_speed':         0.07,
            'angle_enter_turn_deg': 30.0,
            'angle_exit_turn_deg': 15.0,
            'angle_ema_alpha':    0.45,
            'signal_timeout_sec': 2.0,
            'control_hz':         10.0,
            'cmd_vel_topic':      '/cmd_vel',
            'scan_topic':         '/scan',
            'lidar_front_center_deg': 90.0,
            'front_window_deg':   20.0,
            'scan_timeout_sec':   0.5,
        }],
    )

    bw_dr03_node = Node(
        package='bw_dr03_ros2',
        executable='decoded_serial_node',
        name='bw_dr03_decoded_serial_node',
        output='screen',
        parameters=[{
            'port':        LaunchConfiguration('robot_port'),
            'baudrate':    115200,
            'max_linear':  0.3,
            'max_angular': 1.0,
            'max_percent': 30,
            'cmd_timeout': 1.0,
        }],
    )

    return LaunchDescription([
        # Arguments
        rssi_port_arg,
        robot_port_arg,
        linear_speed_arg,
        angular_speed_arg,
        start_enabled_arg,
        use_proportional_arg,
        min_confidence_arg,
        target_distance_arg,
        emergency_stop_arg,
        slowdown_arg,

        # Info
        LogInfo(msg='============================================'),
        LogInfo(msg=' RSSI Follow Stack — Chế độ độc lập (không camera)'),
        LogInfo(msg='  /rssi/direction  → rssi_serial_node'),
        LogInfo(msg='  /rssi/command    → rssi_follow_node → /cmd_vel'),
        LogInfo(msg='  /cmd_vel         → bw_dr03_decoded_serial_node'),
        LogInfo(msg=''),
        LogInfo(msg='  Bật follow: ros2 service call /rssi_follow/enable std_srvs/srv/Trigger'),
        LogInfo(msg='  Dừng robot: ros2 service call /rssi_follow/stop std_srvs/srv/Trigger'),
        LogInfo(msg='============================================'),

        # Nodes
        rssi_serial_node,
        rssi_follow_node,
        bw_dr03_node,
    ])
