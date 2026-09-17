import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_rssi = get_package_share_directory('robot_rssi_ros2')
    vfh_params_file = os.path.join(pkg_robot_rssi, 'config', 'vfh_params.yaml')

    # Chạy node VFH với thông số ghi đè cho mô phỏng
    vfh_node = Node(
        package='robot_rssi_ros2',
        executable='vfh_rssi_controller',
        name='vfh_rssi_controller',
        output='screen',
        parameters=[
            vfh_params_file,
            {
                'use_sim_time': True,
                'lidar_front_center_deg': 0.0,  # Trong mô phỏng Lidar hướng thẳng = 0 độ
                'teleop_input_topic': '/cmd_vel_teleop',
                'start_enabled': True  # Bật chạy ngay lập tức không cần chờ cờ start
            }
        ]
    )

    # Chạy teleop để người dùng lái (cấp tín hiệu vào cmd_vel_teleop)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        prefix='gnome-terminal --',
        remappings=[('/cmd_vel', '/cmd_vel_teleop')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        vfh_node,
        teleop_node
    ])
