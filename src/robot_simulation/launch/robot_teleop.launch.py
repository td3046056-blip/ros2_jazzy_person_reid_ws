import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Teleop keyboard - runs in current terminal with emulate_tty for stdin
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        prefix='gnome-terminal --',
        remappings=[('/cmd_vel', '/cmd_vel')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # TwistStamper: converts Twist -> TwistStamped
    twist_stamper = Node(
        package='robot_simulation',
        executable='twist_stamper.py',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        teleop_keyboard,
        twist_stamper,
    ])
