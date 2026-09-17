from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bw_dr03_ros2',
            executable='decoded_serial_node',
            name='bw_dr03_decoded_serial_node',
            output='screen',
            parameters=[
                {'port': '/dev/bw_dr03'},
                {'baudrate': 115200},

                # Thông số điều khiển đã calib thực tế
                {'max_percent': 30},
                {'max_linear': 0.226},
                {'max_angular': 1.10},

                # Thông số odometry đã calib
                {'wheel_radius': 0.0632},
                {'wheel_separation': 0.40},
                {'encoder_ppr_default': 750},

                # Frame ROS
                {'odom_frame_id': 'odom'},
                {'base_frame_id': 'base_link'},
                {'publish_tf': True},

                # Nếu mất cmd_vel quá 1 giây thì tự dừng
                {'cmd_timeout': 1.0},
            ]
        )
    ])
