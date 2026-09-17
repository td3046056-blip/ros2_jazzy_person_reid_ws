from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/sc_mini'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        Node(
            package='sc_mini',
            executable='sc_mini',
            name='sc_mini',
            output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=str),
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'frame_id': ParameterValue(frame_id, value_type=str),
            }],
        ),
    ])
