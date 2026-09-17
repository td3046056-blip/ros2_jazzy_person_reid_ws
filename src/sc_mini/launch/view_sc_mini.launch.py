from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('sc_mini')
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'sc_m.rviz'])

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sc_mini.launch.py'])
        ),
        launch_arguments={
            'port': port,
            'baud_rate': baud_rate,
            'frame_id': frame_id,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/sc_mini'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        driver_launch,
        rviz,
    ])
