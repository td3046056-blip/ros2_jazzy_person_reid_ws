import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ─── Đường dẫn ─────────────────────────────────────────────────────────
    pkg_robot_sim = get_package_share_directory('robot_simulation')
    pkg_description = get_package_share_directory('description')

    # Lấy URDF từ package description
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_robot_sim, 'rviz', 'robot.rviz')

    # ─── Launch arguments ───────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz     = LaunchConfiguration('use_rviz',     default='false')
    world_path = os.path.join(pkg_description, 'worlds', 'empty.sdf')
    world_file   = LaunchConfiguration('world',        default=world_path)

    # ─── Robot description từ xacro ────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ─── Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }]
    )

    # ─── Gazebo Harmonic ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ─── Spawn robot vào Gazebo ─────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ─── ROS-GZ Bridges ─────────────────────────────────────────────────────
    # /clock: Gz → ROS
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # /scan LiDAR: Gz → ROS
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
    )

    # /camera/image_raw: Gz → ROS
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen',
    )

    # /cmd_vel: ROS → Gz
    # /odom:    Gz → ROS
    # /tf:      Gz → ROS
    drive_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drive_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    # ─── RViz2 ──────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz2'),
        DeclareLaunchArgument('world', default_value=world_path,
                              description='Gazebo world file (tên file hoặc đường dẫn)'),

        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        scan_bridge,
        camera_bridge,
        drive_bridge,
        rviz,
    ])
