import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _preferred_python() -> str:
    # Ưu tiên 1: VIRTUAL_ENV đang active
    venv = os.environ.get("VIRTUAL_ENV", "")
    if venv:
        candidate = os.path.join(venv, "bin", "python")
        if os.path.exists(candidate):
            return candidate
    # Ưu tiên 2: .venv trong workspace
    ws_root = os.path.normpath(os.path.join(
        get_package_share_directory('robot_simulation'),
        '..', '..', '..', '..'
    ))
    candidate = os.path.join(ws_root, '.venv', 'bin', 'python')
    if os.path.exists(candidate):
        return candidate
    return sys.executable


def generate_launch_description():
    # ─── Đường dẫn ─────────────────────────────────────────────────────────
    pkg_robot_sim = get_package_share_directory('robot_simulation')
    pkg_follower = get_package_share_directory('thaihoa_follower_robot_sim')
    pkg_description = get_package_share_directory('description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_robot_sim, 'rviz', 'robot.rviz')

    # configs
    reid_params = os.path.join(pkg_follower, 'config', 'ros_image_reid_sim.yaml')
    actor_world = os.path.join(pkg_follower, 'worlds', 'follow_actor_world.sdf')

    # ─── Launch arguments ───────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz     = LaunchConfiguration('use_rviz',     default='false')
    world_file   = LaunchConfiguration('world',        default=actor_world)

    # ─── Robot description từ xacro (y hệt robot_sim.launch.py) ───────────
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

    # ─── Spawn robot vào Gazebo (y hệt robot_sim.launch.py) ───────────────
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
            '/model/person_standing/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/model/actor/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
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

    # ─── Mock BLE RSSI Node (Giả lập tín hiệu BLE từ người) ──────────────────
    mock_ble_node = Node(
        package='thaihoa_follower_robot_sim',
        executable='mock_ble_rssi',
        name='mock_ble_rssi',
        output='screen',
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "actor_x": 4.0,   # Vị trí người giả lập trong sdf
                "actor_y": 0.0,
                "tx_power": -59.0,
                "path_loss_exponent": 2.5,
                "noise_stddev": 3.0,
            }
        ]
    )

    # ─── NAV2 Local Navigation (MPPI) ─────────────────────────────────────────
    nav2_params_file = os.path.join(pkg_robot_sim, 'config', 'nav2_mppi_params.yaml')
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # ─── Nav2 RSSI Tracker Node ───────────────────────────────────────────────
    nav2_rssi_tracker = Node(
        package='robot_rssi_ros2',
        executable='nav2_rssi_tracker',
        name='nav2_rssi_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_distance_m': 1.5,
            'goal_update_hz': 2.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz2'),
        DeclareLaunchArgument('world', default_value=actor_world,
                              description='Gazebo world file'),

        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        scan_bridge,
        camera_bridge,
        drive_bridge,
        rviz,
        mock_ble_node,
        nav2_bringup,
        nav2_rssi_tracker
    ])
