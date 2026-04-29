import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ====================== PATH ======================
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot_v3.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    ekf_config_path    = os.path.join(pkg_share, 'config', 'ekf.yaml')
    filter_config      = os.path.join(pkg_share, 'config', 'laser_filter.yaml')

    # ====================== Launch Config ======================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_model   = LaunchConfiguration('model',        default=default_model_path)

    # ====================== Gazebo ======================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # ====================== Robot State Publisher ======================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
        }]
    )

    # ====================== EKF Node ======================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # ====================== LASER FILTER ======================
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ],
    )

    # ====================== ULTRASONIC FUSION ======================
    ultrasonic_fusion = Node(
        package='mobile_robot',
        executable='ultrasonic_fusion_node_v3.py',
        name='ultrasonic_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================== Spawn Entity ======================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot',
            '-x', '0', '-y', '0', '-z', '0.05'
        ]
    )

    delayed_spawn_entity       = TimerAction(period=1.0, actions=[spawn_entity])
    delayed_filter             = TimerAction(period=2.0, actions=[scan_filter])
    delayed_ultrasonic_fusion  = TimerAction(period=3.0, actions=[ultrasonic_fusion])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),

        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        ekf_node,
        delayed_spawn_entity,
        delayed_filter,
        delayed_ultrasonic_fusion,
    ])
