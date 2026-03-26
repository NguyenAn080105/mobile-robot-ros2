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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_model_path  = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_world_path  = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    ekf_config_path     = os.path.join(pkg_share, 'config', 'ekf.yaml')
    filter_config       = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    nav2_params         = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file            = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    checkpoint_file     = os.path.join(pkg_share, 'config', 'checkpoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    timeout      = LaunchConfiguration('timeout_at_checkpoint')

    # ====================== GAZEBO ======================
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

    # ====================== ROBOT STATE PUBLISHER ======================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )

    # ====================== EKF & FILTER ======================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan',          '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    # ====================== SPAWN ENTITY ======================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05'
        ]
    )

    # ====================== NAV2 BRINGUP ======================
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          map_file,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
        }.items()
    )

    # ====================== RVIZ ======================
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', default_rviz_config]
    )

    # ====================== CHECKPOINT NAVIGATOR ======================
    navigator_node = Node(
        package='mobile_robot',
        executable='checkpoint_navigator.py',
        name='checkpoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'checkpoint_file':       checkpoint_file,
            'timeout_at_checkpoint': timeout,
            'home_checkpoint_id':    0,
            'goal_tolerance':        0.25,
        }]
    )

    # ====================== TIMING & DELAYS ======================
    delayed_spawn     = TimerAction(period=2.0,  actions=[spawn_entity])
    delayed_filter    = TimerAction(period=3.0,  actions=[scan_filter])
    delayed_nav2      = TimerAction(period=8.0,  actions=[nav2])
    delayed_rviz2     = TimerAction(period=12.0, actions=[rviz2])
    # nav2 starts at t=8s, lifecycle ~10s => ready ~18s
    delayed_navigator = TimerAction(period=15.0, actions=[navigator_node])

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),
        DeclareLaunchArgument('rvizconfig',   default_value=default_rviz_config),
        DeclareLaunchArgument('timeout_at_checkpoint', default_value='30.0'),

        # Immediate Actions
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        ekf_node,

        # Delayed Actions
        delayed_spawn,
        delayed_filter,
        delayed_nav2,
        delayed_rviz2,
        delayed_navigator,
    ])