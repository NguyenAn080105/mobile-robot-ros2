import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # --- Paths ---
    default_slam_params   = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    filter_config         = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    default_model_path    = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_world_path    = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_rviz_config   = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    ekf_config_path       = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_params_path      = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_model   = LaunchConfiguration('model',        default=default_model_path)
    world        = LaunchConfiguration('world',        default=default_world_path)

    # ===== Nodes giống cũ =====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_model])}]
    )
    robot_localization_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )
    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        parameters=[filter_config, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mobile_robot',
                   '-x', '0', '-y', '0', '-z', '0.05']
    )
    slam_toolbox = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[default_slam_params,
                    {'use_sim_time': use_sim_time, 'scan_topic': '/scan_filtered'}]
    )
    rviz2 = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', default_rviz_config]
    )

    # ===== Nav2 (BT Navigator + Planner + Controller) =====
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_path,
        }.items()
    )

    # ===== NEW: Frontier Explorer =====
    frontier_explorer = Node(
        package='mobile_robot',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'min_frontier_size':   5,
            'robot_radius':        0.25,
            'exploration_timeout': 300.0,
        }]
    )

    # ===== NEW: Map Saver Service =====
    map_saver_service = Node(
        package='mobile_robot',
        executable='map_saver_service.py',
        name='map_saver_service',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),

        gazebo,
        robot_state_publisher,
        robot_localization_node,

        TimerAction(period=1.0,  actions=[spawn_entity]),
        TimerAction(period=2.0,  actions=[scan_filter]),
        TimerAction(period=3.0,  actions=[nav2]),
        TimerAction(period=6.0,  actions=[slam_toolbox]),
        TimerAction(period=10.0, actions=[frontier_explorer]),
        TimerAction(period=10.0, actions=[map_saver_service]),
        TimerAction(period=11.0, actions=[rviz2]),
    ])