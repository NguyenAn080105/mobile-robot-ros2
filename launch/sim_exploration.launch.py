import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)

    default_slam_params   = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_model_path    = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_world_path    = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_rviz_config   = os.path.join(pkg_share, 'config', 'rviz', 'slam_config.rviz')
    nav2_params_path      = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world', default=default_world_path)

    # ====================== SIMULATION BRINGUP ======================
    spawn_sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_sim_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model':        LaunchConfiguration('model'),
            'world':        LaunchConfiguration('world'),
        }.items()
    )

    # ====================== SLAM TOOLBOX ======================
    slam_toolbox = Node(
        package='slam_toolbox', 
        executable='async_slam_toolbox_node',
        name='slam_toolbox', 
        output='screen',
        parameters=[
            default_slam_params,
            {'use_sim_time': use_sim_time, 'scan_topic': '/scan_filtered'}
        ]
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

    # ===== Nav2 (BT Navigator + Planner + Controller) =====
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_path,
        }.items()
    )

    # ===== Frontier Explorer =====
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

    # ===== Map Saver Service =====
    map_saver_service = Node(
        package='mobile_robot',
        executable='map_saver_service.py',
        name='map_saver_service',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================== TIMING ======================
    delayed_nav2   = TimerAction(period=3.0,  actions=[nav2])
    delayed_rviz2  = TimerAction(period=5.0, actions=[rviz2])
    delayed_slam   = TimerAction(period=3.0,  actions=[slam_toolbox])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),

        spawn_sim_robot,
        delayed_slam,
        delayed_nav2,
        delayed_rviz2,
        TimerAction(period=7.0, actions=[frontier_explorer]),
        TimerAction(period=7.0, actions=[map_saver_service]),
    ])