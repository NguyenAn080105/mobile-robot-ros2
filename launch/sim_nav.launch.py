import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess,  LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_model_path  = os.path.join(pkg_share, 'urdf', 'mobile_robot_v3.urdf.xacro')
    default_world_path  = os.path.join(pkg_share, 'worlds', 'sim_room_v2.world')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'nav_fusion_config.rviz')
    nav2_params         = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file            = os.path.join(pkg_share, 'maps', 'sim_map_v2.yaml')
    checkpoint_file     = os.path.join(pkg_share, 'config', 'checkpoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    timeout      = LaunchConfiguration('timeout_at_checkpoint')

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
        executable='navigator.py',
        name='navigator',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'checkpoint_file':       checkpoint_file,
            'timeout_at_checkpoint': timeout,
            'home_checkpoint_id':    0,
            'goal_tolerance':        0.25,
        }]
    )

    # ====================== TIMING ======================
    delayed_nav2      = TimerAction(period=3.0,  actions=[nav2])
    delayed_rviz2     = TimerAction(period=5.0, actions=[rviz2])
    delayed_navigator = TimerAction(period=7.0, actions=[navigator_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',           default_value='true'),
        DeclareLaunchArgument('model',                  default_value=default_model_path),
        DeclareLaunchArgument('world',                  default_value=default_world_path),
        DeclareLaunchArgument('timeout_at_checkpoint',  default_value='30.0'),

        spawn_sim_robot,
        delayed_nav2,
        delayed_rviz2,
        delayed_navigator,
    ])