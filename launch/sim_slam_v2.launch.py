import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)

    default_model_path  = os.path.join(pkg_share, 'urdf', 'mobile_robot_v3.urdf.xacro')
    default_world_path  = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'slam_config.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ====================== SIMULATION BRINGUP ======================
    spawn_sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_sim_robot_v2.launch.py')
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

    # ====================== TIMING ======================
    delayed_slam   = TimerAction(period=3.0,  actions=[slam_toolbox])
    delayed_rviz2  = TimerAction(period=5.0,  actions=[rviz2])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value=default_model_path),
        DeclareLaunchArgument('world',        default_value=default_world_path),

        spawn_sim_robot,
        delayed_slam,
        delayed_rviz2,
    ])