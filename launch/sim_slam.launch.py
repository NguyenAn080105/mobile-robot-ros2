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

    default_slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    filter_config = os.path.join(pkg_share, 'config', 'laser_filter.yaml')

    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_rviz_config = os.path.join(pkg_share, 'config', 'slam_config.rviz')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_model = LaunchConfiguration('model', default=default_model_path)
    world = LaunchConfiguration('world', default=default_world_path)

    # ====================== NODES ======================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # ====================== ROBOT STATE PUBLISHER ======================
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

    # ====================== EKF ======================
    robot_localization_node = Node(
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
            ('scan',          '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    # ====================== SPAWN ROBOT ======================
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

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', default_rviz_config]
    )

    delayed_spawn = TimerAction(period=1.0, actions=[spawn_entity])
    delayed_filter = TimerAction(period=2.0, actions=[scan_filter])
    delayed_rviz2 = TimerAction(period=3.0, actions=[rviz2])
    delayed_slam = TimerAction(period=4.0, actions=[slam_toolbox])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('world', default_value=default_world_path),
        gazebo,
        robot_state_publisher,
        robot_localization_node,
        delayed_spawn,
        delayed_filter,
        delayed_slam,
        delayed_rviz2
    ])