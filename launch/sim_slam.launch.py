import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot')
    
    # Configuration paths
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sim.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'room2.world')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_config.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )
    
    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', # Foxy Syntax
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }],
        output='screen',
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py', # Foxy Syntax
        arguments=[
            '-entity', 'mobile_robot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', # Foxy Syntax
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': True}   
        ]
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2', # Foxy Syntax
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    # Execution Flow
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    delayed_slam = TimerAction(period=5.0, actions=[slam_toolbox])

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        delayed_spawn,
        delayed_slam,
        rviz2
    ])