import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # 1. Path & Identity Configuration
    # ==========================================
    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Default paths for assets
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'sim_room.world')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # Launch Configurations (Substitutions)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_model = LaunchConfiguration('model', default=default_model_path)
    world_file = LaunchConfiguration('world', default=default_world_path)

    # ==========================================
    # 2. Xacro Processing
    # ==========================================
    robot_description_content = Command(['xacro ', urdf_model])

    # ==========================================
    # 3. Node & Action Definitions
    # ==========================================

    # 3.1 Gazebo Simulation Environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 3.2 Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # 3.3 Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3.4 Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot_model',
            '-x', '0', '-y', '0', '-z', '0.01'
        ]
    )

    # 3.5 RViz2 Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', default_rviz_config]
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    # ==========================================
    # 4. Final Launch Description
    # ==========================================
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot xacro file'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Absolute path to world file'
        ),

        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn,
        rviz2
    ])