import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Set simulation time to true for Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package directory - Update 'mobile_robot' if your package name differs
    pkg_share = get_package_share_directory('mobile_robot')

    # File paths
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sim.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'room2.world')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_config.rviz')
    
    # Read URDF content for Robot State Publisher
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Configure Gazebo Model Path
    model_path = os.path.join(pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path
        
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', # Foxy Syntax
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc # Passed as string
        }]
    )

    # Spawn Entity Node
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

    # SLAM Toolbox Node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', # Foxy Syntax
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}   
        ],
    )

    # RViz2 Node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2', # Foxy Syntax
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    # Sequence timing
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])
    delayed_slam = TimerAction(period=7.0, actions=[slam_toolbox])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_slam,
        rviz2
    ])