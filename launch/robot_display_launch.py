# launch/robot_display_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Hiển thị hình dạng robot trên Gazebo + RViz (tối giản)
#
# FIX:
#   - Dùng IncludeLaunchDescription từ gazebo_ros thay vì ExecuteProcess('gazebo')
#     → Gazebo được launch đúng cách với đầy đủ biến môi trường ROS,
#       giúp resolve package:// path trong URDF (meshes, plugins...)
#   - Bỏ joint_state_publisher độc lập: Gazebo đã có plugin
#     gazebo_ros_joint_state_publisher publish /joint_states
#   - Tăng delay spawn_entity lên 3s, RViz lên 5s để tránh
#     warning "Invalid frame left_wheel/right_wheel"
#
# Chạy:
#   ros2 launch mobile_robot robot_display_launch.py
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)
    gazebo_ros   = get_package_share_directory('gazebo_ros')

    urdf_model  = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'robot_display.rviz')

    use_sim_time = True

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
    # Publish /robot_description + broadcast TF cố định từ URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model]),
            'publish_frequency': 15.0,   # publish định kỳ để Gazebo và RViz nhận được
        }]
    )

    # ── 2. Gazebo ──────────────────────────────────────────────────────────────
    # QUAN TRỌNG: Dùng IncludeLaunchDescription từ gazebo_ros để Gazebo
    # kế thừa đúng biến môi trường ROS (AMENT_PREFIX_PATH, v.v.)
    # → resolve được "package://mobile_robot/meshes/..." trong URDF
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause':   'false',
        }.items(),
    )

    # ── 3. Spawn robot vào Gazebo (delay 3s để Gazebo load xong) ──────────────
    # Sau khi spawn, Gazebo plugin sẽ tự publish /joint_states
    # (gazebo_ros_joint_state_publisher trong gazebo_control.xacro)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot',
            '-x', '0', '-y', '0', '-z', '0.15',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 4. RViz2 (delay 5s để robot spawn xong và TF ổn định) ─────────────────
    # Cần đợi Gazebo spawn entity → gazebo plugin publish /joint_states
    # → robot_state_publisher broadcast TF left_wheel / right_wheel
    # Nếu vẫn còn warning, tăng delay lên 6-7s
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        LogInfo(msg='[robot_display] Starting robot_state_publisher...'),
        robot_state_publisher,

        LogInfo(msg='[robot_display] Starting Gazebo via gazebo_ros...'),
        gazebo,

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[robot_display] Spawning robot entity...'),
                spawn_entity,
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[robot_display] Starting RViz2...'),
                rviz2,
            ]
        ),
    ])