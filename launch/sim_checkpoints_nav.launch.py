"""
sim_checkpoints_nav.launch.py
================================
Chạy toàn bộ hệ thống:
  - Gazebo + RSP + EKF + laser_filter + spawn + Nav2 + RViz
  - Checkpoint Navigator (chạy ngầm, nhận lệnh qua topic)

Cách dùng:
  ros2 launch mobile_robot sim_checkpoints_nav.launch.py

Sau khi launch xong, mở terminal khác để điều khiển:
  ros2 run mobile_robot send_checkpoint_command.py

Hoặc gửi topic trực tiếp:
  ros2 topic pub --once /robot/navigate_to_checkpoint std_msgs/msg/Int32 "data: 4"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share       = get_package_share_directory('mobile_robot')
    checkpoint_file = os.path.join(pkg_share, 'config', 'checkpoints.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    nav_only_arg = DeclareLaunchArgument(
        'nav_only', default_value='false',
        description='true = chỉ chạy navigator (sim_nav đã chạy rồi)')

    timeout_arg = DeclareLaunchArgument(
        'timeout_at_checkpoint', default_value='30.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_only     = LaunchConfiguration('nav_only')
    timeout      = LaunchConfiguration('timeout_at_checkpoint')

    sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim_nav.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(nav_only)
    )

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

    # sim_nav: nav2 starts at t=8s, lifecycle ~10s => ready ~18s
    delayed_navigator_full = TimerAction(
        period=15.0,
        actions=[navigator_node],
        condition=UnlessCondition(nav_only)
    )

    delayed_navigator_only = TimerAction(
        period=3.0,
        actions=[navigator_node],
        condition=IfCondition(nav_only)
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav_only_arg,
        timeout_arg,
        sim_nav,
        delayed_navigator_full,
        delayed_navigator_only,
    ])