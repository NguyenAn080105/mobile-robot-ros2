# launch/rviz_view_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Chạy trên LAPTOP — chỉ khởi động RViz2
#
# Laptop sẽ tự động nhận các topic từ Jetson qua mạng WiFi (DDS discovery):
#   /scan, /scan_filtered, /robot_description, /tf, /tf_static
#
# Yêu cầu trước khi chạy (cả 2 máy):
#   export ROS_DOMAIN_ID=0
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # (nếu dùng FastDDS)
#
# Nếu WiFi chặn multicast, thêm trên cả 2 máy:
#   export ROS_DISCOVERY_SERVER=<IP_JETSON>:11811
#   (cần khởi động fast-discovery-server trên Jetson trước)
#
# Chạy trên laptop:
#   ros2 launch mobile_robot rviz_view_launch.py
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)

    # ── RViz2 config ───────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'nav_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False,
            'tf_buffer_size': 30.0,
        }]
    )

    return LaunchDescription([
        rviz_node,
    ])