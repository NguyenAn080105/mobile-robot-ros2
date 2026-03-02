import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Path to the joystick device'
    )

    joy_config_arg = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(
            get_package_share_directory('mobile_robot'), 
            'config', 
            'joystick.yaml'
        ),
        description='Path to config file for teleop_twist_joy'
    )


    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,
            'scale_linear.x': 0.5,
            'axis_angular.yaw': 0,
            'scale_angular.yaw': 1.0,
            'enable_button': 0,
            'require_enable_button': True
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel')      
        ]
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node
    ])