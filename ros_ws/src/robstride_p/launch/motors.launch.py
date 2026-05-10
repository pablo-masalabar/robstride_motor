"""
motors.launch.py – Launch the RobStride motor node.

Driver files live inside the robstride_p Python package and are imported
automatically — no extra PYTHONPATH manipulation required.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robstride_p')

    default_config = os.path.join(pkg_share, 'config', 'config.toml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Absolute path to the motor config.toml file',
    )

    motor_node = Node(
        package='robstride_p',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config')}],
    )

    return LaunchDescription([config_arg, motor_node])
