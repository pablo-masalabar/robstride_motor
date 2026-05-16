import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('remote_joystick'),
        'config', 'config.toml'
    )

    return LaunchDescription([
        Node(
            package='remote_joystick',
            executable='remote_joystick_node',
            name='remote_joystick_node',
            parameters=[{'config_path': config_path}],
            output='screen',
        ),
    ])
