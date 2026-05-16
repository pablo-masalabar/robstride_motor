import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mimic'),
        'config', 'right_s_left_t.toml'
    )

    return LaunchDescription([
        Node(
            package='mimic',
            executable='mimic_node',
            name='mimic_node',
            parameters=[{'config_path': config_path}],
            output='screen',
        ),
    ])
