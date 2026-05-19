import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            get_package_share_directory('mimic_sim'),
            'config',
            'config.toml',
        ),
        description='Path to mimic_sim config TOML file',
    )

    node = Node(
        package='mimic_sim',
        executable='mimic_sim_node',
        name='mimic_sim',
        parameters=[{'config_path': LaunchConfiguration('config')}],
        output='screen',
    )

    return LaunchDescription([config_arg, node])
