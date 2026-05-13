import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mimic')

    default_config = os.path.join(pkg_share, 'config', 'config.toml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Absolute path to the mimic config.toml file',
    )

    mimic_node = Node(
        package='mimic',
        executable='mimic_node',
        name='mimic_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config')}],
    )

    return LaunchDescription([config_arg, mimic_node])
