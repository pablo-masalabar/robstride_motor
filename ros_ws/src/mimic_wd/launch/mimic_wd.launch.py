import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mimic_wd')

    default_config = os.path.join(pkg_share, 'config', 'mimic.toml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Absolute path to the mimic_wd config toml file',
    )

    mimic_wd_node = Node(
        package='mimic_wd',
        executable='mimic_wd_node',
        name='mimic_wd_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config')}],
    )

    return LaunchDescription([config_arg, mimic_wd_node])
