from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('zed'), 'config', 'config.toml']
            ),
            description='Path to zed config toml file',
        ),

        Node(
            package='zed',
            executable='zed_node',
            name='zed_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
            }],
        ),
    ])
