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
                [FindPackageShare('ezmotion_p'), 'config', 'config.toml']
            ),
            description='Path to ezmotion config toml file',
        ),

        Node(
            package='ezmotion_p',
            executable='ezmotion_node',
            name='ezmotion_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
            }],
        ),
    ])
