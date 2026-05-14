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
                [FindPackageShare('teleop'), 'config', 'config.toml']
            ),
            description='Path to teleop config.toml',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        Node(
            package='teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),
    ])
