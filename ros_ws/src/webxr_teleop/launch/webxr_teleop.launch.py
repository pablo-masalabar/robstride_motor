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
                [FindPackageShare('webxr_teleop'), 'config', 'config.toml']
            ),
            description='Path to webxr_teleop config toml file',
        ),

        Node(
            package='webxr_teleop',
            executable='webxr_teleop_node',
            name='webxr_teleop_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
            }],
        ),
    ])
