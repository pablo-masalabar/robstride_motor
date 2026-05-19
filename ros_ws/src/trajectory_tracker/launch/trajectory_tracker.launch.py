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
                [FindPackageShare('trajectory_tracker'), 'config', 'config.toml']
            ),
            description='Path to trajectory_tracker config toml file',
        ),

        Node(
            package='trajectory_tracker',
            executable='trajectory_tracker_node',
            name='trajectory_tracker_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'config_dir': PathJoinSubstitution(
                    [FindPackageShare('trajectory_tracker'), 'config']
                ),
            }],
        ),
    ])
