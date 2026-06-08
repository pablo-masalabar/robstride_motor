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
                [FindPackageShare('realsense'), 'config', 'right_rs_d405.toml']
            ),
            description='Path to right RealSense D405 config toml file',
        ),

        Node(
            package='realsense',
            executable='realsense_node',
            name='right_realsense_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
            }],
        ),
    ])
