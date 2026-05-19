import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mimic_sim')

    def motor_node(name, config):
        return Node(
            package='mimic_sim',
            executable='sim_motor_node',
            name=name,
            parameters=[{'config_path': os.path.join(pkg, 'config', config)}],
            output='screen',
        )

    return LaunchDescription([
        motor_node('left_arm',      'sim_left_arm.toml'),
        motor_node('right_arm',     'sim_right_arm.toml'),
        motor_node('neck',          'sim_neck.toml'),
        motor_node('base_brackets', 'sim_base_brackets.toml'),
        motor_node('base_wheels',   'sim_base_wheels.toml'),
        motor_node('torso',         'sim_torso.toml'),
    ])
