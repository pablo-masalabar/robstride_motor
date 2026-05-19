import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file (SDF)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Absolute path to RViz2 config file (optional)',
    )

    world       = LaunchConfiguration('world')
    rviz        = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # ── GZ_SIM_RESOURCE_PATH so Gazebo resolves package:// mesh URIs ─────────
    # Gazebo converts package:// → model:// and searches GZ_SIM_RESOURCE_PATH
    # for a directory named after the package containing the meshes.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(get_package_share_directory('featherbot_description')),
    )

    # ── URDF via xacro ───────────────────────────────────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory('featherbot_description'),
        'urdf',
        'featherbot.xacro',
    )
    robot_description = Command(['xacro ', xacro_file, ' use_sim:=true'])

    # ── robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── Gazebo (Harmonic) ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Spawn robot in Gazebo ────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'featherbot',
            '-topic', 'robot_description',
            '-z', '0.5',
        ],
        output='screen',
    )

    # ── Controllers config ────────────────────────────────────────────────────
    controllers_yaml = os.path.join(
        get_package_share_directory('featherbot_ros2_control'),
        'config',
        'controllers.yaml',
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # joint_state_broadcaster must start before motion controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    neck_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'neck_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    base_wheel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'base_wheel_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    base_bracket_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'base_bracket_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    torso_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'torso_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── gz → ROS2 bridge ─────────────────────────────────────────────────────
    # Bridges /clock so use_sim_time works for all ROS2 nodes.
    # Add extra entries here for any Gazebo sensors (camera, IMU, lidar, etc.)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Motion controllers start after joint_state_broadcaster is active
    motion_controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_spawner,
                right_arm_spawner,
                neck_spawner,
                base_wheel_spawner,
                base_bracket_spawner,
                torso_spawner,
            ],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        world_arg,
        rviz_arg,
        rviz_config_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        rviz_node,
        joint_state_broadcaster_spawner,
        motion_controllers_after_jsb,
    ])
