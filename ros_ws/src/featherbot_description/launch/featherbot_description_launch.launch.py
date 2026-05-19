from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_name = "featherbot_description"
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(
        pkg_path,
        "urdf",
        "featherbot.urdf.xacro"
    )
    robot_description_config = Command(["xacro ", xacro_file])
    params = {
        "robot_description": robot_description_config,
        "use_sim_time":False
    }

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    ld.add_action(rsp_node)
    return ld