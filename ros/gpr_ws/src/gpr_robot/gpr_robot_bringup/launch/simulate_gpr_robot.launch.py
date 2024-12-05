#ros2 launch ros_gz_bridge ros_gz_bridge.launch.py 
#   bridge_name:=ros_gz_bridge 
#   config_file:=<path_to_your_YAML_file>

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Define paths
    pkg_project_bringup = get_package_share_directory('gpr_robot_bringup')

    # Define lunch file
    world_launch_file = PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'start_mars_crater_world.launch.py'))
    robot_launch_file = PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'spawn_gpr_robot.launch.py'))
    bridge_launch_file = PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'bridge_ros_gz_gpr_robot.launch.py'))

    return LaunchDescription([
        IncludeLaunchDescription(world_launch_file),
        IncludeLaunchDescription(robot_launch_file),
        IncludeLaunchDescription(bridge_launch_file) 
        ])
