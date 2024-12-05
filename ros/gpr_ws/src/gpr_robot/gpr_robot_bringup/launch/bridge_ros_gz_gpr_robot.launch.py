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
    gz_bridge_pkg = get_package_share_directory('ros_gz_bridge')
    pkg_project_bringup = get_package_share_directory('gpr_robot_bringup')

    # Define paths
    bridge_config_path = os.path.join(pkg_project_bringup, 'config', 'gpr_robot_bridge.yaml')

    # Define lunch file
    gz_bridge_launch_file = PythonLaunchDescriptionSource(os.path.join(gz_bridge_pkg, 'launch', 'ros_gz_bridge.launch.py'))

    return LaunchDescription([
        # Include the ros_gz_sim launch file
        IncludeLaunchDescription( 
            gz_bridge_launch_file, 
            launch_arguments={'bridge_name': 'ros_gz_bridge',
                              'config_file' : f'{bridge_config_path}'}.items()) 
        ])
