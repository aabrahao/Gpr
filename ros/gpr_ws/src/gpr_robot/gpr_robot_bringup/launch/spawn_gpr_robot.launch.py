#ros2 launch ros_gz_sim gz_spawn_model.launch.py
#   world:=empty 
#   file:=model.sdf
#   entity_name:=my_vehicle
#   x:=5.0 y:=5.0 z:=0.5

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Define paths
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    pkg_project_description = get_package_share_directory('gpr_robot_description')
    pkg_project_bringup = get_package_share_directory('gpr_robot_bringup')

    # Define paths
    model_path = os.path.join(pkg_project_description, 'models', 'gpr_robot', 'model.sdf')
    bridge_config_path = os.path.join(pkg_project_bringup, 'config', 'gpr_robot_bridge.yaml')

    # Define lunch file
    gz_sim_launch_file = PythonLaunchDescriptionSource(os.path.join(gz_sim_pkg, 'launch', 'gz_spawn_model.launch.py'))

    return LaunchDescription([
        # Include the ros_gz_sim launch file
        IncludeLaunchDescription( 
            gz_sim_launch_file, 
            launch_arguments={'file': f'{model_path}',
                              'entity_name' : 'gpr_robot_1',
                              'x' : '0.0',
                              'y' : '0.0',
                              'z' : '1.0',
                              'create_own_container' : 'True'}.items()) 
        ])
