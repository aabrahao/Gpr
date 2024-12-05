# ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=world_file_path.sdf

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Define paths
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    pkg_project_description = get_package_share_directory('gpr_robot_description')

    # Define world
    world_file_path = os.path.join(pkg_project_description, 'worlds', 'mars_crater.sdf')

    # Define lunch_file
    gz_sim_launch_file = PythonLaunchDescriptionSource(os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py'))

    return LaunchDescription([
        # Include the ros_gz_sim launch file
        IncludeLaunchDescription( 
            gz_sim_launch_file, 
            launch_arguments={'gz_args': f'-r {world_file_path}'}.items()) 
        ])