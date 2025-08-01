from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the world file
    world_file = os.path.join(
        get_package_share_directory('world_01'),
        'worlds',
        'world.sdf'
    )
    
    # Launch ros_gz_sim as an executable
    gz_sim = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 
             f'gz_args:=-r {world_file}'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim
    ])