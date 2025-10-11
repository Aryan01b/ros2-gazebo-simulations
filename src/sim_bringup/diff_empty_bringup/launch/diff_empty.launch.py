from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Package and model information
    model_pkg_name = 'diffbot_description'
    world_pkg_name = 'empty_world'
    model_name = 'diffbot'
    
    # Get package share directory
    model_pkg_share = get_package_share_directory(model_pkg_name)
    world_pkg_share = get_package_share_directory(world_pkg_name)
    
    # Path to the world file and robot model
    world_file = os.path.join(world_pkg_share, 'worlds', 'empty_world.sdf')
    model_file = os.path.join(model_pkg_share, 'models', model_name, 'model.sdf')
    

    # Launch Gazebo with the specified world
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen'
    )
    
    # Spawn the robot in Gazebo with a delay
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_file,
            '-name', model_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )

    # Delay spawn until Gazebo is ready
    delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_sim,
            on_start=[TimerAction(period=3.0, actions=[spawn_entity])]
        )
    )

    # ROS-Gazebo bridge for cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    

    return LaunchDescription([
        gz_sim,
        delayed_spawn,
        cmd_vel_bridge,
    ])