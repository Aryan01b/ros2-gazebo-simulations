from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Package and model information
    model_pkg_name = 'diffbot_description'
    world_pkg_name = 'empty_world'
    model_name = 'diffbot_sensor'
    
    # Get package share directory
    model_pkg_share = get_package_share_directory(model_pkg_name)
    world_pkg_share = get_package_share_directory(world_pkg_name)
    
    # Path to the world file and robot model
    world_file = os.path.join(world_pkg_share, 'worlds', 'empty_world_for_xacro.sdf')
    xacro_file = os.path.join(model_pkg_share, 'urdf', model_name, 'diffbot_sensor.urdf.xacro')
    
    # Launch Gazebo with the specified world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )
    
    # Spawn the robot in Gazebo with a delay
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', model_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Delay spawn until Gazebo is ready
    delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_sim,
            on_start=[
                TimerAction(period=3.0, actions=[
                    robot_state_publisher,
                    spawn_entity
                ])
            ]
        )
    )

    # ROS-Ignition bridges - all in one parameter_bridge node
    bridge_cmds = [
        # Main bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Joint state bridge with remapping
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/world/empty_world/model/{model_name}/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'],
            remappings=[(f'/world/empty_world/model/{model_name}/joint_state', '/joint_states')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ]

    
    return LaunchDescription([
        gz_sim,
        delayed_spawn,
        *bridge_cmds
    ])