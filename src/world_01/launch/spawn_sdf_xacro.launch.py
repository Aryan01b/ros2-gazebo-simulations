from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the world file and robot model
    pkg_share = get_package_share_directory('world_01')
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    model_path = os.path.join(pkg_share, 'models', 'diff_bot', 'model.sdf')
    xacro_path = os.path.join(pkg_share, 'models', 'diff_bot', 'model.urdf.xacro')

    # Generate robot description from Xacro
    robot_description = Command(['xacro ', xacro_path])

    # Launch Gazebo with the specified world
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
        shell=True
    )

    # robot_state_publisher node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn the robot
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-file', model_path,
        ],
        output='screen',
    )

    # Delay the spawn command to ensure Gazebo is fully initialized
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity_cmd]
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Joint states (Gazebo → ROS)
            '/world/world_demo/model/diff_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # cmd_vel (ROS → Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            # Remap Gazebo's joint state topic to standard ROS topic
            ('/world/world_demo/model/diff_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo
        gz_sim,
        # robot_state_publisher node
        robot_state_pub,
        # Delay the spawn to ensure everything is ready
        delayed_spawn,
        # ros_gz_bridge node    
        ros_gz_bridge_node
    ])