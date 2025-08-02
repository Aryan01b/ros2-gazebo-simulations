from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

model_name = 'diff_bot_sensors'

def generate_launch_description():
    # Path to the world file and robot model
    pkg_share = get_package_share_directory('world_01')
    world_file = os.path.join(pkg_share, 'worlds', 'world_keyboard.sdf')
    model_path = os.path.join(pkg_share, 'models', model_name, 'model.sdf')
    xacro_path = os.path.join(pkg_share, 'models', model_name, 'model.urdf.xacro')

    # Generate robot description from Xacro
    robot_description = Command(['xacro ', xacro_path])

    # Launch Gazebo with the specified world
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '-r', world_file],
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
            '-name', model_name,
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
            '/world/world_keyboard/model/' + model_name + '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # cmd_vel (ROS → Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # imu (Gazebo → ROS)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            # Remap Gazebo's joint state topic to standard ROS topic
            ('/world/world_keyboard/model/' + model_name + '/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # RViz node
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_sensors.rviz')
    
    # Ensure RViz config directory exists
    os.makedirs(os.path.dirname(rviz_config_file), exist_ok=True)

    # RViz node (using sensors configuration)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Command to save RViz config when RViz is closed
    save_rviz_config = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config_file, '--save-config-after-exit'],
        output='screen',
        shell=True
    )

    # IMU TF broadcaster node
    imu_tf_broadcaster_node = Node(
        package='dummy_data',
        executable='imu_tf_broadcaster',
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
        ros_gz_bridge_node,
        # IMU TF broadcaster nodes
        imu_tf_broadcaster_node,
        # Start RViz
        rviz_node,
        # Save RViz config when RViz exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[save_rviz_config]
            )
        )
    ])