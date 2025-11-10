# ROS2 Gazebo Simulations - Developer Guide

## Table of Contents
1. [Project Structure](#project-structure)
2. [Adding a New Robot](#adding-a-new-robot)
3. [Creating New Simulation Worlds](#creating-new-simulation-worlds)
4. [Extending Functionality](#extending-functionality)
5. [Building and Testing](#building-and-testing)

## Project Structure

```
src/
├── robots/                 # Robot descriptions and configurations
│   └── diffbot_description/  # Example robot package
│       ├── urdf/           # Robot URDF/XACRO files
│       ├── meshes/         # 3D model files
│       ├── launch/         # Launch files specific to this robot
│       └── config/         # Configuration files (controllers, etc.)
│
├── sim_bringup/            # Launch and configuration for simulation setups
│   ├── diff_empty_bringup/  # Example: Bringup for robot in empty world
│   └── diff_ros2_control/  # ROS2 Control configurations
│
└── sim_worlds/             # Gazebo world definitions
    └── empty_world/        # Example: Empty world definition
```

## Adding a New Robot

1. Create a new package in the `robots/` directory:
   ```bash
   cd src/robots/
   ros2 pkg create --build-type ament_cmake <robot_name>_description
   ```

2. Directory structure for a new robot:
   ```
   <robot_name>_description/
   ├── urdf/               # URDF/XACRO files
   ├── meshes/            # 3D model files (collada, STL, etc.)
   ├── launch/            # Robot-specific launch files
   ├── config/            # Configuration files
   │   ├── controllers/    # Controller configurations
   │   └── sensors/        # Sensor configurations
   ├── package.xml        # Package manifest
   └── CMakeLists.txt     # Build configuration
   ```

3. Update the `package.xml` with necessary dependencies:
   ```xml
   <depend>rclcpp</depend>
   <depend>robot_state_publisher</depend>
   <depend>joint_state_publisher</depend>
   <depend>gazebo_ros_pkgs</depend>
   ```

## Creating New Simulation Worlds

1. Create a new package in the `sim_worlds/` directory:
   ```bash
   cd src/sim_worlds/
   ros2 pkg create --build-type ament_cmake <world_name>_world
   ```

2. Add your world file (`.world`) to the package

3. Create a launch file to start the world:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import ExecuteProcess

   def generate_launch_description():
       return LaunchDescription([
           ExecuteProcess(
               cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                   '-s', 'libgazebo_ros_factory.so', 'path/to/your/world.world'],
               output='screen'
           )
       ])
   ```

## Extending Functionality

### Adding New Sensors
1. Add sensor plugins to your robot's URDF:
   ```xml
   <gazebo reference="sensor_link">
       <sensor type="camera" name="camera1">
           <update_rate>30.0</update_rate>
           <camera name="front">
               <horizontal_fov>1.3962634</horizontal_fov>
               <image>
                   <width>800</width>
                   <height>600</height>
                   <format>R8G8B8</format>
               </image>
               <clip>
                   <near>0.02</near>
                   <far>300</far>
               </clip>
           </camera>
           <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
               <alwaysOn>true</alwaysOn>
               <cameraName>camera1</cameraName>
               <imageTopicName>image_raw</imageTopicName>
               <cameraInfoTopicName>camera_info</cameraInfoTopicName>
               <frameName>camera_link</frameName>
           </plugin>
       </sensor>
   </gazebo>
   ```

### Adding ROS2 Control
1. Create controller configurations in `config/controllers/`
2. Update your robot's URDF with transmission elements
3. Create a launch file to load the controllers

## Building and Testing

1. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Launch a simulation:
   ```bash
   ros2 launch sim_bringup diff_empty_bringup.launch.py
   ```

3. Run tests:
   ```bash
   colcon test
   colcon test-result --verbose
   ```

## Debugging Tips

- View TF tree:
  ```bash
  ros2 run tf2_tools view_frames
  ```

- List topics:
  ```bash
  ros2 topic list
  ```

- View node graph:
  ```bash
  rqt_graph
  ```

- Inspect TF frames:
  ```bash
  ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
  ```

## Best Practices

1. Keep robot descriptions separate from simulation configurations
2. Use XACRO for complex URDFs
3. Document all launch file arguments
4. Include example launch files
5. Add appropriate dependencies in package.xml
6. Follow ROS 2 naming conventions
7. Write unit tests for new functionality

## Need Help?

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS 2 Control](https://control.ros.org/)

For specific questions, please open an issue in the repository.
