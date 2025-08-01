# World 01 - Developer's Guide

## Overview
This document provides a comprehensive guide to the structure and workflow of the World 01 simulation package, which is built on ROS 2 Humble and Gazebo Fortress with ros_gz packages and xacro dependencies.

## Project Structure
```
world_01/
├── docs/                   # Documentation files
├── launch/                 # Launch files
│   ├── spawn_sdf_xacro.launch.py  # Main entry point
│   └── world.launch.py     # World spawning implementation
├── models/                 # Robot and environment models
│   └── diff_bot/           # Example robot model
│       ├── config/         # Model configuration
│       ├── model.sdf       # Gazebo SDF model definition
│       └── model.urdf.xacro     # Xacro model for robot state publishing
└── worlds/                 # Gazebo world files
    └── world.sdf           # Basic world with sun and ground
```

## Key Components

### 1. Worlds
Location: `worlds/`
- Contains SDF world files for Gazebo simulation
- Basic world includes essential elements:
  - Sun lighting
  - Ground plane
  - Basic physics properties

### 2. Models
Location: `models/diff_bot/`

#### 2.1 Model Configuration (`config/`)
- Contains metadata about the robot model
- Includes basic information like name, version, and description

#### 2.2 SDF Model (`model.sdf`)
- Defines the robot's physical properties for Gazebo
- Includes:
  - Visual and collision geometries
  - Inertial properties
  - Sensors and their properties
  - Plugins for control and simulation

#### 2.3 Xacro Model (`model.urdf.xacro`)
- Used for robot state publishing in ROS 2
- Defines:
  - Robot's kinematic structure
  - Joint definitions
  - TF tree configuration

### 3. Launch Files
Location: `launch/`

#### 3.1 Main Launch File (`spawn_sdf_xacro.launch.py`)
The primary entry point that orchestrates the simulation:
1. Sets up shared resource paths
2. Converts XACRO to URDF for robot state publishing
3. Launches Ignition Gazebo with the specified world
4. Spawns the robot model with proper timing
5. Establishes ROS 2 - Gazebo bridges for:
   - `cmd_vel` (ROS 2 → Gazebo): For velocity control
   - `joint_states` (Gazebo → ROS 2): For robot state visualization

#### 3.2 World Launch File (`world.launch.py`)
- Handles basic world spawning
- Can be used independently for world-only simulation

## Workflow

### Simulation Startup Sequence
1. **World Initialization**
   - Gazebo loads the specified world file
   - Physics engine and rendering systems are initialized

2. **Robot State Publisher**
   - XACRO file is processed into URDF
   - Robot state publisher node is started
   - TF tree is established

3. **Model Spawning**
   - Gazebo loads the SDF model
   - Robot is spawned in the simulation
   - Required plugins are initialized

4. **Bridge Setup**
   - ROS 2 - Gazebo bridges are established
   - Communication channels for control and state are created

### Development Workflow
1. **Model Development**
   - Modify `model.sdf` for physical properties
   - Update `model.xacro` for kinematic properties
   - Test changes in simulation

2. **World Customization**
   - Edit or create new world files in `worlds/`
   - Add or modify environmental elements

3. **Launch File Configuration**
   - Modify launch parameters as needed
   - Add new bridges for additional topics

## Best Practices
1. **Model Development**
   - Keep visual and collision geometries simple for better performance
   - Test inertia properties to ensure realistic physics
   - Use proper naming conventions for links and joints

2. **Simulation**
   - Always include proper timing delays between world loading and model spawning
   - Monitor system resources during simulation
   - Use proper namespacing for multiple robots

3. **Debugging**
   - Check Gazebo console output for warnings/errors
   - Use RViz2 for visualizing TF and sensor data
   - Monitor ROS 2 topics for expected data flow

## Troubleshooting
- **Model Not Appearing**
  - Check Gazebo console for loading errors
  - Verify model paths in launch files
  - Ensure proper SDF formatting

- **Bridges Not Working**
  - Verify topic names match between ROS 2 and Gazebo
  - Check bridge configuration in launch files
  - Ensure proper data types for bridged messages

- **Physics Issues**
  - Verify mass and inertia properties
  - Check for proper collision geometries
  - Adjust simulation step size if needed
