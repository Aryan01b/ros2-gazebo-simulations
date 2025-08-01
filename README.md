# ROS 2 Humble with Gazebo Fortress Simulation Workspace

A comprehensive ROS 2 Humble and Gazebo Fortress workspace for learning and experimenting with robotics simulation, SLAM, Nav2, MoveIt, and plugin integration. This workspace includes custom worlds, robot spawning, navigation, and full-stack robotic workflows in simulation.


## 🛠️ Prerequisites

- Ubuntu 22.04 (Recommended)
- ROS 2 Humble
- Gazebo Fortress
- Python 3.8+
- Git

## 🏗️ Installation

1. **Clone the repository:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone git@github.com:Aryan01b/ros2-gazebo-simulations.git
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🚦 Quick Start

1. **Source the workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Launch the simulation:**
   ```bash
   ros2 launch world_01 spawn_sdf_xacro.launch.py
   ```

## 🗂️ Project Structure

```
ros2-gazebo-simulations/
├── config/               # Configuration files
├── launch/               # Launch files
├── maps/                 # Pre-built maps
├── models/               # 3D models and robot descriptions
├── src/                  # Source packages
├── worlds/               # Gazebo world files
└── README.md
```

## 📚 Learning Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Tutorials](https://gazebosim.org/docs/fortress)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)

## 🤝 Contributing

Contributions are welcome! Please read our [Contributing Guidelines](CONTRIBUTING.md) before making a pull request.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS 2 and Gazebo communities
- Open-source robotics community
