# ROS 2 Humble with Gazebo Fortress Simulation Workspace

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-0096D6?logo=gazebosim)](https://gazebosim.org/)

A ROS 2 Humble and Gazebo Fortress workspace for robotics simulation.

## 📋 Table of Contents
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Project Structure](#-project-structure)

## 🛠️ Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- Python 3.8+

## 🏗️ Installation

1. **Create a workspace and clone the repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Aryan01b/ros2-gazebo-simulations.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🚀 Quick Start

1. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Launch the simulation**:
   ```bash
   ros2 launch world_01 spawn_sdf_xacro.launch.py
   ```

## 🗂️ Project Structure

```
ros2-gazebo-simulations/
├── config/               # Configuration files
├── launch/               # Launch files
├── models/               # 3D models and robot descriptions
├── src/                  # Source packages
├── worlds/               # Gazebo world files
└── README.md
```

## 📚 Documentation

For more information, please refer to the [ROS 2 Documentation](https://docs.ros.org/).

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
  <p>Made with ❤️ by the ROS 2 Community</p>
  <p>
    <a href="https://github.com/Aryan01b/ros2-gazebo-simulations/graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Aryan01b/ros2-gazebo-simulations" alt="Contributors" />
    </a>
  </p>
</div>
