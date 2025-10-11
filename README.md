# ROS 2 Humble with Gazebo Fortress Simulation Workspace

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-0096D6?logo=gazebosim)](https://gazebosim.org/docs/fortress/)

A ROS 2 Humble and Gazebo Fortress workspace for robotics simulation.

## 🛠️ Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- Python 3.8+

## 🏗️ Installation

1. **Clone the repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Aryan01b/ros2-gazebo-simulations.git
   ```

2. **Install dependencies and build**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
  <p>Contributions welcome!</p>
  <p>
    <a href="https://github.com/Aryan01b/ros2-gazebo-simulations/graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Aryan01b/ros2-gazebo-simulations" alt="Contributors" />
    </a>
  </p>
</div>
