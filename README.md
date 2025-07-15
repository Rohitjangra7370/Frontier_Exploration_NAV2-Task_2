# 🚀 task_2: Multi-Robot Exploration and Mapping in ROS2
Working:https://drive.google.com/drive/folders/1AE7zTh7yydI7f0yBz0ihsfpwbSRC41ob?usp=sharing

Welcome to **task_2**, a high-tech ROS2 powerhouse for autonomous multi-robot systems. Dive into frontier exploration, seamless map merging, and precision navigation. Built for simulations in Gazebo and real-world deployments with TurtleBot3, this repo is your gateway to futuristic robotics projects.

## 🔍 Project Overview

This framework orchestrates multiple robots to conquer unknown terrains, fusing their maps into a single, intelligent global view. It's optimized for ROS2 (from Eloquent to Humble), blending Nav2's SLAM prowess with custom algorithms for a *tech-forward* experience.

**Core Tech Stack:**
- **ROS2 & Gazebo**: For immersive simulations.
- **Nav2 & SLAM Toolbox**: Advanced navigation and mapping.
- **OpenCV**: Feature matching for dynamic map merging.

Key highlights:
- Multi-robot synergy for exploration and mapping.
- Customizable params for peak performance.
- Ready for both virtual sims and hardware like JetBot.

## ⚙️ Features

- **Frontier Exploration Engine**: Lightweight, ROS1-ported algorithm that auto-detects frontiers and dispatches robots. Control via topics for pause/resume.  
  *Tech Note*: Configurable with `planner_frequency` and `min_frontier_size`.

- **Dynamic Map Merging**: Fuse maps on-the-fly. Supports known poses (param-driven) or unknown (via AKAZE/ORB/SURF detectors).  
  *Pro Tip*: CPU-intensive? Tune `estimation_rate` for efficiency.

- **Simulation Suite**: Fire up Gazebo with TurtleBot3 models (e.g., waffle). Custom worlds and multi-bot launches included.

- **Navigation Core**: Nav2 integration for SLAM, pathfinding, and obstacle avoidance.

- **Testing Framework**: Robust unit tests for exploration and merging, auto-pulling test data.

- **Hardware Integration**: Plug-and-play for real bots with Realsense cams – map your sensor topics and go!

## 🛠️ Dependencies

Gear up with these essentials:
- ROS2 Humble (or Eloquent+).
- Gazebo for sim magic.
- TurtleBot3 packages: `turtlebot3`, `turtlebot3_msgs`.
- OpenCV 4.x (for max feature power).
- Nav2 and SLAM Toolbox (experimental branch for extras).

Quick install on Ubuntu:
```
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
```

## 📦 Installation Guide

1. **Clone the Repo**:
   ```
   git clone https://github.com/rohitjangra7370/task_2.git
   cd task_2
   ```

2. **Install Deps**:
   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build & Source**:
   ```
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🚀 Quick Start Usage

### Single-Robot Sim
Launch exploration in Gazebo:
```
ros2 launch m-explore-ros2 tb3_simulation_launch.py
```
*Expected Output*: Robot spawns, maps, and explores autonomously.

### Multi-Robot Merging
For dual bots with known poses:
```
ros2 launch m-explore-ros2 multi_tb3_simulation_launch.py
```
Customize poses in launch files (e.g., `init_pose_x/y`).

For unknown poses: Enable feature matching in configs – let the algorithms align!

### Real-World Deployment
Tweak launches for your hardware. Example topic remap:
```
/camera/depth/image_rect_raw -> /depth_camera
```

### Config Tweaks
Hack `params.yaml` in `m-explore-ros2/explore`:
```yaml
planner_frequency: 1.0
min_frontier_size: 0.75
return_to_init: true
```

## ⚠️ Troubleshooting & Optimization

- **Performance Hiccups**: Unknown pose merging? Dial down `estimation_rate` to ease CPU load.
- **Compat Issues**: Nav2 params evolve – check ROS version diffs.
- **Setup Snags**: Missing poses trigger fallback to feature matching; verify OpenCV install.

*Tech Hack*: Monitor with `rqt` for real-time topic insights.

## 🤝 Contributing

Join the tech revolution! Fork, branch, code, and PR. Prioritize docs, bug fixes, or new tests. Let's build the future of robotics together.

## 📜 License

BSD License – open and flexible. Check files for specifics (standardization in progress).
