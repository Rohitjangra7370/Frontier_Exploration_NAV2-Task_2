# Frontier Exploration with Nav2

Autonomous frontier exploration using ROS2 and Nav2. The robot identifies unexplored frontier regions on an occupancy map and navigates to them iteratively until the environment is fully mapped.

## Overview

Frontier-based exploration is a classical approach to autonomous map building: the robot continuously selects the nearest unknown boundary (frontier) between explored free space and unexplored space, navigates to it, and repeats until no frontiers remain.

## Stack

- **ROS2 Humble**
- **Nav2** — navigation and path planning
- **SLAM** (Cartographer / slam_toolbox) — real-time map building
- **Gazebo** — simulation environment
- **Python** — frontier detection and goal selection

## How It Works

1. Subscribe to the occupancy grid (`/map`)
2. Detect frontier cells — cells adjacent to unknown space
3. Cluster frontiers and select the closest centroid as goal
4. Send goal to Nav2 action server
5. On arrival, repeat until no frontiers remain

## Usage

```bash
# Build
colcon build
source install/setup.bash

# Launch simulation + exploration
ros2 launch frontier_exploration explore_launch.py
```

## Robotronics Club — IIT Mandi

Part of the Robotronics Club recruitment task series (Task 2).

## Author

Rohit Jangra · [github.com/Rohitjangra7370](https://github.com/Rohitjangra7370)
