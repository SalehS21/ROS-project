# Autonomous Mobile Robot Navigation and Mapping (ROS1)

## Project Overview
This project was developed as my Final Year Project (FYP) using ROS1. It focuses on autonomous mobile robot navigation and mapping, combining theoretical design with real-world implementation on a physical robot. The robot can build maps, localize itself, navigate autonomously from point A to point B, and avoid obstacles in static and dynamic environments. Simulation and visualization were performed using Gazebo and RViz, while navigation was achieved using the ROS Navigation Stack (`move_base`). Full supporting materials, including PowerPoint, videos, source code, and configuration files, are included.

## Environment Requirements
- Ubuntu 20.04 LTS
- ROS1 Noetic
- Gazebo 11
- RViz
- Python 3.x / C++ compiler
- Dependencies listed in the `package.xml` of the project package

## Installation Steps
1. Install ROS1 Noetic following the official guide: [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Install Gazebo and RViz (usually comes with ROS Desktop-Full)
3. Create a catkin workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   cd ~/catkin_ws/src
git clone <your-repo-url>

rosdep install --from-paths src --ignore-src -r -y

project structure
catkin_ws/
├── src/
│   └── my_robot_nav/
│       ├── src/        # C++ nodes
│       ├── scripts/    # Python nodes (e.g., goal sender)
│       ├── launch/     # Launch files (simulation, navigation)
│       ├── config/     # Costmap and planner configuration files
│       ├── maps/       # Generated maps from SLAM
│       └── README.md

