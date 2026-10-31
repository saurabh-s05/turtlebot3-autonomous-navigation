# TurtleBot3 Autonomous Navigation Using ROS 2 and Gazebo

A ROS 2 Jazzy-based autonomous navigation project using the TurtleBot3 Burger robot, Gazebo simulation, and the Nav2 navigation stack.

The project includes a custom occupancy grid map and Python utilities for spawning and experimenting with obstacles in Gazebo.

## Project Overview

- TurtleBot3 Burger robot simulation.
- ROS 2 Jazzy.
- Gazebo Sim.
- ROS 2 Navigation2 (Nav2).
- Custom environment map.
- Static obstacle spawning utility.
- Experimental dynamic obstacle movement utility.

## Software Requirements

| Component | Version |
|---|---|
| Operating System | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Robot | TurtleBot3 Burger |
| Simulator | Gazebo Sim |
| Navigation | Nav2 |

## Repository Structure

```text
turtlebot3-autonomous-navigation/
├── src/
│   ├── DynamixelSDK/
│   ├── turtlebot3/
│   ├── turtlebot3_msgs/
│   └── turtlebot3_simulations/
├── dynamic_obstacle/
│   ├── spawn_obstacles.py
│   └── move_four_obstacles.py
├── turtlebot3_maps/
│   ├── clean_map.yaml
│   └── clean_map.pgm
├── .gitignore
└── README.md
Installation
1. Install ROS 2 Jazzy
Install ROS 2 Jazzy on Ubuntu 24.04 by following the official instructions:

https://docs.ros.org/en/jazzy/Installation.html
2. Install Dependencies
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  python3-colcon-common-extensions \
  python3-rosdep

Initialize rosdep if it has not already been initialized:
sudo rosdep init
rosdep update

3. Clone the Repository
Replace the placeholder with your GitHub repository URL:
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd turtlebot3-autonomous-navigation

4. Install Package Dependencies
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

5. Build the Workspace
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger

Running the Simulation
Open a terminal:
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3-autonomous-navigation/install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

This launches the Gazebo world and spawns the TurtleBot3 Burger robot.
Running Nav2 Navigation
Open another terminal:
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3-autonomous-navigation/install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/turtlebot3-autonomous-navigation/turtlebot3_maps/clean_map.yaml

In RViz:
1. Set the robot's initial pose using 2D Pose Estimate.
2. Set a navigation goal using 2D Goal Pose.
3. Monitor the robot's navigation in the simulated environment.
Obstacle Utilities
The dynamic_obstacle directory contains Python utilities for working with obstacles in Gazebo.
1. Spawn Obstacles
After launching Gazebo, open another terminal:
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3-autonomous-navigation/install/setup.bash

python3 dynamic_obstacle/spawn_obstacles.py

The script creates four box-shaped obstacles in the Gazebo world.
2. Experimental Obstacle Movement
The repository also contains:

dynamic_obstacle/move_four_obstacles.py

This script is an experimental utility intended to move the four spawned obstacles back and forth using Gazebo pose-update services.

Note: Dynamic obstacle movement and its integration with Nav2 have not yet been validated. This functionality is experimental and is not part of the verified navigation workflow.
Custom Gazebo Bridge Configuration
The TurtleBot3 Burger Gazebo bridge configuration is located at:

src/turtlebot3_simulations/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml

The cmd_vel bridge uses the following message types:
ros_topic_name: "cmd_vel"
gz_topic_name: "cmd_vel"
ros_type_name: "geometry_msgs/msg/Twist"
gz_type_name: "gz.msgs.Twist"
direction: ROS_TO_GZ

The ROS message type is configured as geometry_msgs/msg/Twist for velocity commands sent from ROS 2 to Gazebo.
Custom Map
The project includes a custom map:
turtlebot3_maps/
├── clean_map.yaml
└── clean_map.pgm

The YAML file contains the map metadata, including resolution, origin, and occupancy thresholds. The corresponding PGM file contains the occupancy grid image.
Notes
- The simulation and navigation instructions target ROS 2 Jazzy on Ubuntu 24.04.
- Set TURTLEBOT3_MODEL=burger in every terminal that launches TurtleBot3 or Nav2.
- Use use_sim_time:=True when running Nav2 with Gazebo.
- The obstacle movement script is experimental and has not been validated with Nav2.
- The upstream ROBOTIS source packages are retained with their original license files.
Acknowledgements
This project uses packages from the ROBOTIS TurtleBot3 repositories:
- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations
- https://github.com/ROBOTIS-GIT/DynamixelSDK
The upstream packages retain their original license and copyright notices.
