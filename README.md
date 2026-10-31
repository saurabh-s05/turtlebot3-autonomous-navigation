# TurtleBot3 Autonomous Navigation Using ROS 2 and Gazebo

Autonomous navigation with a TurtleBot3 Burger in Gazebo, built on ROS 2 Jazzy and the Nav2
navigation stack.

The project has two parts:

1. **Navigation without obstacles** - the robot navigates the standard TurtleBot3 world using a
   map I built of the environment.
2. **Navigation with obstacles** - the same map, but the world now has extra obstacles that are
   not in the map. Some are static and some keep moving. The robot has to detect them with the
   lidar and plan around them. This part lives in the `tb3_dynamic_nav` package.

## Software Requirements

| Component | Version |
|---|---|
| Operating System | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Robot | TurtleBot3 Burger |
| Simulator | Gazebo Sim (Harmonic) |
| Navigation | Nav2 |

## Repository Structure

```text
turtlebot3-autonomous-navigation/
├── src/
│   ├── DynamixelSDK/
│   ├── turtlebot3/
│   ├── turtlebot3_msgs/
│   ├── turtlebot3_simulations/
│   └── tb3_dynamic_nav/          # part 2: navigation with obstacles
│       ├── config/nav2_dynamic.yaml
│       ├── launch/
│       │   ├── dynamic_nav.launch.py
│       │   └── nav2.launch.py
│       ├── maps/
│       ├── rviz/
│       ├── worlds/dynamic_world.sdf
│       └── tb3_dynamic_nav/
│           ├── dynamic_obstacles.py
│           ├── nav_watchdog.py
│           └── patrol.py
├── turtlebot3_maps/
│   ├── clean_map.yaml
│   └── clean_map.pgm
├── .gitignore
└── README.md
```

## Installation

### 1. Install ROS 2 Jazzy

Follow the official instructions for Ubuntu 24.04:
https://docs.ros.org/en/jazzy/Installation.html

### 2. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions \
  python3-rosdep
```

If rosdep has not been set up yet:

```bash
sudo rosdep init
rosdep update
```

### 3. Clone the repository

```bash
cd ~
git clone https://github.com/saurabh-s05/turtlebot3-autonomous-navigation.git
cd turtlebot3-autonomous-navigation
```

### 4. Install package dependencies

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

### 5. Build

```bash
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Every new terminal needs:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3-autonomous-navigation/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Part 1: Navigation Without Obstacles

This uses the stock TurtleBot3 world and Nav2 setup with my map.

### Start the simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This opens Gazebo and spawns the TurtleBot3 Burger.

### Start Nav2

In another terminal:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/turtlebot3-autonomous-navigation/turtlebot3_maps/clean_map.yaml
```

In RViz:

1. Set the robot's initial pose with **2D Pose Estimate**.
2. Send a goal with **Nav2 Goal**.
3. Watch the robot plan a path and drive to the goal.

### First obstacle scripts

Before writing part 2, I tried adding obstacles with two scripts in `dynamic_obstacle/`. With
Gazebo running:

```bash
python3 dynamic_obstacle/spawn_obstacles.py      # adds four boxes
python3 dynamic_obstacle/move_four_obstacles.py  # moves them back and forth
```

With the stock Nav2 settings the robot often got stuck near these obstacles, and the boxes jump
instead of moving smoothly because every pose update starts a new `gz service` process. Part 2
is the version that works.

---

## Part 2: Navigation With Static and Moving Obstacles

Everything starts with one launch file:

```bash
ros2 launch tb3_dynamic_nav dynamic_nav.launch.py
```

This starts Gazebo with `dynamic_world.sdf`, spawns the robot, and brings up Nav2, RViz and the
node that moves the obstacles. The world is the normal TurtleBot3 world plus:

- 4 static obstacles (orange) placed in open areas
- 4 moving obstacles (red) that go back and forth through the corridors between the pillars at
  0.08-0.12 m/s

None of these are in the map, so the robot only knows about them from the lidar.

The initial pose is set automatically, so there is no need for 2D Pose Estimate. Wait until the
terminal shows `Managed nodes are active` twice, then click **Nav2 Goal** in RViz and pick a
spot on the map. Drag while clicking to choose the final heading. You can send a new goal at
any time.

### Patrol mode

To let the robot drive a fixed route around the arena by itself:

```bash
ros2 launch tb3_dynamic_nav dynamic_nav.launch.py patrol:=true
```

It goes through 8 waypoints in a loop. If a waypoint fails, for example because a moving
obstacle is blocking the corridor, it waits and tries again, and skips it after two retries.
The waypoints are in `src/tb3_dynamic_nav/tb3_dynamic_nav/patrol.py`.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `patrol` | `false` | Drive the waypoint route instead of waiting for RViz goals |
| `move_obstacles` | `true` | Move the red obstacles |
| `obstacle_speed` | `1.0` | Speed multiplier for the moving obstacles, e.g. `2.0` for a harder test |
| `gui` | `true` | Open the Gazebo window |
| `rviz` | `true` | Open RViz |
| `map` | `maps/clean_map.yaml` | Map file |
| `params_file` | `config/nav2_dynamic.yaml` | Nav2 parameters |

### What I changed compared to part 1

The default Nav2 settings worked fine in the empty world but the robot kept getting stuck once
obstacles were added. These are the changes that fixed it:

- **Controller:** replaced DWB with the MPPI controller. DWB barely weighted obstacles in the
  TurtleBot3 config, so the robot drove right up to them and could not get out. MPPI looks about
  2.8 s ahead and steers around obstacles, including ones moving into its path.
- **Robot size and inflation:** the Nav2 default params assume a much bigger robot (0.22 m radius,
  0.7 m inflation), which closes the gaps between the pillars. I set the real Burger radius
  (0.105 m) and an inflation radius of 0.40 m.
- **Costmap clearing:** moving obstacles left trails of old obstacle cells behind them. I removed
  the duplicate voxel layer and turned on `inf_is_valid`, so laser beams that hit nothing also
  clear the cells they pass through.
- **Velocity commands:** Nav2 was publishing `TwistStamped` but the Gazebo bridge expects `Twist`,
  so the robot did not move at all. `enable_stamped_cmd_vel` is now `false`.
- **Collision monitor:** the stop zone was smaller than the robot. Now it stops when something is
  within 0.15 m and slows to half speed when something is just ahead. The scan timeout is 1 s
  because the lidar only runs at 5 Hz.
- **Progress checker:** the robot gets 20 s to make progress instead of 10 s, so it can wait for an
  obstacle to pass instead of aborting.
- **Smooth obstacle motion:** `dynamic_obstacles.py` talks to Gazebo directly through gz-transport
  and updates all four obstacles in one request, 20 times a second.

### Keeping Nav2 running

Sometimes the Nav2 lifecycle nodes got deactivated during a run and navigation stopped. To
handle that:

- Each Nav2 node runs as its own process and restarts by itself if it crashes
  (`launch/nav2.launch.py`).
- The lifecycle managers wait 10 s for a heartbeat instead of 4 s, and keep trying to reconnect
  to a restarted node for up to 2 minutes.
- `nav_watchdog.py` checks every 5 s that both lifecycle managers are active. If one stays down for
  more than 15 s, it resets the nodes and starts them again.

I tested this by killing `controller_server` in the middle of a run and by forcing a reset of all
navigation nodes. In both cases the nodes came back and navigation continued.

### Tuning

All Nav2 settings are in `src/tb3_dynamic_nav/config/nav2_dynamic.yaml`.

- Robot goes too close to obstacles: increase `CostCritic.cost_weight` or `inflation_radius`.
- Robot will not go through narrow gaps: decrease `inflation_radius`.
- Slow computer, controller warnings: reduce `batch_size` from 1000 to 600.
- Robot slows down too much near walls: reduce the `PolygonSlow` box in `collision_monitor`.

---

## Custom Gazebo Bridge Configuration

The TurtleBot3 Burger bridge config is at:

```text
src/turtlebot3_simulations/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml
```

The `cmd_vel` bridge uses plain `Twist` messages:

```yaml
ros_topic_name: "cmd_vel"
gz_topic_name: "cmd_vel"
ros_type_name: "geometry_msgs/msg/Twist"
gz_type_name: "gz.msgs.Twist"
direction: ROS_TO_GZ
```

Both parts rely on this. If you use a fresh `turtlebot3_simulations` checkout, check that the type
is still `Twist`.

## Custom Map

```text
turtlebot3_maps/
├── clean_map.yaml
└── clean_map.pgm
```

The YAML file has the map metadata (resolution, origin and occupancy thresholds) and the PGM file
is the occupancy grid image. Part 2 uses a copy of the same map in `src/tb3_dynamic_nav/maps/`.

The map was recorded with the robot starting at (-2.0, -0.5) in Gazebo, so map coordinates are
the Gazebo world coordinates shifted by (+2.0, +0.5). The robot has to be spawned at that
position for the map to line up.

## Troubleshooting

**Gazebo shows the wrong world or no obstacles.** A Gazebo server from an earlier run is still
running. `gz sim` sometimes keeps running after the launch is stopped. Close everything and run:

```bash
pkill -f "gz sim"
pgrep -af "gz sim"    # should print nothing, otherwise: pkill -9 -f "gz sim"
```

The part 2 launch file uses its own Gazebo partition every time (printed at startup as
`GZ_PARTITION=...`), so old servers should not interfere with it anymore. To use `gz` commands
with it, export that value first, e.g. `export GZ_PARTITION=tb3_dynamic_nav_12345`.

**Robot does not move.** Check that the bridge uses `Twist` (see above) and that
`TURTLEBOT3_MODEL=burger` is set.

**Map shows up but the robot is in the wrong place.** The robot has to spawn at (-2.0, -0.5). The
part 2 launch file does this automatically.

## Notes

- Everything here targets ROS 2 Jazzy on Ubuntu 24.04.
- Set `TURTLEBOT3_MODEL=burger` in every terminal.
- Use `use_sim_time:=True` when running Nav2 with Gazebo (the part 2 launch sets it already).
- The upstream ROBOTIS packages keep their original license files.

## Acknowledgements

This project uses packages from the ROBOTIS TurtleBot3 repositories:

- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations
- https://github.com/ROBOTIS-GIT/DynamixelSDK

The upstream packages keep their original license and copyright notices.
