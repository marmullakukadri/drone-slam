# Drone SLAM – ROS 2 Jazzy + Gazebo Harmonic + RTAB-Map

Simulated drone with a 3D lidar and IMU in a 32×22 m building (`worlds/home.sdf`).
RTAB-Map builds a 3D map from the lidar, then localizes the drone in it.
Everything runs in the Docker container `drone-slam-dev` (`~/drone-slam` → `/workspace`).

## Setup

```bash
task docker:build   # build image drone-slam:jazzy, create container
task build          # colcon build inside the container
```

## Commands

| Command | What it does |
|---|---|
| `task run:dron` | Localization on `maps/home`. Drone starts at the corridor entrance. |
| `task run:map MAP=name` | New map in `maps/name`. |
| `task run:demo` | Drone flies an automatic mission and builds `maps/demo`. |
| `task teleop` | Keyboard window in the browser. |
| `task takeoff` / `task land` | Take off to 1.5 m / land. |
| `task speed V=1.0` | Change speed while flying (`V` m/s, `VZ` up/down m/s, `YAW` rad/s). |
| `task save` | Save the current map (`map.pcd`, `map.pgm`). |
| `task stop` | Stop the simulation and the container. |
| `task view` | Open Gazebo + RViz (http://localhost:6080/vnc.html). |
| `task status` | Container, simulation and drone state. |
| `task shell` | Shell inside the container. |
| `task test` | C++ unit tests (A* planner, 2D grid). |

`run:*` stays in the foreground. **Ctrl+C** (or `task stop` in another terminal)
shuts down all ROS 2 nodes and then stops the container.
`run:*` does not build: after changing C++ code run `task build` first.

`task run:dron` options: `MAP=home X=-6.5 Y=-0.6 YAW=0.0`.

### Drone speed

- Default: edit `src/drone_control/config/controller.yaml` (`max_speed`, `max_vertical_speed`, `max_yaw_rate`), then `task run:dron`.
- While flying: `task speed V=1.5` or `ros2 param set /drone_node max_speed 1.5`.
- Mission waypoints: `src/drone_control/config/mission.yaml`.

### In RViz

- **2D Goal Pose** – click where the drone should go. It takes off and flies around walls.
- **2D Pose Estimate** – only if "SLAM" (orange) drifts away from "Drone (Gazebo)" (cyan).

### Keyboard (`task teleop`)

| Key | Motion |
|---|---|
| ↑ / ↓ | forward / back |
| ← / → | left / right |
| `u` / `d` | up / down |
| `q` / `e` | turn left / right |
| `+` / `-` | faster / slower |
| space | stop |

## ROS 2 run

```bash
ros2 launch drone_bringup slam_sim.launch.py mode:=localization map_name:=home spawn_x:=-6.5 spawn_y:=-0.6
ros2 launch drone_bringup slam_sim.launch.py mode:=mapping map_name:=map1
ros2 run drone_control drone_teleop.py
ros2 run drone_slam map_saver.py --ros-args -p output_dir:=/workspace/maps/map1
ros2 topic pub --once /drone/command std_msgs/msg/String '{data: shutdown}'
```

Each package can also be launched on its own:

```bash
ros2 launch drone_sim sim.launch.py world:=home.sdf
ros2 launch drone_slam mapping.launch.py database_path:=/workspace/maps/map1/rtabmap.db
ros2 launch drone_localization localization.launch.py database_path:=/workspace/maps/home/rtabmap.db
ros2 launch drone_control control.launch.py
```

| Launch argument | Default | Meaning |
|---|---|---|
| `mode` | `demo` | `demo`, `mapping`, `localization` |
| `map_name` | `demo` / `map` | Folder under `maps/` |
| `spawn_x`, `spawn_y`, `spawn_yaw` | `0` | Start pose in Gazebo (yaw in rad) |
| `gui` | `true` | Gazebo window |
| `rviz` | `true` | RViz2 |
| `auto_initial_pose` | `true` | Localization: send start pose to RTAB-Map automatically |

## Packages and nodes

| Package | Node | Language | Role |
|---|---|---|---|
| `drone_sim` | Gazebo, `create`, `parameter_bridge` | – | World, drone spawn, ROS ↔ Gazebo bridge, sensor TF |
| `drone_slam` | `icp_odometry`, `rtabmap` | RTAB-Map | Lidar odometry and mapping |
| | `slam_evaluator` | C++ | SLAM error vs. Gazebo ground truth, CSV in `results/` |
| | `map_saver.py` | Python | Writes `map.pcd`, `map.pgm/yaml` |
| | `map_filter.py` | Python | Floor-free 3D map and clean 2D map for RViz |
| `drone_localization` | `icp_odometry`, `rtabmap` | RTAB-Map | Localization in a saved map |
| | `initial_pose` | C++ | Sends the start pose to RTAB-Map |
| `drone_control` | `drone_node` | C++ | Flight controller: takeoff/land, A* path to goals, teleop, mission |
| | `drone_teleop.py` | Python | Keyboard window |
| `drone_bringup` | `slam_sim.launch.py` | – | Starts everything for `demo`, `mapping`, `localization` |
| | `drone_model` | C++ | Drone markers in RViz (SLAM and ground truth) |

## Topics

| Topic | Type | Meaning |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | 3D lidar |
| `/imu` | `sensor_msgs/Imu` | IMU |
| `/odom_gt` | `nav_msgs/Odometry` | Ground truth from Gazebo (not used by SLAM) |
| `/odom` | `nav_msgs/Odometry` | Lidar odometry |
| `/cloud_map` | `sensor_msgs/PointCloud2` | RTAB-Map 3D map |
| `/cloud_map_filtered` | `sensor_msgs/PointCloud2` | 3D map without floor |
| `/map_filtered` | `nav_msgs/OccupancyGrid` | Clean 2D map |
| `/planning_map` | `nav_msgs/OccupancyGrid` | 2D grid used by A* |
| `/planned_path` | `nav_msgs/Path` | Route to the goal |
| `/gt_path` | `nav_msgs/Path` | Flown path |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal (RViz 2D Goal Pose) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Start pose for localization |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Keyboard command (priority) |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity to Gazebo |
| `/drone/command` | `std_msgs/String` | `takeoff`, `land`, `hover`, `mission`, `shutdown` |
| `/mission/state` | `std_msgs/String` | `IDLE`, `TAKEOFF`, `GOTO`, `MISSION`, `LANDING` |
| `/slam/position_error` | `std_msgs/Float64` | SLAM error in m |
| `/drone_model`, `/drone_model_gt` | `visualization_msgs/MarkerArray` | Drone in RViz |

TF: `map → odom → base_link → {lidar_link, imu_link}`, `map → world`.

## Saved map (`maps/<name>/`)

| File | Content |
|---|---|
| `map.pcd` / `map_full.pcd` | 3D map without / with floor (ASCII PCD) |
| `map.pgm` + `map.yaml` | Filtered 2D map (Nav2 format) |
| `map_full.pgm` + `map_full.yaml` | Unfiltered 2D map |
| `rtabmap.db` | RTAB-Map database, used for localization |
| `map_info.yaml` | World and map origin |

## Layout

```
worlds/home.sdf           building
models/slam_drone/        drone (lidar, IMU, velocity control)
maps/                     saved maps
results/                  SLAM error CSV
src/
  drone_sim/              launch/sim.launch.py, config/gz_bridge.yaml
  drone_slam/             launch/mapping.launch.py, config/rtabmap.yaml, slam_evaluator, map_saver, map_filter
  drone_localization/     launch/localization.launch.py, initial_pose
  drone_control/          launch/control.launch.py, config/controller.yaml + mission.yaml, drone_node, drone_teleop, test/
  drone_bringup/          launch/slam_sim.launch.py, rviz/slam.rviz, drone_model, start_vnc.sh, sync_check.sh
taskfile.yml              tasks
Dockerfile                image
```

C++ packages follow the same layout:

```
include/<package>/<node>.hpp   class declaration
src/<node>.cpp                 implementation
src/main.cpp                   rclcpp::init, spin, shutdown
```
