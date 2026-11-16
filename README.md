# Drone SLAM

A simulated drone flies inside a building. With its 3D lidar it builds a 3D map
of the building (SLAM), saves it, and later uses that map to know where it is
(localization). You click a goal in RViz and the drone flies there around the walls.

Runs in Docker (ROS 2 Jazzy, Gazebo, RTAB-Map). Gazebo + RViz open in the browser:
http://localhost:6080/vnc.html

## First time

```bash
task docker:build   # build the Docker image and create the container
task build          # compile the ROS 2 code
```

## Fly in the saved map

```bash
task run:dron
```

In RViz: **2D Goal Pose** → click where the drone should go.

## Make a new map

```bash
task run:map MAP=map1
task teleop
task save
```

## Commands

| Command | What it does |
|---|---|
| `task run:dron` | Fly in the saved map (`maps/home`) |
| `task run:map MAP=name` | Make a new map |
| `task run:demo` | Automatic flight, builds `maps/demo` |
| `task teleop` | Keyboard control |
| `task takeoff` | Take off |
| `task land` | Land |
| `task speed V=1.0` | Change speed while flying |
| `task save` | Save the map |
| `task stop` | Stop everything |
| `task view` | Open Gazebo + RViz |
| `task status` | Show state |
| `task build` | Build after changing code |
| `task test` | Run tests |
| `task shell` | Shell in the container |

**Ctrl+C** or `task stop` stops the simulation.

## Keyboard

| Key | Motion |
|---|---|
| ↑ ↓ | forward / back |
| ← → | left / right |
| `u` `d` | up / down |
| `q` `e` | turn |
| `+` `-` | faster / slower |
| space | stop |
