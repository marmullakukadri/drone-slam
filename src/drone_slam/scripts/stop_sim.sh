#!/usr/bin/env bash
# Stop every simulation/SLAM process inside the container (leaves VNC running).
# Never touches this script's own shell or any of its ancestors.
patterns=(
  "ros2 launch drone_slam"
  "gz sim"
  "parameter_bridge"
  "rtabmap_slam/rtabmap"
  "icp_odometry"
  "static_transform_publisher"
  "drone_slam/drone_node"
  "slam_evaluator.py"
  "rviz2"
  "drone_teleop.py"
  "map_filter.py"
  "drone_model.py"
  "initial_pose.py"
)

# PIDs of this script and its ancestors
protected=" $$ "
pid=$$
while [[ "$pid" -gt 1 ]]; do
  pid=$(ps -o ppid= -p "$pid" | tr -d ' ')
  [[ -z "$pid" ]] && break
  protected+="$pid "
done

targets() {
  for p in "${patterns[@]}"; do pgrep -f "$p"; done | sort -u | while read -r t; do
    [[ "$protected" == *" $t "* ]] || echo "$t"
  done
}

wait_gone() {  # wait up to $1 seconds for all targets to exit
  for _ in $(seq 1 $(( $1 * 2 ))); do
    [[ -z "$(targets)" ]] && return 0
    sleep 0.5
  done
  return 1
}

pids=$(targets)
[[ -z "$pids" ]] && { echo "Nothing running."; exit 0; }
# SIGINT first: lets RTAB-Map close and save its database cleanly.
kill -INT $pids 2>/dev/null
if ! wait_gone 20; then
  kill -TERM $(targets) 2>/dev/null
  wait_gone 5 || kill -KILL $(targets) 2>/dev/null
fi
echo "Simulation stopped."
