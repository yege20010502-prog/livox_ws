#!/usr/bin/env bash
# Record a supervised navigation test; never starts navigation or motion.
set -Eeuo pipefail
workspace="${LITE3_WS:-/root/nav/livox_ws}"
set +u
source "$workspace/scripts/ros_env.bash"
set -u
label="${1:-nav_test}"
if [[ ! "$label" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Test label must contain only letters, digits, underscore or hyphen.' >&2
  exit 1
fi
run_dir="$workspace/test_records/$(date +%Y%m%d_%H%M%S)_${label}_$$"
mkdir -p "$run_dir"
date -Is > "$run_dir/start_time.txt"
git -C "$workspace" rev-parse HEAD > "$run_dir/git_commit.txt"
git -C "$workspace" diff -- src scripts README.md > "$run_dir/source_changes.patch"
ros2 topic list -t > "$run_dir/topics.txt"
ros2 node list > "$run_dir/nodes.txt"
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY > "$run_dir/ros_environment.txt" || true
for node in /nav3d_bridge_node /scan_planner_node /closed_loop_controller /localizer_node /dog_control_bridge_node; do
  ros2 param dump "$node" > "$run_dir/${node#/}_parameters.yaml" 2>&1 || true
done
echo "Recording directory: $run_dir"
echo 'Start the supervised test in another terminal. Ctrl-C here stops recording only.'
echo 'This does NOT stop the robot: use the remote/emergency stop before ending the test.'
# Default to lightweight diagnostics: full-rate clouds can burden the board.
if [[ "${2:-lean}" == full ]]; then
  exec ros2 bag record -a -o "$run_dir/bag"
fi
exec ros2 bag record -o "$run_dir/bag" \
  /tf /tf_static /rosout /parameter_events /cmd_vel /nav3d/status /nav3d/goal \
  /nav3d/trajectory /initial_path /planning/bspline \
  /planning/go2_execution_frozen /fastlio2/lio_odom \
  /LIO/odom_vehicle /LIO/odom_imu /livox/imu \
  /localizer/healthy
