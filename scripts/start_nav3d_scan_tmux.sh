#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${1:-}" == "__run" ]]; then
  workspace="$2"
  delay="$3"
  label="$4"
  shift 4
  set +u
  source "$workspace/scripts/ros_env.bash"
  set -u
  printf '\n===== %s (starts after %ss) =====\n' "$label" "$delay"
  sleep "$delay"
  printf 'Starting: '
  printf '%q ' "$@"
  printf '\n\n'
  exec "$@"
fi

workspace="${LITE3_WS:-/root/nav/livox_ws}"
pcd_path="${1:-$workspace/maps/map9802/map_edited.pcd}"
session="${NAV_TMUX_SESSION:-lite3_nav3d}"
nav3d_rviz="${NAV3D_RVIZ:-false}"
nav3d_rosbridge="${NAV3D_ROSBRIDGE:-false}"
nav_tmux_detach="${NAV_TMUX_DETACH:-false}"
nav_motion_enabled="${NAV_MOTION_ENABLED:-false}"
script_path="$(readlink -f "$0")"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Run: sudo apt install tmux" >&2
  exit 1
fi
if [[ ! -f "$workspace/scripts/ros_env.bash" ]]; then
  echo "Missing environment script: $workspace/scripts/ros_env.bash" >&2
  exit 1
fi
if [[ ! -f "$pcd_path" ]]; then
  echo "Missing PCD map: $pcd_path" >&2
  exit 1
fi
if [[ ! -f "$workspace/src/pose_bridge.py" ]]; then
  echo "Missing pose bridge: $workspace/src/pose_bridge.py" >&2
  exit 1
fi

set +u
source "$workspace/scripts/ros_env.bash"
set -u
required_packages=(
  livox_ros_driver2
  fastlio2
  localizer
  topic_bridge
  sensor_extrinsic
  nav3d_ros2_bridge
  scan_planner
  dog_control_bridge
)
missing_packages=()
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    missing_packages+=("$package")
  fi
done
if ((${#missing_packages[@]})); then
  printf 'Packages not found:' >&2
  printf ' %s' "${missing_packages[@]}" >&2
  printf '\nBuild and source the workspace before starting.\n' >&2
  exit 1
fi

if ! ros2 pkg executables topic_bridge | grep -Fq 'topic_bridge nav_data_bridge'; then
  echo "nav_data_bridge is not installed; rebuild topic_bridge before starting." >&2
  exit 1
fi
if ros2 node list 2>/dev/null | grep -Fxq '/all_bridges'; then
  echo "Old /all_bridges is still running. Stop the old navigation session first." >&2
  exit 1
fi

if tmux has-session -t "$session" 2>/dev/null; then
  echo "tmux session '$session' already exists; attaching without restarting nodes."
  echo "To load rebuilt nodes, stop this session and run the script again."
  exec tmux attach-session -t "$session"
fi

# PGO and localizer both publish map -> odom. Never start navigation with PGO active.
if ros2 node list 2>/dev/null | grep -Fxq '/pgo/pgo_node'; then
  echo "PGO is running and also publishes map -> odom. Stop PGO before starting navigation." >&2
  exit 1
fi
if ros2 node list 2>/dev/null | grep -Fxq '/fastlio2/lio_node'; then
  echo "FAST-LIO2 is already running. Stop the old LIO session before starting another." >&2
  exit 1
fi

queue_command() {
  local target="$1"
  local delay="$2"
  local label="$3"
  shift 3
  local command_line
  printf -v command_line '%q ' \
    "$script_path" __run "$workspace" "$delay" "$label" "$@"
  tmux send-keys -t "$target" "$command_line" C-m
}

tmux new-session -d -s "$session" -n "ALL-NODES"
tmux set-option -t "$session" mouse on
tmux set-option -t "$session" status-interval 2
tmux set-option -t "$session" status-left "#[bold,fg=green] Lite3导航 #[default]"
tmux set-option -t "$session" status-right "#[fg=cyan]九宫格总览 #[default] %H:%M"
# Direct detach key for users/terminals where the Ctrl-b prefix is inconvenient.
tmux bind-key -n F12 detach-client
tmux set-window-option -t "$session:ALL-NODES" pane-border-status top
tmux set-window-option -t "$session:ALL-NODES" pane-border-format \
  "#[bold,fg=yellow] #{pane_title} #[default]"

# One screen, nine panes. Re-tile after every split to keep a 3x3 dashboard.
for _ in {1..8}; do
  tmux split-window -d -t "$session:ALL-NODES"
  tmux select-layout -t "$session:ALL-NODES" tiled
done

tmux select-pane -t "$session:ALL-NODES.0" -T "1/9  MID360雷达"
tmux select-pane -t "$session:ALL-NODES.1" -T "2/9  FAST-LIO2+TF"
tmux select-pane -t "$session:ALL-NODES.2" -T "3/9  Nav3D位姿桥"
tmux select-pane -t "$session:ALL-NODES.3" -T "4/9  SCAN外参"
tmux select-pane -t "$session:ALL-NODES.4" -T "5/9  路径点云位姿桥"
tmux select-pane -t "$session:ALL-NODES.5" -T "6/9  Nav3D全局规划"
tmux select-pane -t "$session:ALL-NODES.6" -T "7/9  SCAN局部规划"
tmux select-pane -t "$session:ALL-NODES.7" -T "8/9  机器狗底盘"
tmux select-pane -t "$session:ALL-NODES.8" -T "9/9  Localizer最后启动"

queue_command "$session:ALL-NODES.0" 0 "Livox MID360" \
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
queue_command "$session:ALL-NODES.1" 2 "FAST-LIO2 + Lite3 TF" \
  ros2 launch fastlio2 lio_launch.py rviz:=false
queue_command "$session:ALL-NODES.2" 25 "Nav3D map pose bridge" \
  python3 "$workspace/src/pose_bridge.py"
queue_command "$session:ALL-NODES.3" 26 "SCAN sensor pose" \
  ros2 run sensor_extrinsic lidar_extrinsic_publisher
queue_command "$session:ALL-NODES.4" 27 "Path/cloud/pose data bridge" \
  ros2 run topic_bridge nav_data_bridge
queue_command "$session:ALL-NODES.5" 30 "Nav3D global planner" \
  ros2 launch nav3d_ros2_bridge nav3d_bridge.launch.py \
  pcd_path:="$pcd_path" frame_id:=map planning_mode:=3d planning_traversability:=ground \
  rviz:="$nav3d_rviz" rosbridge:="$nav3d_rosbridge"
if [[ "$nav_motion_enabled" == "true" ]]; then
  queue_command "$session:ALL-NODES.6" 32 "SCAN local planner" \
    ros2 launch scan_planner run.launch.py \
    is_real_world:=true navi_mode:=3 sensor_type:=lidar \
    controller_mode:=closed_loop use_sim_time:=false
  queue_command "$session:ALL-NODES.7" 34 "Lite3 motion bridge" \
    ros2 run dog_control_bridge dog_control_bridge
else
  tmux send-keys -t "$session:ALL-NODES.6" \
    "echo '[SAFE] SCAN运动控制未启动；实机测试时设置 NAV_MOTION_ENABLED=true'" C-m
  tmux send-keys -t "$session:ALL-NODES.7" \
    "echo '[SAFE] 机器狗底盘桥未启动；实机测试时设置 NAV_MOTION_ENABLED=true'" C-m
fi
queue_command "$session:ALL-NODES.8" 36 "Localizer (last node)" \
  ros2 run localizer localizer_node --ros-args \
  -p config_path:="$workspace/install/localizer/share/localizer/config/localizer.yaml" \
  -p map_path:="$pcd_path"

tmux select-layout -t "$session:ALL-NODES" tiled
tmux select-pane -t "$session:ALL-NODES.0"
echo "Created tmux session: $session"
echo "Map: $pcd_path"
echo "Nav3D RViz: $nav3d_rviz (set NAV3D_RVIZ=true to enable)"
echo "Motion control: $nav_motion_enabled (set NAV_MOTION_ENABLED=true only for supervised tests)"
echo "localizer_node starts last, after 36 seconds."
echo "Open another terminal for relocalization and navigation commands."
if [[ "$nav_tmux_detach" == "true" ]]; then
  echo "tmux session is running in the background (NAV_TMUX_DETACH=true)."
  exit 0
fi
exec tmux attach-session -t "$session"
