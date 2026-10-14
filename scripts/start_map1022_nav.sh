#!/usr/bin/env bash
set -Eeuo pipefail

workspace="${LITE3_WS:-/root/nav/livox_ws}"
export TMUX_TMPDIR="${TMUX_TMPDIR:-/tmp/lite3_tmux_nav}"
mkdir -p "$TMUX_TMPDIR"
chmod 700 "$TMUX_TMPDIR"

session="${NAV_TMUX_SESSION:-lite3_nav3d}"
case "${1:-start}" in
  stop)
    if tmux has-session -t "$session" 2>/dev/null; then
      tmux kill-session -t "$session"
      echo "Stopped navigation tmux session: $session"
    else
      echo "Navigation tmux session is not running: $session"
    fi
    exit 0
    ;;
  attach)
    if ! tmux has-session -t "$session" 2>/dev/null; then
      echo "Navigation tmux session is not running: $session" >&2
      echo "Start it with: $0" >&2
      exit 1
    fi
    exec tmux attach-session -t "$session"
    ;;
  status)
    if tmux has-session -t "$session" 2>/dev/null; then
      echo "RUNNING: $session"
      tmux list-panes -t "$session:ALL-NODES" -F '#{pane_index} #{pane_title} #{pane_current_command}'
    else
      echo "STOPPED: $session"
      exit 1
    fi
    exit 0
    ;;
  start)
    map_path="$workspace/maps/map1022/map_nav_candidate.pcd"
    ;;
  *)
    map_path="$1"
    ;;
esac

# Validated map1022 planning parameters (2026-09-29).  Keep these together:
# the 0.1 m grid needs a deeper ground-support search than the 0.2 m default.
export NAV3D_MIN_POINTS_PER_VOXEL=1
export NAV3D_RESOLUTION=0.1
export NAV3D_GROUND_ROBOT_RADIUS=0.20
export NAV3D_GROUND_SUPPORT_DEPTH_CELLS=3
export NAV3D_OCCUPANCY_GRID_MIN_Z=-0.2
export NAV3D_OCCUPANCY_GRID_MAX_Z=0.5
# Only auto-load zones belonging to the selected map. Explicit overrides must
# remain strict: silently dropping a requested keepout file is unsafe.
if [[ ! -v NAV3D_ZONE_FILE ]]; then
  zone_candidate="$(dirname -- "$map_path")/navigation_zones.yaml"
  NAV3D_ZONE_FILE=''
  if [[ -f "$zone_candidate" ]]; then
    NAV3D_ZONE_FILE="$zone_candidate"
  fi
fi
export NAV3D_ZONE_FILE

# Safe unless the operator explicitly opts in for a supervised motion test.
export NAV_MOTION_ENABLED="${NAV_MOTION_ENABLED:-false}"
export NAV3D_RVIZ="${NAV3D_RVIZ:-false}"
export NAV_TMUX_DETACH="${NAV_TMUX_DETACH:-false}"

exec "$workspace/scripts/start_nav3d_scan_tmux.sh" "$map_path"
