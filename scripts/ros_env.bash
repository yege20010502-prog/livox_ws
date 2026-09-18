#!/usr/bin/env bash
# Load ROS 2 plus every package from this isolated colcon workspace.
source /opt/ros/humble/setup.bash
LITE3_WS="${LITE3_WS:-/root/nav/livox_ws}"
if [ -f "$LITE3_WS/install/setup.bash" ]; then
  source "$LITE3_WS/install/setup.bash"
fi
for prefix in "$LITE3_WS"/install/*; do
  [ -d "$prefix/share/ament_index/resource_index/packages" ] || continue
  case ":${AMENT_PREFIX_PATH:-}:" in
    *":$prefix:"*) ;;
    *) export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
  for package_setup in "$prefix"/share/*/package.bash; do
    [ -f "$package_setup" ] && source "$package_setup"
  done
done
unset prefix package_setup