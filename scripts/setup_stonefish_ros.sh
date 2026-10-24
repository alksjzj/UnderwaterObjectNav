#!/usr/bin/env bash
# Source this file in every new container terminal before using Stonefish/ROS:
#   source /workspace/scripts/setup_stonefish_ros.sh

unset ROS_IP
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

_aquaflow_stonefish_lib="${STONEFISH_ROOT:-/opt/stonefish}/lib"
case ":${LD_LIBRARY_PATH:-}:" in
  *":${_aquaflow_stonefish_lib}:"*) ;;
  *) export LD_LIBRARY_PATH="${_aquaflow_stonefish_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac
unset _aquaflow_stonefish_lib

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
else
  echo "AquaFlow setup error: /opt/ros/noetic/setup.bash not found." >&2
  return 1 2>/dev/null || exit 1
fi

_aquaflow_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${_aquaflow_workspace}/devel/setup.bash" ]]; then
  source "${_aquaflow_workspace}/devel/setup.bash"
else
  echo "AquaFlow setup error: ${_aquaflow_workspace}/devel/setup.bash not found; run catkin_make first." >&2
  return 1 2>/dev/null || exit 1
fi

echo "Stonefish/ROS environment ready."
echo "  ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "  ROS_HOSTNAME=${ROS_HOSTNAME}"
echo "  workspace=${_aquaflow_workspace}"
unset _aquaflow_workspace
