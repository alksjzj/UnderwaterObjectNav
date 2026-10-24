#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

export STONEFISH_ROOT="${STONEFISH_ROOT:-/opt/stonefish}"
export CMAKE_PREFIX_PATH="${STONEFISH_ROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${STONEFISH_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PKG_CONFIG_PATH="${STONEFISH_ROOT}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

if [[ -f /workspace/devel/setup.bash ]]; then
  source /workspace/devel/setup.bash
fi

exec "$@"
