#!/usr/bin/env bash

WS="${WS:-/home/rosmatch/colcon_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"

source /opt/ros/jazzy/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source "${WS}/install/setup.bash"
fi

echo "MUR620A ROS environment ready: WS=${WS}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
