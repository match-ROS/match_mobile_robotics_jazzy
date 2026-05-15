#!/usr/bin/env bash
set -eo pipefail

source_setup() {
  set +u
  source "$1"
  set -u
}

set -u

WS="${WS:-/home/rosmatch/colcon_ws}"
ROBOT_NAME="${ROBOT_NAME:-mur620a}"
WORLD="${WORLD:-maze}"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
CLEAN_START="${CLEAN_START:-true}"

stop_old_sim() {
  local patterns=(
    "ros2 launch mur_launch_sim mur_base.launch.py"
    "gz sim"
    "gz sim server"
    "gz sim gui"
    "ros_gz_sim/create"
    "parameter_bridge"
    "robot_state_publisher"
    "controller_manager/spawner"
    "nav2_map_server"
    "nav2_amcl"
    "nav2_lifecycle_manager"
    "laserscan_multi_merger"
  )

  echo "[start_mur620a_sim] Stopping old ROS/Gazebo simulation processes..."
  for pattern in "${patterns[@]}"; do
    pkill -f "$pattern" 2>/dev/null || true
  done
  sleep 2
}

cd "$WS"

if [[ "${CLEAN_START}" == "true" ]]; then
  stop_old_sim
fi

source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

colcon build \
  --symlink-install \
  --packages-select mir_description mur_description mur_launch_sim mir_gazebo match_gazebo \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

source_setup install/setup.bash

rm -f "/tmp/mur_launch_sim/${ROBOT_NAME}_mur_controllers.yaml"
rm -f "/tmp/mur_launch_sim/${ROBOT_NAME}_localization.yaml"

exec ros2 launch mur_launch_sim mur_base.launch.py \
  robot_name:="${ROBOT_NAME}" \
  world:="${WORLD}" \
  load_controllers:=true \
  laser_merger:=true \
  localization:=true
