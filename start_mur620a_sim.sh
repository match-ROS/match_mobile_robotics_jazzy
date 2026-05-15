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
CLEAN_START_FORCE_KILL="${CLEAN_START_FORCE_KILL:-true}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:-0.0}"
ROBOT_Z="${ROBOT_Z:-0.07}"
ROBOT_YAW="${ROBOT_YAW:-0.0}"

stop_old_sim() {
  local patterns=(
    "ros2 launch mur_launch_sim mur_base.launch.py"
    "gz sim"
    "gz sim server"
    "gz sim gui"
    "ruby .*gz sim"
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
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 2

  if [[ "${CLEAN_START_FORCE_KILL}" == "true" ]]; then
    for pattern in "${patterns[@]}"; do
      pkill -KILL -f "$pattern" 2>/dev/null || true
    done
    sleep 1
  fi
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
  x:="${ROBOT_X}" \
  y:="${ROBOT_Y}" \
  z:="${ROBOT_Z}" \
  Y:="${ROBOT_YAW}" \
  load_controllers:=true \
  laser_merger:=true \
  localization:=true \
  ground_truth:=true
