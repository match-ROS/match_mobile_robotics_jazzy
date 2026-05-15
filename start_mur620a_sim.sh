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

cd "$WS"

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

exec ros2 launch mur_launch_sim mur_base.launch.py \
  robot_name:="${ROBOT_NAME}" \
  world:="${WORLD}" \
  load_controllers:=true
