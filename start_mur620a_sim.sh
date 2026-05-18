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
WORLD="${WORLD:-scale}"
MAP="${MAP:-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
CLEAN_START="${CLEAN_START:-true}"
CLEAN_START_FORCE_KILL="${CLEAN_START_FORCE_KILL:-true}"
ROBOT_X="${ROBOT_X:-44.0}"
ROBOT_Y="${ROBOT_Y:-44.0}"
ROBOT_Z="${ROBOT_Z:-0.07}"
ROBOT_YAW="${ROBOT_YAW:-0.0}"
LAUNCH_MOVEIT="${LAUNCH_MOVEIT:-true}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
LAUNCH_SERVO="${LAUNCH_SERVO:-false}"
LOAD_ARM_CONTROLLERS="${LOAD_ARM_CONTROLLERS:-true}"

export ROS_DOMAIN_ID

stop_old_sim() {
  local patterns=(
    "ros2 launch mur_launch_sim mur_base.launch.py"
    "ros2 launch mur_launch_sim mur620.launch.py"
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
    "nav2_controller"
    "nav2_planner"
    "nav2_behaviors"
    "nav2_bt_navigator"
    "nav2_lifecycle_manager"
    "laserscan_multi_merger"
    "move_group"
    "moveit_servo"
    "servo_node"
    "rviz2"
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

  if command -v ros2 >/dev/null 2>&1; then
    echo "[start_mur620a_sim] Waiting for old /clock publisher to disappear..."
    for _ in {1..20}; do
      if ! ros2 topic info /clock 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        return
      fi
      sleep 0.25
    done
    echo "[start_mur620a_sim] Warning: /clock publisher is still visible. Close old Gazebo/ROS sessions if TF time-jump warnings continue."
  fi
}

cd "$WS"

source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

echo "[start_mur620a_sim] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

if [[ "${CLEAN_START}" == "true" ]]; then
  stop_old_sim
fi

colcon build \
  --symlink-install \
  --packages-select mir_description mur_description mur_launch_sim mur_moveit_config mir_gazebo match_gazebo \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

source_setup install/setup.bash

rm -f "/tmp/mur_launch_sim/${ROBOT_NAME}_mur_controllers.yaml"
rm -f "/tmp/mur_launch_sim/${ROBOT_NAME}_localization.yaml"
rm -f "/tmp/mur_launch_sim/${ROBOT_NAME}_navigation.yaml"

if [[ -z "${MAP}" ]]; then
  MAP="${WS}/install/mir_gazebo/share/mir_gazebo/maps/${WORLD}.yaml"
fi

exec ros2 launch mur_launch_sim mur620.launch.py \
  robot_name:="${ROBOT_NAME}" \
  world:="${WORLD}" \
  map:="${MAP}" \
  x:="${ROBOT_X}" \
  y:="${ROBOT_Y}" \
  z:="${ROBOT_Z}" \
  Y:="${ROBOT_YAW}" \
  load_controllers:=true \
  load_arm_controllers:="${LOAD_ARM_CONTROLLERS}" \
  laser_merger:=true \
  localization:=true \
  navigation:=true \
  ground_truth:=true \
  include_gz:=true \
  launch_moveit:="${LAUNCH_MOVEIT}" \
  launch_rviz:="${LAUNCH_RVIZ}" \
  launch_servo:="${LAUNCH_SERVO}"
