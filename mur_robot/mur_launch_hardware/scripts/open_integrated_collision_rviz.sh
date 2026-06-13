#!/usr/bin/env bash
set -Eeuo pipefail

source_setup() {
  set +u
  source "$1"
  set -u
}

WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LOG_DIR="${LOG_DIR:-${REPO}/logs/rviz_collision_debug}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"

source_setup /opt/ros/jazzy/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source_setup "${WS}/install/setup.bash"
fi

export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"

RVIZ_CONFIG="${RVIZ_CONFIG:-$(ros2 pkg prefix mur_launch_hardware)/share/mur_launch_hardware/config/rviz/integrated_collision_debug.rviz}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/integrated_collision_rviz_${TIMESTAMP}.log}"
LATEST_LOG="${LOG_DIR}/latest.log"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_FILE" "$LATEST_LOG"

echo "[open_integrated_collision_rviz] $(date --iso-8601=seconds)" | tee -a "$LOG_FILE"
echo "[open_integrated_collision_rviz] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" | tee -a "$LOG_FILE"
echo "[open_integrated_collision_rviz] config=${RVIZ_CONFIG}" | tee -a "$LOG_FILE"
echo "[open_integrated_collision_rviz] log=${LOG_FILE}" | tee -a "$LOG_FILE"

exec rviz2 -d "$RVIZ_CONFIG" --ros-args --log-level warn 2>&1 | tee -a "$LOG_FILE"
