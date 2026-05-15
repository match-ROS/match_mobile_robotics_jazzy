#!/usr/bin/env bash
set -eo pipefail

source_setup() {
  set +u
  source "$1"
  set -u
}

section() {
  printf '\n== %s ==\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" || true
}

set -u

WS="${WS:-/home/rosmatch/colcon_ws}"
ROBOT_NAME="${ROBOT_NAME:-mur620a}"
WORLD="${WORLD:-maze}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LOG_DIR="${LOG_DIR:-${REPO}/logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/check_${ROBOT_NAME}_${STAMP}.log}"

mkdir -p "$LOG_DIR"
exec > >(tee "$LOG_FILE") 2>&1

echo "Writing diagnostic log to: $LOG_FILE"
echo "Workspace: $WS"
echo "Robot: $ROBOT_NAME"
echo "World: $WORLD"

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

section "Processes"
pgrep -af "mur_base.launch.py|gz sim|ground_truth|amcl|map_server|laserscan_multi_merger|parameter_bridge" || true

section "Core Topics"
ros2 topic list | grep -E '(^/tf$|^/tf_static$|/map$|scan$|ground_truth|/world/.*/pose/info|mobile_base_controller/odom)' || true

section "Controllers"
run ros2 control list_controllers -c "/${ROBOT_NAME}/controller_manager"

section "Localization TF"
run timeout 8 ros2 run tf2_ros tf2_echo "${ROBOT_NAME}/odom" "${ROBOT_NAME}/base_footprint"
run timeout 8 ros2 run tf2_ros tf2_echo map "${ROBOT_NAME}/odom"

section "Scan Sanity"
echo "-- merged scan header"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/scan" --once --field header --qos-reliability best_effort || true

section "Ground Truth Topics"
run ros2 topic info "/world/${WORLD}/pose/info"
run ros2 topic info "/${ROBOT_NAME}/ground_truth/pose"
run ros2 topic info "/${ROBOT_NAME}/ground_truth/odom"

echo "-- Gazebo pose names, first 80"
timeout 8 ros2 topic echo "/world/${WORLD}/pose/info" --once \
  | awk '/child_frame_id:/ {print; count++; if (count >= 80) exit}' || true

echo "-- /${ROBOT_NAME}/ground_truth/pose sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/ground_truth/pose" --once || true

echo "-- /${ROBOT_NAME}/ground_truth/odom pose sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/ground_truth/odom" --once --field pose.pose || true

section "Recent Ground Truth Logs"
find /home/rosmatch/.ros/log -maxdepth 2 -type f \
  \( -name '*ground_truth*.log' -o -name 'python3_*.log' \) \
  -mmin -30 -printf '%T@ %p\n' \
  | sort -nr \
  | head -5 \
  | cut -d' ' -f2- \
  | while read -r logfile; do
      echo "---- $logfile"
      tail -80 "$logfile" || true
    done

section "Done"
echo "Diagnostic log written to: $LOG_FILE"
