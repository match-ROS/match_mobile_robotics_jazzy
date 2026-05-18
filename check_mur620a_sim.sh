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
pgrep -af "mur_base.launch.py|gz sim|ground_truth|amcl|map_server|controller_server|planner_server|behavior_server|bt_navigator" || true

section "Core Topics"
ros2 topic list | grep -E '(^/tf$|^/tf_static$|/map$|ground_truth|/f_scan$|/b_scan$|/scan$|/scan_merged_raw$|mobile_base_controller/odom)' || true

section "Clock"
run ros2 topic info /clock --verbose
echo "-- /clock samples"
timeout 5 ros2 topic echo /clock --once || true
sleep 1
timeout 5 ros2 topic echo /clock --once || true

section "Map"
run ros2 lifecycle get /map_server
run ros2 topic info /map --verbose
echo "-- /map sample with transient local QoS"
timeout 8 ros2 topic echo /map --once --field info --qos-durability transient_local || true
run ros2 param get /amcl alpha1
run ros2 param get /amcl alpha4
run ros2 param get /amcl update_min_a
run ros2 param get /amcl update_min_d
echo "-- map -> odom"
timeout 8 ros2 run tf2_ros tf2_echo map "${ROBOT_NAME}/odom" || true

section "Scan QoS"
run ros2 topic info "/${ROBOT_NAME}/f_scan" --verbose
run ros2 topic info "/${ROBOT_NAME}/b_scan" --verbose
run ros2 topic info "/${ROBOT_NAME}/scan_merged_raw" --verbose
run ros2 topic info "/${ROBOT_NAME}/scan" --verbose
echo "-- /${ROBOT_NAME}/scan sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/scan" --once --field header --qos-reliability reliable || true

section "Ground Truth Topics"
run ros2 topic info "/${ROBOT_NAME}/ground_truth/pose"
run ros2 topic info "/${ROBOT_NAME}/ground_truth/odom"

echo "-- /${ROBOT_NAME}/ground_truth/pose sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/ground_truth/pose" --once || true

echo "-- /${ROBOT_NAME}/ground_truth/odom pose sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/ground_truth/odom" --once --field pose.pose || true

echo "-- Gazebo pose names, first 40"
timeout 5 gz topic -e -t "/world/${WORLD}/pose/info" -n 1 \
  | awk '/name:/ {print; count++; if (count >= 40) exit}' || true

section "Navigation"
run ros2 lifecycle get /controller_server
run ros2 lifecycle get /planner_server
run ros2 lifecycle get /behavior_server
run ros2 lifecycle get /bt_navigator
run ros2 action list
run ros2 param get /controller_server enable_stamped_cmd_vel
run ros2 param get /controller_server odom_topic
run ros2 param get /behavior_server enable_stamped_cmd_vel
run ros2 topic info "/${ROBOT_NAME}/goal_pose" --verbose
run ros2 topic info /odom --verbose
run ros2 topic info "/${ROBOT_NAME}/mobile_base_controller/odom" --verbose
run ros2 topic info "/${ROBOT_NAME}/mobile_base_controller/cmd_vel" --verbose
run ros2 param get "/${ROBOT_NAME}/mobile_base_controller" wheel_separation
run ros2 param get "/${ROBOT_NAME}/mobile_base_controller" wheel_radius
echo "-- /${ROBOT_NAME}/mobile_base_controller/cmd_vel TwistStamped sample"
timeout 8 ros2 topic echo "/${ROBOT_NAME}/mobile_base_controller/cmd_vel" geometry_msgs/msg/TwistStamped --once || true

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
