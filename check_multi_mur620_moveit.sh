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

echo_once() {
  local topic="$1"
  local type="$2"
  local lines="${3:-60}"
  timeout 4 ros2 topic echo "$topic" "$type" --once | sed -n "1,${lines}p" || true
}

tf_once() {
  local target="$1"
  local source="$2"
  timeout 4 ros2 run tf2_ros tf2_echo "$target" "$source" || true
}

set -u

WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
ROBOTS="${ROBOTS:-mur620a mur620b}"
LOG_DIR="${LOG_DIR:-${REPO}/logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/check_multi_mur620_moveit_${STAMP}.log}"

export ROS_DOMAIN_ID

mkdir -p "$LOG_DIR"
exec > >(tee "$LOG_FILE") 2>&1

echo "Writing diagnostic log to: $LOG_FILE"
echo "Workspace: $WS"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Robots: $ROBOTS"

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

section "Processes"
pgrep -af "multi_mur620.launch.py|move_group|rviz2|moveit_tf_republisher|ground_truth|fake_localization|map_server|parameter_bridge" || true
ps -eo pid,ppid,pcpu,pmem,etime,cmd --sort=-pcpu \
  | grep -E "move_group|rviz2|gz sim|moveit_tf_republisher|ground_truth|fake_localization|map_server|parameter_bridge" \
  | grep -v grep \
  | head -30 || true

section "Core Topics"
run ros2 topic info /clock --verbose
run ros2 topic info /map --verbose
echo_once /map nav_msgs/msg/OccupancyGrid 45
run ros2 lifecycle get /map_server
run ros2 topic list
run ros2 topic info /robot_description_semantic --verbose

section "Gazebo Dynamic Pose"
GZ_DYNAMIC_POSE_TOPIC="$(gz topic -l | grep -E '^/world/.+/dynamic_pose/info$' | head -1 || true)"
if [[ -n "$GZ_DYNAMIC_POSE_TOPIC" ]]; then
  echo "Gazebo dynamic pose topic: $GZ_DYNAMIC_POSE_TOPIC"
  timeout 4 gz topic -e -t "$GZ_DYNAMIC_POSE_TOPIC" | sed -n '1,90p' || true
else
  echo "No Gazebo /world/<world>/dynamic_pose/info topic found."
fi

section "TF Topics"
run ros2 topic info /tf --verbose
run ros2 topic info /tf_static --verbose

for robot in $ROBOTS; do
  section "Robot ${robot}"

  echo "-- nodes"
  ros2 node list --no-daemon | grep -E "${robot}.*ground_truth|${robot}.*fake_localization|/${robot}/move_group" || true

  echo "-- ground truth odom"
  run ros2 topic info "/${robot}/ground_truth/odom" --verbose
  echo_once "/${robot}/ground_truth/odom" nav_msgs/msg/Odometry 70

  echo "-- fake localization params"
  run ros2 param get "/${robot}_fake_localization" global_frame_id
  run ros2 param get "/${robot}_fake_localization" odom_frame_id
  run ros2 param get "/${robot}_fake_localization" base_frame_id
  run ros2 param get "/${robot}/mobile_base_controller" enable_odom_tf

  echo "-- expected TF chain"
  tf_once map "${robot}/odom"
  tf_once "${robot}/odom" "${robot}/base_footprint"
  tf_once map "${robot}/base_footprint"

  echo "-- MoveIt quick check"
  run ros2 node list --no-daemon
  run ros2 action info "/${robot}/move_action"
  run ros2 topic info "/${robot}/monitored_planning_scene" --verbose
  run ros2 topic info "/${robot}/robot_description_semantic" --verbose
  run ros2 topic info "/${robot}/moveit_tf" --verbose
  run ros2 topic info "/${robot}/moveit_tf_static" --verbose
  echo_once "/${robot}/moveit_tf_static" tf2_msgs/msg/TFMessage 80
done

section "Done"
echo "Diagnostic log written to: $LOG_FILE"
