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
pgrep -af "multi_mur620.launch.py|mur620.launch.py|move_group|rviz2|controller_manager|robot_state_publisher" || true
echo
ps -eo pid,ppid,pcpu,pmem,etime,cmd --sort=-pcpu \
  | grep -E "move_group|rviz2|gz sim|robot_state_publisher|controller_manager" \
  | grep -v grep \
  | head -40 || true

section "Core Graph"
run ros2 node list --no-daemon
run ros2 topic list --no-daemon
run ros2 action list

section "MoveIt Topics"
for topic in /mur620a/monitored_planning_scene /mur620a/display_planned_path /mur620b/monitored_planning_scene /mur620b/display_planned_path /monitored_planning_scene /display_planned_path; do
  run ros2 topic info "$topic" --verbose
done

for robot in $ROBOTS; do
  section "MoveIt ${robot}"

  echo "-- Nodes"
  ros2 node list --no-daemon | grep -E "^/${robot}/move_group|/${robot}/moveit|moveit.*${robot}|${robot}.*moveit" || true

  echo "-- MoveIt action"
  run ros2 action info "/${robot}/move_action"

  echo "-- Robot descriptions"
  timeout 8 ros2 param get "/${robot}/move_group" robot_description \
    | wc -c \
    | awk '{print "robot_description chars:", $1}' || true
  timeout 8 ros2 param get "/${robot}/move_group" robot_description_semantic \
    | wc -c \
    | awk '{print "robot_description_semantic chars:", $1}' || true

  echo "-- MoveIt core params"
  run ros2 param get "/${robot}/move_group" planning_pipelines
  run ros2 param get "/${robot}/move_group" default_planning_pipeline
  run ros2 param get "/${robot}/move_group" robot_description_kinematics.UR_arm_l.kinematics_solver
  run ros2 param get "/${robot}/move_group" robot_description_kinematics.UR_arm_r.kinematics_solver
  run ros2 param get "/${robot}/move_group" robot_description_planning.joint_limits.UR10_l/shoulder_pan_joint.max_velocity

  echo "-- Controllers"
  timeout 8 ros2 control list_controllers -c "/${robot}/controller_manager" || true

  echo "-- Joint states"
  run ros2 topic info "/${robot}/joint_states" --verbose
  timeout 8 ros2 topic echo "/${robot}/joint_states" sensor_msgs/msg/JointState --once \
    | sed -n '1,140p' || true

  echo "-- Current TF for MoveIt base"
  timeout 7 ros2 run tf2_ros tf2_echo map "${robot}/base_footprint" || true
  timeout 7 ros2 run tf2_ros tf2_echo "${robot}/base_footprint" "${robot}/UR10_l/tool0" || true
done

section "Recent MoveIt/RViz Logs"
find /home/rosmatch/.ros/log -maxdepth 2 -type f \
  \( -name '*move_group*.log' -o -name '*rviz*.log' -o -name '*moveit*.log' \) \
  -mmin -45 -printf '%T@ %p\n' \
  | sort -nr \
  | head -16 \
  | cut -d' ' -f2- \
  | while read -r logfile; do
      echo "---- $logfile"
      tail -120 "$logfile" || true
    done

section "RViz Checks"
echo "For namespace mur620a, MotionPlanning should use:"
echo "  Move Group Namespace: mur620a"
echo "  Planning Scene Topic: monitored_planning_scene"
echo "  Robot Description: robot_description"
echo "  Planned Path Trajectory Topic: display_planned_path"

section "Done"
echo "Diagnostic log written to: $LOG_FILE"
