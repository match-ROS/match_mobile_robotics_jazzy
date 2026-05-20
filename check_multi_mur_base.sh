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
WORLD="${WORLD:-scale}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
ROBOTS="${ROBOTS:-mur620a mur620b mur620c mur620d}"
LOG_DIR="${LOG_DIR:-${REPO}/logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/check_multi_mur_base_${STAMP}.log}"

export ROS_DOMAIN_ID

mkdir -p "$LOG_DIR"
exec > >(tee "$LOG_FILE") 2>&1

echo "Writing diagnostic log to: $LOG_FILE"
echo "Workspace: $WS"
echo "World: $WORLD"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Robots: $ROBOTS"

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

section "Processes"
pgrep -af "multi_mur_base.launch.py|multi_mur620.launch.py|mur_base.launch.py|mur620.launch.py|gz sim|parameter_bridge|robot_state_publisher|map_server|amcl|lifecycle_manager|controller_manager|laserscan_multi_merger|rviz2" || true
echo
ps -eo pid,ppid,pcpu,pmem,etime,cmd --sort=-pcpu \
  | grep -E "gz sim|ruby .*gz|rviz2|robot_state_publisher|map_server|amcl|controller_manager|parameter_bridge|laserscan_multi_merger" \
  | grep -v grep \
  | head -40 || true

section "Core Graph"
run ros2 node list --no-daemon
run ros2 topic list --no-daemon

section "Clock"
run ros2 topic info /clock --verbose
echo "-- /clock sample"
timeout 5 ros2 topic echo /clock --once || true

section "Map"
run ros2 lifecycle get /map_server
run ros2 topic info /map --verbose
echo "-- /map metadata sample, transient local"
timeout 8 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  | sed -n '1,80p' || true

section "RViz Hints"
echo "RViz Fixed Frame should be: map"
echo "RViz Map display topic should be: /map"
echo "If /map has a publisher but no map appears, set Map display QoS Durability to Transient Local."
echo "For performance, avoid enabling four RobotModel displays with full visual meshes at once."
echo "Lightweight config: ros2 launch mur_launch_sim multi_mur_base.launch.py launch_rviz:=true"

for robot in $ROBOTS; do
  section "Robot ${robot}"

  echo "-- Nodes"
  ros2 node list --no-daemon | grep -E "^/${robot}(/|_|$)|${robot}" || true

  echo "-- Controllers"
  timeout 8 ros2 control list_controllers -c "/${robot}/controller_manager" || true

  echo "-- Topics"
  ros2 topic list --no-daemon | grep -E "^/${robot}/(scan|scan_merged_raw|f_scan|b_scan|mobile_base_controller/odom|ground_truth|joint_states)" || true

  echo "-- AMCL lifecycle"
  run ros2 lifecycle get "/${robot}/amcl"
  run ros2 param get "/${robot}/amcl" map_topic
  run ros2 param get "/${robot}/amcl" scan_topic
  run ros2 param get "/${robot}/amcl" odom_frame_id
  run ros2 param get "/${robot}/amcl" base_frame_id

  echo "-- TF map -> ${robot}/odom"
  timeout 7 ros2 run tf2_ros tf2_echo map "${robot}/odom" || true

  echo "-- TF ${robot}/odom -> ${robot}/base_footprint"
  timeout 7 ros2 run tf2_ros tf2_echo "${robot}/odom" "${robot}/base_footprint" || true

  echo "-- Scan info"
  run ros2 topic info "/${robot}/scan" --verbose
  echo "-- /${robot}/scan header sample"
  timeout 8 ros2 topic echo "/${robot}/scan" --once --field header --qos-reliability reliable || true

  echo "-- Odom sample"
  timeout 8 ros2 topic echo "/${robot}/mobile_base_controller/odom" --once --field header || true

  echo "-- Robot description size"
  timeout 8 ros2 param get "/${robot}/${robot}_rsp" robot_description \
    | wc -c \
    | awk '{print "robot_description chars:", $1}' || true
done

section "Recent ROS Logs"
find /home/rosmatch/.ros/log -maxdepth 2 -type f \
  \( -name '*map_server*.log' -o -name '*amcl*.log' -o -name '*lifecycle*.log' -o -name '*robot_state_publisher*.log' -o -name '*rviz*.log' \) \
  -mmin -45 -printf '%T@ %p\n' \
  | sort -nr \
  | head -12 \
  | cut -d' ' -f2- \
  | while read -r logfile; do
      echo "---- $logfile"
      tail -80 "$logfile" || true
    done

section "Done"
echo "Diagnostic log written to: $LOG_FILE"
