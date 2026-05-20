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
ROBOTS="${ROBOTS:-mur620a mur620b mur620c mur620d}"
LOG_DIR="${LOG_DIR:-${REPO}/logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/check_multi_mur_perf_${STAMP}.log}"

export ROS_DOMAIN_ID

mkdir -p "$LOG_DIR"
exec > >(tee "$LOG_FILE") 2>&1

echo "Writing performance log to: $LOG_FILE"
echo "Workspace: $WS"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Robots: $ROBOTS"

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

section "Host"
run date
run uptime
run uname -a
run lscpu
run free -h
run swapon --show

section "GPU Snapshot"
run nvidia-smi
echo "-- nvidia-smi pmon, per-process utilization samples"
timeout 8 nvidia-smi pmon -c 5 || true
echo "-- nvidia-smi dmon, device utilization samples"
timeout 8 nvidia-smi dmon -c 5 || true

section "Process Snapshot"
pgrep -af "multi_mur_base.launch.py|multi_mur620.launch.py|mur_base.launch.py|mur620.launch.py|gz sim|parameter_bridge|robot_state_publisher|map_server|amcl|controller_manager|laserscan_multi_merger|rviz2" || true
echo
ps -eo pid,ppid,pcpu,pmem,etime,rss,vsz,cmd --sort=-pcpu \
  | grep -E "gz sim|ruby .*gz|rviz2|robot_state_publisher|map_server|amcl|controller_manager|parameter_bridge|laserscan_multi_merger|jparse|move_group|nav2|bt_navigator|planner_server|controller_server" \
  | grep -v grep \
  | head -80 || true

section "Top Samples"
timeout 8 top -b -n 3 -d 1 -o %CPU | sed -n '1,45p' || true

section "Gazebo Transport"
run gz topic -l
echo "-- Gazebo /stats"
timeout 8 gz topic -e -t /stats -n 5 || true
echo "-- Gazebo /clock frequency"
timeout 8 gz topic -f -t /clock || true

section "ROS Rates"
echo "-- /clock"
timeout 8 ros2 topic hz /clock --window 50 || true
echo "-- /tf"
timeout 8 ros2 topic hz /tf --window 50 || true
echo "-- /tf_static info"
run ros2 topic info /tf_static --verbose
echo "-- /map info"
run ros2 topic info /map --verbose

for robot in $ROBOTS; do
  section "Robot ${robot} Rates"
  run ros2 topic info "/${robot}/scan" --verbose
  timeout 8 ros2 topic hz "/${robot}/scan" --window 20 || true
  timeout 8 ros2 topic hz "/${robot}/f_scan_raw" --window 20 || true
  timeout 8 ros2 topic hz "/${robot}/b_scan_raw" --window 20 || true
  timeout 8 ros2 topic hz "/${robot}/joint_states" --window 20 || true
  timeout 8 ros2 topic hz "/${robot}/mobile_base_controller/odom" --window 20 || true
  echo "-- ${robot} controller list"
  timeout 8 ros2 control list_controllers -c "/${robot}/controller_manager" || true
done

section "Robot Description Size"
for robot in $ROBOTS; do
  printf '%s ' "$robot"
  timeout 8 ros2 param get "/${robot}/${robot}_rsp" robot_description \
    | wc -c \
    | awk '{print "robot_description chars:", $1}' || true
done

section "Quick Diagnosis Hints"
echo "Compare two runs:"
echo "1) ros2 launch mur_launch_sim multi_mur_base.launch.py launch_rviz:=true"
echo "2) ros2 launch mur_launch_sim multi_mur_base.launch.py gz_gui:=false launch_rviz:=true"
echo "3) ros2 launch mur_launch_sim multi_mur_base.launch.py gz_gui:=false launch_rviz:=false simulate_lidars:=false localization:=false laser_merger:=false lidar_bridge:=false"
echo "4) ros2 launch mur_launch_sim multi_mur_base.launch.py gz_gui:=false launch_rviz:=false lidar_sensor_type:=gpu_lidar"
echo "If run 2 is much faster, Gazebo GUI/rendering is the bottleneck."
echo "If run 3 is much faster than server-only with lidars, Gazebo GPU LiDAR render passes are the bottleneck."
echo "If run 4 is much slower than the default LiDAR run, gpu_lidar rendering is the bottleneck and CPU lidar should remain the default."
echo "If GPU remains high in run 2, inspect nvidia-smi pmon for Xorg/Firefox/RViz/Gazebo attribution."
echo "If CPU is dominated by laserscan_multi_merger or AMCL, reduce scan/AMCL load next."

section "Done"
echo "Performance log written to: $LOG_FILE"
