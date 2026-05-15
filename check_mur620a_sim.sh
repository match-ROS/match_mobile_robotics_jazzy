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

section() {
  printf '\n== %s ==\n' "$1"
}

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

section "Processes"
pgrep -af "mur_base.launch.py|gz sim|controller_manager|robot_state_publisher|nav2_|laserscan_multi_merger|parameter_bridge" || true
PIDS="$(pgrep -f "mur_base.launch.py|gz sim|controller_manager|robot_state_publisher|nav2_|laserscan_multi_merger|parameter_bridge" | tr '\n' ',' | sed 's/,$//')"
if [[ -n "${PIDS}" ]]; then
  ps -o pid,ppid,stat,etime,cmd -p "${PIDS}" || true
fi

section "Topics"
ros2 topic list | grep -E '(^/tf$|^/tf_static$|scan|odom|map|cmd_vel|joint_states|ground_truth)' || true

section "Scan Publishers"
ros2 topic info "/${ROBOT_NAME}/f_scan" -v || true
ros2 topic info "/${ROBOT_NAME}/b_scan" -v || true
ros2 topic info "/${ROBOT_NAME}/f_scan_raw" -v || true
ros2 topic info "/${ROBOT_NAME}/b_scan_raw" -v || true
ros2 topic info "/${ROBOT_NAME}/scan" -v || true

section "Nodes"
ros2 node list | sort || true

section "Controllers"
ros2 control list_controllers -c "/${ROBOT_NAME}/controller_manager" || true

section "Diff Drive Params"
for param in odom_frame_id base_frame_id tf_frame_prefix_enable tf_frame_prefix; do
  ros2 param get "/${ROBOT_NAME}/mobile_base_controller" "$param" || true
done

section "Localization"
ros2 lifecycle get /map_server || true
ros2 lifecycle get /amcl || true
ros2 param get /amcl global_frame_id || true
ros2 param get /amcl odom_frame_id || true
ros2 param get /amcl base_frame_id || true
ros2 param get /amcl scan_topic || true

section "Ground Truth"
ros2 topic info "/${ROBOT_NAME}/ground_truth/pose" -v || true
ros2 topic info "/${ROBOT_NAME}/ground_truth/odom" -v || true
timeout 5 ros2 topic echo "/${ROBOT_NAME}/ground_truth/pose" --once || true
timeout 5 ros2 topic echo "/${ROBOT_NAME}/ground_truth/odom" --once --field pose.pose || true

section "Merged Scan Header"
echo "-- /${ROBOT_NAME}/f_scan_raw"
timeout 5 ros2 topic echo "/${ROBOT_NAME}/f_scan_raw" --once --field header --qos-reliability reliable || true
echo "-- /${ROBOT_NAME}/b_scan_raw"
timeout 5 ros2 topic echo "/${ROBOT_NAME}/b_scan_raw" --once --field header --qos-reliability reliable || true
echo "-- /${ROBOT_NAME}/f_scan"
timeout 5 ros2 topic echo "/${ROBOT_NAME}/f_scan" --once --field header --qos-reliability best_effort || true
echo "-- /${ROBOT_NAME}/b_scan"
timeout 5 ros2 topic echo "/${ROBOT_NAME}/b_scan" --once --field header --qos-reliability best_effort || true
echo "-- /${ROBOT_NAME}/scan"
timeout 10 ros2 topic echo "/${ROBOT_NAME}/scan" --once --field header --qos-reliability best_effort || true

section "TF Checks"
timeout 5 ros2 run tf2_ros tf2_echo "${ROBOT_NAME}/odom" "${ROBOT_NAME}/base_footprint" || true
timeout 5 ros2 run tf2_ros tf2_echo map "${ROBOT_NAME}/odom" || true
