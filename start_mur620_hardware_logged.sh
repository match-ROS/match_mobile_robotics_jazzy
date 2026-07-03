#!/usr/bin/env bash
set -Eeuo pipefail

source_setup() {
  set +u
  source "$1"
  set -u
}

WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LOG_DIR="${LOG_DIR:-${REPO}/logs/hardware}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-mur_launch_hardware}"
LAUNCH_FILE="${LAUNCH_FILE:-mur_620.launch.py}"
BUILD_BEFORE_LAUNCH="${BUILD_BEFORE_LAUNCH:-true}"
CLEAN_START="${CLEAN_START:-true}"
CLEAN_START_FORCE_KILL="${CLEAN_START_FORCE_KILL:-true}"
BUILD_PACKAGES="${BUILD_PACKAGES:-serial ewellix_driver mur_control mur_moveit_config mur_launch_hardware}"
ROBOT_PROFILE="${ROBOT_PROFILE:-mur620d}"
INTEGRATED_CARTESIAN_ACTIVE="${INTEGRATED_CARTESIAN_ACTIVE:-false}"
INTEGRATED_CARTESIAN_USE_FT="${INTEGRATED_CARTESIAN_USE_FT:-false}"
INTEGRATED_CARTESIAN_REQUIRE_WRENCH="${INTEGRATED_CARTESIAN_REQUIRE_WRENCH:-false}"
MOVEIT_WITH_INTEGRATED_CARTESIAN="${MOVEIT_WITH_INTEGRATED_CARTESIAN:-false}"
MUR_REQUIRE_HOST_PREFLIGHT="${MUR_REQUIRE_HOST_PREFLIGHT:-false}"
MUR_CHECK_UR_NETWORK="${MUR_CHECK_UR_NETWORK:-false}"
MUR_UR_HOSTS="${MUR_UR_HOSTS:-UR10_l UR10_r}"
MUR_AUTO_REVERSE_IP="${MUR_AUTO_REVERSE_IP:-true}"
MUR_EXPECTED_REVERSE_IP="${MUR_EXPECTED_REVERSE_IP:-}"
HOST_SETUP_SCRIPT="${HOST_SETUP_SCRIPT:-${REPO}/setup_mur_hardware_host.sh}"
HOST_DIAG_SCRIPT="${HOST_DIAG_SCRIPT:-${REPO}/diagnose_mur_hardware_host.sh}"
MUR_STANDALONE_SERVICE="${MUR_STANDALONE_SERVICE:-mur-mir-standalone.service}"
MUR_STANDALONE_WAS_ACTIVE=false
MUR_STANDALONE_STOPPED=false

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/${LAUNCH_PACKAGE}_${LAUNCH_FILE%.launch.py}_${TIMESTAMP}.log}"
LATEST_LOG="${LOG_DIR}/latest.log"
ENV_FILE="${LOG_DIR}/source_latest_ros_env.bash"

export ROS_DOMAIN_ID
export ROS_LOG_DIR
export ROS2CLI_NO_DAEMON=1
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

mkdir -p "$LOG_DIR"
mkdir -p "$ROS_LOG_DIR"
ln -sfn "$LOG_FILE" "$LATEST_LOG"
cat > "$ENV_FILE" <<EOF
source /opt/ros/jazzy/setup.bash
source ${WS}/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR=${ROS_LOG_DIR}
EOF

exec > >(tee -a "$LOG_FILE") 2>&1

cd "$WS"

stop_old_hardware() {
  local patterns=(
    "ros2 launch mur_launch_hardware"
    "mur_launch_hardware mur_620.launch.py"
    "controller_manager/ros2_control_node"
    "controller_manager/spawner"
    "ur_robot_driver/dashboard_client"
    "ur_robot_driver/robot_state_helper"
    "ur_robot_driver/controller_stopper_node"
    "ur_robot_driver/urscript_interface"
    "ur_robot_driver/trajectory_until_node"
    "ur_robot_driver/tool_communication.py"
    "ur_startup_enable.py"
    "ewellix_driver/ewellix_node"
    "ewellix_dual_state_to_joint_state.py"
    "ewellix_state_to_joint_state.py"
    "fake_mir_wheel_joint_states.py"
    "bms_can_node.py"
    "move_group"
    "moveit_trajectory_controller_proxy.py"
    "robot_state_publisher"
    "virtual_object_state_node"
    "virtual_object_tcp_transform_node"
    "virtual_object_demo_runner"
    "log_cooperative_tracking.py"
    "rviz2"
  )

  echo "[start_mur620_hardware_logged] Stopping old hardware bringup processes..."
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
}

report_serial_access() {
  local ports=("/dev/ttyUSB0" "/dev/ttyUSB1")
  local serial_blocking=0

  if ! id -nG | tr ' ' '\n' | grep -qx "dialout"; then
    echo "[start_mur620_hardware_logged] WARN: user $(id -un) is not in group dialout; Ewellix serial ports may fail with Permission denied."
    echo "MUR_PREFLIGHT: status=fail issue=dialout user=$(id -un)"
    serial_blocking=1
  else
    echo "MUR_PREFLIGHT: status=ok issue=dialout user=$(id -un)"
  fi

  for port in "${ports[@]}"; do
    if [[ -e "$port" ]]; then
      ls -l "$port"
      if [[ ! -r "$port" || ! -w "$port" ]]; then
        echo "[start_mur620_hardware_logged] WARN: ${port} is not readable/writable for user $(id -un)."
        echo "MUR_PREFLIGHT: status=fail issue=serial_permission port=${port}"
        serial_blocking=1
      else
        echo "MUR_PREFLIGHT: status=ok issue=serial_permission port=${port}"
      fi
    else
      echo "[start_mur620_hardware_logged] WARN: ${port} does not exist right now."
      echo "MUR_PREFLIGHT: status=warn issue=serial_missing port=${port}"
    fi
  done
  if [[ "$serial_blocking" -ne 0 ]]; then
    echo "MUR_PREFLIGHT: summary=fail blocking=serial_permission"
  else
    echo "MUR_PREFLIGHT: summary=ok blocking=none"
  fi
  echo
  return "$serial_blocking"
}

restore_standalone_mir_service() {
  if [[ "${MUR_STANDALONE_STOPPED}" != "true" || "${MUR_STANDALONE_WAS_ACTIVE}" != "true" ]]; then
    return 0
  fi
  echo "[start_mur620_hardware_logged] Restarting ${MUR_STANDALONE_SERVICE} after GUI MiR takeover."
  if sudo -n systemctl start "${MUR_STANDALONE_SERVICE}"; then
    echo "MUR_STANDALONE: status=restarted service=${MUR_STANDALONE_SERVICE}"
  else
    echo "MUR_STANDALONE: status=warn issue=restart_failed service=${MUR_STANDALONE_SERVICE}"
  fi
}

trap restore_standalone_mir_service EXIT

run_host_preflight() {
  local preflight_status=0
  if [[ -x "$HOST_SETUP_SCRIPT" ]]; then
    "$HOST_SETUP_SCRIPT" --check || preflight_status="$?"
  else
    echo "MUR_HOST_CHECK: status=warn issue=setup_script_missing path=${HOST_SETUP_SCRIPT}"
  fi

  if [[ "$preflight_status" -ne 0 && "${MUR_REQUIRE_HOST_PREFLIGHT}" == "true" ]]; then
    echo "[start_mur620_hardware_logged] ERROR: strict host preflight failed; refusing hardware launch."
    echo "[start_mur620_hardware_logged] Run: ${HOST_SETUP_SCRIPT} --apply"
    exit "$preflight_status"
  fi
  if [[ "$preflight_status" -ne 0 ]]; then
    echo "[start_mur620_hardware_logged] WARN: host preflight reported blocking issues, but strict mode is disabled."
  fi
}

echo "[start_mur620_hardware_logged] $(date --iso-8601=seconds)"
echo "[start_mur620_hardware_logged] WS=${WS}"
echo "[start_mur620_hardware_logged] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[start_mur620_hardware_logged] ROS_LOG_DIR=${ROS_LOG_DIR}"
echo "[start_mur620_hardware_logged] launch=${LAUNCH_PACKAGE} ${LAUNCH_FILE}"
echo "[start_mur620_hardware_logged] log=${LOG_FILE}"
echo "[start_mur620_hardware_logged] latest=${LATEST_LOG}"
echo "[start_mur620_hardware_logged] inspect_env=${ENV_FILE}"
echo "[start_mur620_hardware_logged] inspect command: source ${ENV_FILE} && ros2 topic list"
echo "[start_mur620_hardware_logged] build_before_launch=${BUILD_BEFORE_LAUNCH}"
echo "[start_mur620_hardware_logged] build_packages=${BUILD_PACKAGES}"
echo "[start_mur620_hardware_logged] clean_start=${CLEAN_START}"
echo "[start_mur620_hardware_logged] robot_profile=${ROBOT_PROFILE}"
echo "[start_mur620_hardware_logged] integrated_cartesian_admittance_active=${INTEGRATED_CARTESIAN_ACTIVE}"
echo "[start_mur620_hardware_logged] integrated_cartesian_admittance_use_ft=${INTEGRATED_CARTESIAN_USE_FT}"
echo "[start_mur620_hardware_logged] integrated_cartesian_admittance_require_wrench=${INTEGRATED_CARTESIAN_REQUIRE_WRENCH}"
echo "[start_mur620_hardware_logged] moveit_with_integrated_cartesian=${MOVEIT_WITH_INTEGRATED_CARTESIAN}"
echo "[start_mur620_hardware_logged] mur_require_host_preflight=${MUR_REQUIRE_HOST_PREFLIGHT}"
echo "[start_mur620_hardware_logged] extra args: $*"
echo

source_setup /opt/ros/jazzy/setup.bash
run_host_preflight
report_serial_access || true

if [[ "${CLEAN_START}" == "true" ]]; then
  stop_old_hardware
fi

if [[ "${BUILD_BEFORE_LAUNCH}" == "true" ]]; then
  echo "[start_mur620_hardware_logged] Building hardware packages..."
  # shellcheck disable=SC2086
  colcon build \
    --symlink-install \
    --packages-up-to ${BUILD_PACKAGES} \
    --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  source_setup install/setup.bash
  echo "[start_mur620_hardware_logged] Build finished."
  echo
elif [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

declare -a LAUNCH_ARGS=("$@")
has_launch_arg() {
  local key="$1"
  shift
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

route_src_ip() {
  awk '{
    for (i = 1; i < NF; i++) {
      if ($i == "src") {
        print $(i + 1)
        exit
      }
    }
  }'
}

add_auto_reverse_ip() {
  if [[ "${MUR_AUTO_REVERSE_IP}" != "true" ]]; then
    echo "MUR_REVERSE_IP: status=skip reason=disabled"
    return 0
  fi
  if has_launch_arg "reverse_ip" "${LAUNCH_ARGS[@]}"; then
    echo "MUR_REVERSE_IP: status=skip reason=launch_arg_already_set"
    return 0
  fi
  if [[ "${MUR_CHECK_UR_NETWORK}" != "true" ]]; then
    echo "MUR_REVERSE_IP: status=skip reason=ur_network_check_disabled"
    return 0
  fi

  local host resolved route src_ip reverse_ip=""
  for host in ${MUR_UR_HOSTS}; do
    resolved="$(getent hosts "$host" | awk '{print $1}' | head -n 1 || true)"
    if [[ -z "$resolved" ]]; then
      echo "MUR_REVERSE_IP: status=fail host=${host} detail=name_resolution_failed"
      return 2
    fi

    route="$(ip route get "$resolved" 2>/dev/null | head -n 1 || true)"
    src_ip="$(printf '%s\n' "$route" | route_src_ip)"
    if [[ -z "$src_ip" ]]; then
      echo "MUR_REVERSE_IP: status=fail host=${host} ip=${resolved} detail=missing_route_src route=$(printf '%q' "$route")"
      return 2
    fi

    echo "MUR_REVERSE_IP: route host=${host} ip=${resolved} src=${src_ip} ${route}"
    if [[ -z "$reverse_ip" ]]; then
      reverse_ip="$src_ip"
    elif [[ "$reverse_ip" != "$src_ip" ]]; then
      echo "MUR_REVERSE_IP: status=fail detail=inconsistent_route_src first=${reverse_ip} host=${host} src=${src_ip}"
      return 2
    fi
  done

  if [[ -z "$reverse_ip" ]]; then
    echo "MUR_REVERSE_IP: status=fail detail=no_ur_hosts"
    return 2
  fi

  if [[ -n "$MUR_EXPECTED_REVERSE_IP" && "$reverse_ip" != "$MUR_EXPECTED_REVERSE_IP" ]]; then
    echo "MUR_REVERSE_IP: status=fail detail=unexpected_reverse_ip expected=${MUR_EXPECTED_REVERSE_IP} actual=${reverse_ip}"
    return 2
  fi

  LAUNCH_ARGS+=("reverse_ip:=${reverse_ip}")
  echo "MUR_REVERSE_IP: status=ok value=${reverse_ip}"
}

launch_arg_value() {
  local key="$1"
  local default_value="$2"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == "${key}:="* ]]; then
      printf '%s\n' "${arg#*:=}"
      return 0
    fi
  done
  printf '%s\n' "$default_value"
}

configure_bms_can_interface() {
  local launch_bms can_interface can_bitrate
  launch_bms="$(launch_arg_value "launch_bms" "true")"
  if [[ "$launch_bms" != "true" ]]; then
    echo "MUR_BMS_CAN: status=skip reason=launch_bms_${launch_bms}"
    return 0
  fi

  can_interface="$(launch_arg_value "bms_can_interface" "can0")"
  can_bitrate="$(launch_arg_value "bms_can_bitrate" "250000")"
  if [[ ! -e "/sys/class/net/${can_interface}" ]]; then
    echo "MUR_BMS_CAN: status=warn issue=interface_missing interface=${can_interface}"
    return 0
  fi

  echo "MUR_BMS_CAN: configuring interface=${can_interface} bitrate=${can_bitrate}"
  if sudo -n ip link set "$can_interface" up type can bitrate "$can_bitrate"; then
    ip -details link show "$can_interface" | sed 's/^/MUR_BMS_CAN: /'
    echo "MUR_BMS_CAN: status=ok interface=${can_interface}"
  else
    echo "MUR_BMS_CAN: status=warn issue=configure_failed interface=${can_interface} detail=sudo_or_ip_link_failed"
  fi
}

clean_stale_mir_processes_for_takeover() {
  local patterns=(
    "mir_driver/lib/mir_driver/mir_bridge"
    "twist_stamper_cmd_vel_mir"
    "mir_launch_hardware/lib/mir_launch_hardware/mir_battery_state_publisher"
    "mir_launch_hardware/lib/mir_launch_hardware/mir_pose_simple"
    "mir_restapi/lib/mir_restapi/mir_restapi_server"
    "mir_launch_hardware/lib/mir_launch_hardware/rgb_control"
    "mur_mir_teleop/lib/mur_mir_teleop/standalone_supervisor"
    "mur_mir_teleop/lib/mur_mir_teleop/ds4_mir_teleop"
    "joy/joy_node"
  )

  echo "[start_mur620_hardware_logged] Cleaning stale MiR driver/teleop processes before GUI takeover..."
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
}

takeover_standalone_mir_if_needed() {
  local launch_mir
  launch_mir="$(launch_arg_value "launch_mir" "false")"
  if [[ "${launch_mir}" != "true" ]]; then
    echo "MUR_STANDALONE: status=skip reason=launch_mir_${launch_mir}"
    return 0
  fi

  if ! systemctl list-unit-files "${MUR_STANDALONE_SERVICE}" --no-legend 2>/dev/null | grep -q "${MUR_STANDALONE_SERVICE}"; then
    echo "MUR_STANDALONE: status=skip reason=service_not_installed service=${MUR_STANDALONE_SERVICE}"
    clean_stale_mir_processes_for_takeover
    return 0
  fi

  if sudo -n systemctl is-active --quiet "${MUR_STANDALONE_SERVICE}"; then
    MUR_STANDALONE_WAS_ACTIVE=true
    echo "[start_mur620_hardware_logged] Stopping ${MUR_STANDALONE_SERVICE} for GUI MiR takeover."
    if sudo -n systemctl stop "${MUR_STANDALONE_SERVICE}"; then
      MUR_STANDALONE_STOPPED=true
      echo "MUR_STANDALONE: status=stopped service=${MUR_STANDALONE_SERVICE}"
    else
      echo "MUR_STANDALONE: status=warn issue=stop_failed service=${MUR_STANDALONE_SERVICE}"
    fi
  else
    echo "MUR_STANDALONE: status=inactive service=${MUR_STANDALONE_SERVICE}"
  fi

  clean_stale_mir_processes_for_takeover
}

if ! has_launch_arg "robot_profile" "${LAUNCH_ARGS[@]}"; then
  LAUNCH_ARGS+=("robot_profile:=${ROBOT_PROFILE}")
fi
if [[ "${INTEGRATED_CARTESIAN_ACTIVE}" == "true" ]]; then
  LAUNCH_ARGS+=(
    "use_integrated_cartesian_admittance_controller:=true"
    "integrated_controller_initial_active:=true"
    "integrated_controller_use_ft_sensor:=${INTEGRATED_CARTESIAN_USE_FT}"
    "integrated_controller_require_wrench:=${INTEGRATED_CARTESIAN_REQUIRE_WRENCH}"
  )
fi
if [[ "${MOVEIT_WITH_INTEGRATED_CARTESIAN}" == "true" ]]; then
  LAUNCH_ARGS+=("moveit_velocity_controller:=integrated_cartesian_admittance_controller")
fi
add_auto_reverse_ip
takeover_standalone_mir_if_needed
configure_bms_can_interface

echo "[start_mur620_hardware_logged] Starting launch..."
printf '[start_mur620_hardware_logged] full command: ros2 launch %s %s' "$LAUNCH_PACKAGE" "$LAUNCH_FILE"
printf ' %q' "${LAUNCH_ARGS[@]}"
printf '\n'
set +e
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"
status="$?"
set -e

echo
echo "[start_mur620_hardware_logged] exit_code=${status}"
echo "[start_mur620_hardware_logged] finished=$(date --iso-8601=seconds)"
echo "[start_mur620_hardware_logged] latest=${LATEST_LOG}"
if [[ -x "$HOST_DIAG_SCRIPT" ]]; then
  echo "[start_mur620_hardware_logged] diagnosis summary:"
  "$HOST_DIAG_SCRIPT" "$LOG_FILE" || true
fi

exit "$status"
