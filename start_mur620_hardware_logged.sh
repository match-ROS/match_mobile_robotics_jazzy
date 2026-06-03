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

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/${LAUNCH_PACKAGE}_${LAUNCH_FILE%.launch.py}_${TIMESTAMP}.log}"
LATEST_LOG="${LOG_DIR}/latest.log"
ENV_FILE="${LOG_DIR}/source_latest_ros_env.bash"

export ROS_DOMAIN_ID
export ROS_LOG_DIR
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
    "ewellix_state_to_joint_state.py"
    "move_group"
    "moveit_trajectory_controller_proxy.py"
    "robot_state_publisher"
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

  if ! id -nG | tr ' ' '\n' | grep -qx "dialout"; then
    echo "[start_mur620_hardware_logged] WARN: user $(id -un) is not in group dialout; Ewellix serial ports may fail with Permission denied."
  fi

  for port in "${ports[@]}"; do
    if [[ -e "$port" ]]; then
      ls -l "$port"
      if [[ ! -r "$port" || ! -w "$port" ]]; then
        echo "[start_mur620_hardware_logged] WARN: ${port} is not readable/writable for user $(id -un)."
      fi
    else
      echo "[start_mur620_hardware_logged] WARN: ${port} does not exist right now."
    fi
  done
  echo
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
echo "[start_mur620_hardware_logged] clean_start=${CLEAN_START}"
echo "[start_mur620_hardware_logged] extra args: $*"
echo

source_setup /opt/ros/jazzy/setup.bash
report_serial_access

if [[ "${CLEAN_START}" == "true" ]]; then
  stop_old_hardware
fi

if [[ "${BUILD_BEFORE_LAUNCH}" == "true" ]]; then
  echo "[start_mur620_hardware_logged] Building hardware packages..."
    colcon build \
      --symlink-install \
      --packages-up-to serial ewellix_driver mur_moveit_config mur_launch_hardware \
      --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  source_setup install/setup.bash
  echo "[start_mur620_hardware_logged] Build finished."
  echo
elif [[ -f install/setup.bash ]]; then
  source_setup install/setup.bash
fi

echo "[start_mur620_hardware_logged] Starting launch..."
set +e
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "$@"
status="$?"
set -e

echo
echo "[start_mur620_hardware_logged] exit_code=${status}"
echo "[start_mur620_hardware_logged] finished=$(date --iso-8601=seconds)"
echo "[start_mur620_hardware_logged] latest=${LATEST_LOG}"

exit "$status"
