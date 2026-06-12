#!/usr/bin/env bash
set -Eeuo pipefail

source_setup() {
  set +u
  source "$1"
  set -u
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
LOG_DIR="${LOG_DIR:-${SCRIPT_DIR}/logs/integrated_cartesian_step_response}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
BUILD_BEFORE_RUN="${BUILD_BEFORE_RUN:-true}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/integrated_cartesian_step_response_${TIMESTAMP}.log}"
LATEST_LOG="${LOG_DIR}/latest.log"

export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_FILE" "$LATEST_LOG"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "[run_integrated_cartesian_step_response_logged] $(date --iso-8601=seconds)"
echo "[run_integrated_cartesian_step_response_logged] WS=${WS}"
echo "[run_integrated_cartesian_step_response_logged] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[run_integrated_cartesian_step_response_logged] log=${LOG_FILE}"
echo "[run_integrated_cartesian_step_response_logged] latest=${LATEST_LOG}"
echo "[run_integrated_cartesian_step_response_logged] extra args: $*"
echo

cd "$WS"
source_setup /opt/ros/jazzy/setup.bash

if [[ "${BUILD_BEFORE_RUN}" == "true" ]]; then
  echo "[run_integrated_cartesian_step_response_logged] Building mur_control..."
  colcon build --packages-select mur_control --symlink-install --allow-overriding mur_control
  echo "[run_integrated_cartesian_step_response_logged] Build finished."
  echo
fi

source_setup "${WS}/install/setup.bash"

echo "[run_integrated_cartesian_step_response_logged] Starting step response logger..."
printf '[run_integrated_cartesian_step_response_logged] full command: ros2 run mur_control log_integrated_cartesian_step_response.py'
printf ' %q' "$@"
printf '\n'
set +e
ros2 run mur_control log_integrated_cartesian_step_response.py "$@"
status="$?"
set -e

echo
echo "[run_integrated_cartesian_step_response_logged] exit_code=${status}"
echo "[run_integrated_cartesian_step_response_logged] finished=$(date --iso-8601=seconds)"
echo "[run_integrated_cartesian_step_response_logged] latest=${LATEST_LOG}"

exit "$status"
