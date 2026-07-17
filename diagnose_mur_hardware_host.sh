#!/usr/bin/env bash
set -Eeuo pipefail

WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LOG_FILE="${1:-${REPO}/logs/hardware/latest.log}"
SETUP_SCRIPT="${REPO}/setup_mur_hardware_host.sh"

echo "MUR_DIAG: host=$(hostname) user=$(id -un) date=$(date --iso-8601=seconds)"
echo "MUR_DIAG: ws=${WS}"
echo "MUR_DIAG: log=${LOG_FILE}"
if [[ -z "${MUR_CHECK_UR_NETWORK:-}" ]]; then
  export MUR_CHECK_UR_NETWORK=true
fi
if [[ -z "${MUR_EXPECTED_REVERSE_IP:-}" && "$(hostname)" == "mur620d" ]]; then
  export MUR_EXPECTED_REVERSE_IP=192.168.12.69
fi
echo "MUR_DIAG: ur_network_check=${MUR_CHECK_UR_NETWORK} hosts=${MUR_UR_HOSTS:-UR10_l UR10_r} expected_reverse_ip=${MUR_EXPECTED_REVERSE_IP:-unset}"
echo

setup_rc=0
if [[ -x "$SETUP_SCRIPT" ]]; then
  "$SETUP_SCRIPT" --check || setup_rc=$?
else
  echo "MUR_DIAG: status=warn issue=setup_script_missing path=${SETUP_SCRIPT}"
fi

echo
echo "MUR_DIAG: groups=$(id -nG)"
for port in /dev/ttyUSB0 /dev/ttyUSB1; do
  if [[ -e "$port" ]]; then
    ls -l "$port" | sed 's/^/MUR_DIAG: device /'
  else
    echo "MUR_DIAG: device_missing ${port}"
  fi
done
echo "MUR_DIAG: ulimit_rtprio=$(ulimit -r 2>/dev/null || echo unknown)"
echo "MUR_DIAG: ulimit_memlock=$(ulimit -l 2>/dev/null || echo unknown)"
python3 -c 'import can; print("MUR_DIAG: python_can=ok version=" + getattr(can, "__version__", "unknown"))' 2>/dev/null \
  || echo "MUR_DIAG: python_can=missing_ignored_for_now"

if [[ ! -e "$LOG_FILE" ]]; then
  echo
  echo "MUR_DIAG: status=warn issue=hardware_log_missing path=${LOG_FILE}"
  exit 0
fi

echo
echo "MUR_DIAG: hardware_log_size=$(wc -c < "$LOG_FILE")"
echo "MUR_DIAG: hardware_log_mtime=$(date --iso-8601=seconds -r "$LOG_FILE")"

serial_count=0
ur_estop_count=0
ur_protective_count=0
ur_reverse_count=0
realtime_count=0
bms_count=0
octomap_count=0
ewellix_state_count=0

if grep -Eqi 'Permission denied|not in group dialout|not readable/writable|Failed to open port communication' "$LOG_FILE"; then
  serial_count="$(grep -Ein 'Permission denied|not in group dialout|not readable/writable|Failed to open port communication' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=error type=serial_permission count=${serial_count}"
  grep -Ein 'Permission denied|not in group dialout|not readable/writable|Failed to open port communication' "$LOG_FILE" | tail -n 8 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'ROBOT_EMERGENCY_STOP|EM-Stop|SetMode goal was rejected|UR SetMode failed|Transition to target mode failed' "$LOG_FILE"; then
  ur_estop_count="$(grep -Ein 'ROBOT_EMERGENCY_STOP|EM-Stop|SetMode goal was rejected|UR SetMode failed|Transition to target mode failed' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=error type=ur_emergency_stop_or_setmode count=${ur_estop_count}"
  echo "MUR_DIAG_DIAGNOSIS: severity=error problem='Robot in Emergency Stop or UR SetMode rejected' action='Release EM-stop/safety condition on pendant, then rerun Check Host.'"
  grep -Ein 'ROBOT_EMERGENCY_STOP|EM-Stop|SetMode goal was rejected|UR SetMode failed|Transition to target mode failed' "$LOG_FILE" | tail -n 10 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'PROTECTIVE_STOP|C161A0|Dashboard play failed|Failed to execute: play|UR program is not running yet' "$LOG_FILE"; then
  ur_protective_count="$(grep -Ein 'PROTECTIVE_STOP|C161A0|Dashboard play failed|Failed to execute: play|UR program is not running yet' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=error type=ur_protective_stop_or_play_failed count=${ur_protective_count}"
  echo "MUR_DIAG_DIAGNOSIS: severity=error problem='Robot in Protective Stop or Dashboard play failed' action='Check the UR pendant for Protective Stop/Safety popup; unlock/close it and ensure Remote Control is enabled.'"
  grep -Ein 'PROTECTIVE_STOP|C161A0|Dashboard play failed|Failed to execute: play|UR program is not running yet' "$LOG_FILE" | tail -n 12 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'Receive Program Failed|Connection to reverse interface dropped|Robot requested program|Robot connected to reverse interface|Failed to read from stream' "$LOG_FILE"; then
  ur_reverse_count="$(grep -Ein 'Receive Program Failed|Connection to reverse interface dropped|Robot requested program|Robot connected to reverse interface|Failed to read from stream' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=warn type=ur_reverse_interface count=${ur_reverse_count}"
  grep -Ein 'Receive Program Failed|Connection to reverse interface dropped|Robot requested program|Robot connected to reverse interface|Failed to read from stream' "$LOG_FILE" | tail -n 16 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'Could not enable FIFO RT scheduling|overruns:|missed its desired rate' "$LOG_FILE"; then
  realtime_count="$(grep -Ein 'Could not enable FIFO RT scheduling|overruns:|missed its desired rate' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=warn type=realtime_missing_or_overrun count=${realtime_count}"
  grep -Ein 'Could not enable FIFO RT scheduling|overruns:|missed its desired rate' "$LOG_FILE" | tail -n 10 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'python-can is not installed|No module named .can.|bms_can_node|Failed to send BMS request|Network is down' "$LOG_FILE"; then
  bms_count="$(grep -Ein 'python-can is not installed|No module named .can.|bms_can_node|Failed to send BMS request|Network is down' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=info type=bms_can_ignored count=${bms_count}"
  grep -Ein 'python-can is not installed|No module named .can.|bms_can_node|Failed to send BMS request|Network is down' "$LOG_FILE" | tail -n 8 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if grep -Eqi 'No 3D sensor plugin.*octomap' "$LOG_FILE"; then
  octomap_count="$(grep -Ein 'No 3D sensor plugin.*octomap' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=info type=moveit_octomap count=${octomap_count}"
fi

if grep -Eqi 'No Ewellix state received|Missing left_lift_joint|Missing right_lift_joint' "$LOG_FILE"; then
  ewellix_state_count="$(grep -Ein 'No Ewellix state received|Missing left_lift_joint|Missing right_lift_joint' "$LOG_FILE" | wc -l)"
  echo "MUR_DIAG_ISSUE: severity=warn type=lift_state_missing count=${ewellix_state_count}"
  grep -Ein 'No Ewellix state received|Missing left_lift_joint|Missing right_lift_joint' "$LOG_FILE" | tail -n 8 | sed 's/^/MUR_DIAG_DETAIL: /'
fi

if [[ "$serial_count" -eq 0 && "$ur_estop_count" -eq 0 && "$ur_protective_count" -eq 0 && "$ur_reverse_count" -eq 0 && "$realtime_count" -eq 0 && "$bms_count" -eq 0 && "$octomap_count" -eq 0 && "$ewellix_state_count" -eq 0 ]]; then
  echo "MUR_DIAG: summary=no_known_issues_found"
else
  echo "MUR_DIAG: summary=known_issues_found serial=${serial_count} ur=${ur_estop_count} ur_protective=${ur_protective_count} ur_reverse=${ur_reverse_count} realtime=${realtime_count} bms_ignored=${bms_count} octomap_info=${octomap_count} lift_state=${ewellix_state_count}"
fi

if [[ "$setup_rc" -ne 0 ]]; then
  echo "MUR_DIAG: result=fail reason=host_check_blocking setup_exit=${setup_rc}"
  exit "$setup_rc"
fi

echo "MUR_DIAG: result=ok reason=no_blocking_host_check"
