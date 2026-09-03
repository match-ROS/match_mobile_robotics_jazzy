#!/usr/bin/env bash
set -Eeuo pipefail

USER_NAME="${MUR_HOST_USER:-$(id -un)}"
WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LIMITS_FILE="${MUR_REALTIME_LIMITS_FILE:-/etc/security/limits.d/99-ros-realtime.conf}"
TTY_PORTS_DEFAULT="/dev/ttyUSB0 /dev/ttyUSB1"
TTY_PORTS="${MUR_EWELLIX_PORTS:-${TTY_PORTS_DEFAULT}}"
CHECK_UR_NETWORK="${MUR_CHECK_UR_NETWORK:-false}"
UR_HOSTS="${MUR_UR_HOSTS:-UR10_l UR10_r}"
UR_DASHBOARD_PORT="${MUR_UR_DASHBOARD_PORT:-29999}"
EXPECTED_REVERSE_IP="${MUR_EXPECTED_REVERSE_IP:-}"
MODE="check"
PROFILE=""

usage() {
  cat <<EOF
Usage: $0 [--check|--apply] [--user USER] [--profile NAME]

Checks or configures host settings needed by the MuR hardware stack.

  --check       Diagnose only. Exits non-zero only for blocking issues.
  --apply       Apply group/limits setup with sudo where needed.
  --user USER   User to configure. Default: current user (${USER_NAME})
  --profile NAME Load robot-specific network and lift defaults.

Environment:
  MUR_EWELLIX_PORTS       Space-separated serial ports to check.
  MUR_CHECK_UR_NETWORK    true to require UR dashboard reachability.
  MUR_UR_HOSTS            Space-separated UR hosts/IPs to check.
  MUR_EXPECTED_REVERSE_IP Optional expected local source IP for UR reverse interface.
  MUR_REALTIME_LIMITS_FILE Limits file path.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)
      MODE="check"
      ;;
    --apply)
      MODE="apply"
      ;;
    --user)
      shift
      USER_NAME="${1:?missing value for --user}"
      ;;
    --profile)
      shift
      PROFILE="${1:?missing value for --profile}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "MUR_HOST_CHECK: status=fail issue=bad_argument detail=$(printf '%q' "$1")"
      usage
      exit 2
      ;;
  esac
  shift
done

if [[ -n "$PROFILE" ]]; then
  if [[ ! "$PROFILE" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "MUR_HOST_CHECK: status=fail issue=bad_profile value=${PROFILE}"
    exit 2
  fi
  PROFILE_FILE="${REPO}/config/host_profiles/${PROFILE}.conf"
  if [[ ! -r "$PROFILE_FILE" ]]; then
    echo "MUR_HOST_CHECK: status=fail issue=profile_missing path=${PROFILE_FILE}"
    exit 2
  fi
  # shellcheck source=/dev/null
  source "$PROFILE_FILE"
  EXPECTED_REVERSE_IP="${MUR_EXPECTED_REVERSE_IP:-${ROBOT_REVERSE_IP}}"
  UR_HOSTS="${MUR_UR_HOSTS:-UR10_l UR10_r}"
  if [[ "${USE_LIFT}" == "false" && -z "${MUR_EWELLIX_PORTS+x}" ]]; then
    TTY_PORTS=""
  fi
fi

require_command() {
  local command_name="$1"
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "MUR_HOST_CHECK: status=fail issue=missing_command command=${command_name}"
    return 1
  fi
  return 0
}

has_group() {
  local user="$1"
  local group="$2"
  id -nG "$user" 2>/dev/null | tr ' ' '\n' | grep -qx "$group"
}

sudo_cmd() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
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

check_ur_dashboard_state() {
  local host="$1"
  local ip="$2"
  local port="$3"
  local output rc
  set +e
  output="$(python3 - "$host" "$ip" "$port" <<'PYDASH'
import socket
import sys

host, ip, port_s = sys.argv[1:4]
port = int(port_s)
timeout = 2.0
commands = [
    "robotmode",
    "safetystatus",
    "safetymode",
    "programState",
    "is in remote control",
    "get loaded program",
]
blocking_safety_tokens = (
    "PROTECTIVE_STOP",
    "ROBOT_EMERGENCY_STOP",
    "SYSTEM_EMERGENCY_STOP",
    "SAFEGUARD_STOP",
    "FAULT",
    "VIOLATION",
    "RECOVERY",
)


def emit(line):
    print(line, flush=True)


def quote_value(value):
    return str(value).replace("\\", "\\\\").replace("'", "_")

try:
    sock = socket.create_connection((ip, port), timeout=timeout)
    sock.settimeout(timeout)
    file = sock.makefile("rwb", buffering=0)
    banner = file.readline().decode("utf-8", errors="replace").strip()
except OSError as exc:
    emit(
        "MUR_HOST_CHECK: status=fail issue=ur_network "
        f"host={host} ip={ip} port={port} detail=dashboard_unreachable error='{quote_value(exc)}'"
    )
    emit(
        "MUR_HOST_CHECK_DIAGNOSIS: severity=error "
        f"host={host} problem='UR dashboard unreachable' "
        "action='Check robot power, network cable/IP/name resolution, and that PolyScope is booted.'"
    )
    sys.exit(3)

answers = {}
try:
    emit(f"MUR_HOST_CHECK: status=ok issue=ur_network host={host} ip={ip} port={port}")
    if banner:
        emit(f"MUR_HOST_CHECK: dashboard host={host} banner='{quote_value(banner)}'")
    for command in commands:
        file.write((command + "\n").encode("utf-8"))
        answer = file.readline().decode("utf-8", errors="replace").strip()
        answers[command] = answer
        key = command.replace(" ", "_")
        emit(f"MUR_HOST_CHECK: dashboard host={host} {key}='{quote_value(answer)}'")
finally:
    try:
        file.close()
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass

text = "\n".join(answers.values()).upper()
blocking = False
if any(token in text for token in blocking_safety_tokens):
    blocking = True
    emit(f"MUR_HOST_CHECK: status=fail issue=ur_safety host={host} detail=protective_or_safety_stop")
    emit(
        "MUR_HOST_CHECK_DIAGNOSIS: severity=error "
        f"host={host} problem='Robot in Protective Stop or Safety Stop' "
        "action='Inspect the robot cell, release the stop on the pendant, then unlock/close safety popup before starting the driver.'"
    )

remote = answers.get("is in remote control", "").strip().lower()
if remote in ("false", "remote control: false", "is in remote control: false"):
    blocking = True
    emit(f"MUR_HOST_CHECK: status=fail issue=ur_remote_control host={host} detail=robot_not_in_remote_control_mode")
    emit(
        "MUR_HOST_CHECK_DIAGNOSIS: severity=error "
        f"host={host} problem='Robot not in remote control mode' "
        "action='On the UR pendant switch PolyScope/Operational Mode to Remote Control, then rerun Check Host.'"
    )
elif "unknown command" in remote or "not supported" in remote:
    emit(f"MUR_HOST_CHECK: status=warn issue=ur_remote_control host={host} detail=dashboard_command_unsupported")
elif remote in ("true", "remote control: true", "is in remote control: true"):
    emit(f"MUR_HOST_CHECK: status=ok issue=ur_remote_control host={host}")

robotmode = answers.get("robotmode", "")
if robotmode and not any(state in robotmode.upper() for state in ("RUNNING", "IDLE")):
    blocking = True
    emit(f"MUR_HOST_CHECK: status=fail issue=ur_robotmode host={host} detail='{quote_value(robotmode)}'")
    emit(
        "MUR_HOST_CHECK_DIAGNOSIS: severity=error "
        f"host={host} problem='Robot mode is not RUNNING/IDLE' "
        "action='Power on and brake-release the UR on the pendant before starting the driver.'"
    )

if not blocking:
    emit(f"MUR_HOST_CHECK: status=ok issue=ur_dashboard_state host={host}")
sys.exit(4 if blocking else 0)
PYDASH
)"
  rc=$?
  set -e
  if [[ -n "$output" ]]; then
    printf '%s\n' "$output"
  fi
  return "$rc"
}

apply_setup() {
  if ! getent group dialout >/dev/null; then
    echo "MUR_HOST_SETUP: creating group dialout"
    sudo_cmd groupadd dialout
  fi
  if ! has_group "$USER_NAME" dialout; then
    echo "MUR_HOST_SETUP: adding ${USER_NAME} to dialout"
    sudo_cmd usermod -a -G dialout "$USER_NAME"
  fi

  if ! getent group realtime >/dev/null; then
    echo "MUR_HOST_SETUP: creating group realtime"
    sudo_cmd groupadd realtime
  fi
  if ! has_group "$USER_NAME" realtime; then
    echo "MUR_HOST_SETUP: adding ${USER_NAME} to realtime"
    sudo_cmd usermod -a -G realtime "$USER_NAME"
  fi

  local tmp_file
  tmp_file="$(mktemp)"
  cat > "$tmp_file" <<'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
  echo "MUR_HOST_SETUP: installing ${LIMITS_FILE}"
  sudo_cmd install -m 0644 "$tmp_file" "$LIMITS_FILE"
  rm -f "$tmp_file"
}

blocking=0
warnings=0

echo "MUR_HOST_CHECK: mode=${MODE} user=${USER_NAME} host=$(hostname) ws=${WS}"

if [[ "$MODE" == "apply" ]]; then
  require_command sudo || exit 2
  apply_setup
  echo "MUR_HOST_SETUP: applied=true"
  echo "MUR_HOST_SETUP: action_required=log_out_and_back_in_or_reboot_before_rechecking"
fi

if ! id "$USER_NAME" >/dev/null 2>&1; then
  echo "MUR_HOST_CHECK: status=fail issue=user_missing user=${USER_NAME}"
  exit 2
fi

if has_group "$USER_NAME" dialout; then
  echo "MUR_HOST_CHECK: status=ok issue=dialout user=${USER_NAME}"
else
  echo "MUR_HOST_CHECK: status=fail issue=dialout user=${USER_NAME} detail=not_in_group"
  blocking=1
fi

for port in ${TTY_PORTS}; do
  if [[ -e "$port" ]]; then
    ls -l "$port" | sed 's/^/MUR_HOST_CHECK: device /'
    if [[ -r "$port" && -w "$port" ]]; then
      echo "MUR_HOST_CHECK: status=ok issue=serial_permission port=${port}"
    else
      echo "MUR_HOST_CHECK: status=fail issue=serial_permission port=${port} detail=not_readable_or_writable"
      blocking=1
    fi
  else
    echo "MUR_HOST_CHECK: status=warn issue=serial_missing port=${port}"
    warnings=1
  fi
done

if getent group realtime >/dev/null && has_group "$USER_NAME" realtime; then
  echo "MUR_HOST_CHECK: status=ok issue=realtime_group user=${USER_NAME}"
else
  echo "MUR_HOST_CHECK: status=warn issue=realtime_group user=${USER_NAME} detail=missing_group_or_membership"
  warnings=1
fi

rtprio="$(ulimit -r 2>/dev/null || echo unknown)"
memlock="$(ulimit -l 2>/dev/null || echo unknown)"
echo "MUR_HOST_CHECK: ulimit rtprio=${rtprio} memlock=${memlock}"
if [[ "$rtprio" == "0" || "$rtprio" == "unknown" ]]; then
  echo "MUR_HOST_CHECK: status=warn issue=realtime_ulimit detail=rtprio_${rtprio}"
  warnings=1
else
  echo "MUR_HOST_CHECK: status=ok issue=realtime_ulimit"
fi

if [[ "$CHECK_UR_NETWORK" == "true" ]]; then
  for host in ${UR_HOSTS}; do
    resolved="$(getent hosts "$host" | awk '{print $1}' | head -n 1 || true)"
    if [[ -z "$resolved" ]]; then
      echo "MUR_HOST_CHECK: status=fail issue=ur_network host=${host} detail=name_resolution_failed"
      blocking=1
      continue
    fi
    route="$(ip route get "$resolved" 2>/dev/null | head -n 1 || true)"
    if [[ -z "$route" ]]; then
      echo "MUR_HOST_CHECK: status=fail issue=ur_network host=${host} ip=${resolved} detail=no_route"
      blocking=1
      continue
    fi
    echo "MUR_HOST_CHECK: route host=${host} ip=${resolved} ${route}"
    route_src="$(printf '%s\n' "$route" | route_src_ip)"
    if [[ -z "$route_src" ]]; then
      echo "MUR_HOST_CHECK: status=fail issue=ur_reverse_route host=${host} ip=${resolved} detail=missing_route_src"
      blocking=1
    elif [[ -n "$EXPECTED_REVERSE_IP" && "$route_src" != "$EXPECTED_REVERSE_IP" ]]; then
      echo "MUR_HOST_CHECK: status=fail issue=ur_reverse_route host=${host} ip=${resolved} src=${route_src} expected=${EXPECTED_REVERSE_IP}"
      blocking=1
    else
      echo "MUR_HOST_CHECK: status=ok issue=ur_reverse_route host=${host} ip=${resolved} src=${route_src}"
    fi
    if ! check_ur_dashboard_state "$host" "$resolved" "$UR_DASHBOARD_PORT"; then
      blocking=1
    fi
  done
else
  echo "MUR_HOST_CHECK: status=skip issue=ur_network detail=MUR_CHECK_UR_NETWORK_not_true"
fi

if python3 -c 'import can' >/dev/null 2>&1; then
  version="$(python3 -c 'import can; print(getattr(can, "__version__", "unknown"))' 2>/dev/null || true)"
  echo "MUR_HOST_CHECK: status=ok issue=python_can version=${version:-unknown}"
else
  echo "MUR_HOST_CHECK: status=warn issue=python_can detail=missing_ignored_for_now"
  warnings=1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  echo "MUR_HOST_CHECK: status=ok issue=ros_jazzy"
else
  echo "MUR_HOST_CHECK: status=fail issue=ros_jazzy detail=/opt/ros/jazzy/setup.bash_missing"
  blocking=1
fi

if [[ -f "${WS}/install/setup.bash" ]]; then
  echo "MUR_HOST_CHECK: status=ok issue=workspace_install"
else
  echo "MUR_HOST_CHECK: status=warn issue=workspace_install detail=${WS}/install/setup.bash_missing_build_needed"
  warnings=1
fi

if [[ "$blocking" -ne 0 ]]; then
  echo "MUR_HOST_CHECK: summary=fail blocking=${blocking} warnings=${warnings}"
  exit 2
fi
if [[ "$warnings" -ne 0 ]]; then
  echo "MUR_HOST_CHECK: summary=warn blocking=0 warnings=${warnings}"
  exit 0
fi
echo "MUR_HOST_CHECK: summary=ok blocking=0 warnings=0"
