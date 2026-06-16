#!/usr/bin/env bash
set -Eeuo pipefail

USER_NAME="${MUR_HOST_USER:-$(id -un)}"
WS="${WS:-/home/rosmatch/colcon_ws}"
REPO="${REPO:-${WS}/src/match_mobile_robotics_jazzy}"
LIMITS_FILE="${MUR_REALTIME_LIMITS_FILE:-/etc/security/limits.d/99-ros-realtime.conf}"
TTY_PORTS_DEFAULT="/dev/ttyUSB0 /dev/ttyUSB1"
TTY_PORTS="${MUR_EWELLIX_PORTS:-${TTY_PORTS_DEFAULT}}"
MODE="check"

usage() {
  cat <<EOF
Usage: $0 [--check|--apply] [--user USER]

Checks or configures host settings needed by the MuR hardware stack.

  --check       Diagnose only. Exits non-zero only for blocking issues.
  --apply       Apply group/limits setup with sudo where needed.
  --user USER   User to configure. Default: current user (${USER_NAME})

Environment:
  MUR_EWELLIX_PORTS       Space-separated serial ports to check.
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
