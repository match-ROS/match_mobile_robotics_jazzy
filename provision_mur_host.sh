#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="check"
STAGE="system"
PROFILE=""
TARGET_USER=""
BACKUP_ROOT="${MUR_PROVISION_BACKUP_ROOT:-/var/backups/mur-host-provision}"

usage() {
  cat <<'EOF'
Usage: provision_mur_host.sh --profile NAME [--check|--apply] [--stage system|display|software] [--user USER]

Stages:
  system    Configure Ubuntu, networking, PREEMPT_RT, GRUB and user limits.
            --apply must be run as root and requires a reboot afterwards.
  display   Keep the operator display awake and enable GDM automatic login.
            --apply must be run as root; log out or reboot afterwards.
  software  Install ROS 2 Jazzy and workspace dependencies using ROS2_setup.sh.
            Run only after rebooting into the realtime kernel.

Examples:
  ./provision_mur_host.sh --profile mur620b --check --stage system
  sudo ./provision_mur_host.sh --profile mur620b --apply --stage system --user rosmatch
  sudo ./provision_mur_host.sh --profile mur620b --apply --stage display --user rosmatch
  sudo ./provision_mur_host.sh --profile mur620b --apply --stage software --user rosmatch
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) MODE="check" ;;
    --apply) MODE="apply" ;;
    --stage) shift; STAGE="${1:?missing stage}" ;;
    --profile) shift; PROFILE="${1:?missing profile}" ;;
    --user) shift; TARGET_USER="${1:?missing user}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "MUR_PROVISION: status=fail issue=bad_argument value=$(printf '%q' "$1")"; usage; exit 2 ;;
  esac
  shift
done

if [[ -z "$PROFILE" || ! "$PROFILE" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo "MUR_PROVISION: status=fail issue=profile_required"
  usage
  exit 2
fi
if [[ "$STAGE" != "system" && "$STAGE" != "display" && "$STAGE" != "software" ]]; then
  echo "MUR_PROVISION: status=fail issue=bad_stage value=${STAGE}"
  exit 2
fi

PROFILE_FILE="${SCRIPT_DIR}/config/host_profiles/${PROFILE}.conf"
if [[ ! -r "$PROFILE_FILE" ]]; then
  echo "MUR_PROVISION: status=fail issue=profile_missing path=${PROFILE_FILE}"
  exit 2
fi
# shellcheck source=/dev/null
source "$PROFILE_FILE"
TARGET_USER="${TARGET_USER:-${TARGET_USER_DEFAULT}}"

failures=0
warnings=0
reboot_required=0

ok() { echo "MUR_PROVISION: status=ok issue=$1${2:+ detail=$2}"; }
warn() { echo "MUR_PROVISION: status=warn issue=$1${2:+ detail=$2}"; warnings=$((warnings + 1)); }
fail() { echo "MUR_PROVISION: status=fail issue=$1${2:+ detail=$2}"; failures=$((failures + 1)); }

require_root() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "MUR_PROVISION: status=fail issue=root_required action='rerun_with_sudo'"
    exit 2
  fi
}

profile_sanity_check() {
  local name
  for name in HOSTNAME_EXPECTED TIMEZONE_EXPECTED TARGET_USER_DEFAULT \
    MANAGEMENT_INTERFACE MANAGEMENT_CONNECTION MANAGEMENT_ADDRESS MANAGEMENT_GATEWAY \
    MANAGEMENT_DNS ROBOT_INTERFACE ROBOT_CONNECTION ROBOT_ADDRESS ROBOT_REVERSE_IP \
    MIR_ADDRESS UR_LEFT_ADDRESS UR_RIGHT_ADDRESS ROBOT_PROFILE UR_TYPE USE_LIFT; do
    if [[ -z "${!name:-}" ]]; then
      echo "MUR_PROVISION: status=fail issue=profile_value_missing key=${name}"
      exit 2
    fi
  done
}

backup_system_state() {
  local stamp backup_dir
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  backup_dir="${BACKUP_ROOT}/${HOSTNAME_EXPECTED}-${stamp}"
  install -d -m 0700 "$backup_dir"
  cp -a /etc/default/grub "$backup_dir/grub" 2>/dev/null || true
  cp -a /etc/security/limits.d "$backup_dir/limits.d" 2>/dev/null || true
  cp -a /etc/hosts "$backup_dir/hosts" 2>/dev/null || true
  cp -a /etc/gdm3/custom.conf "$backup_dir/gdm3-custom.conf" 2>/dev/null || true
  cp -a /etc/dconf/profile "$backup_dir/dconf-profile" 2>/dev/null || true
  cp -a /etc/dconf/db/local.d "$backup_dir/dconf-local.d" 2>/dev/null || true
  systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target \
    > "$backup_dir/sleep-target-state.txt" 2>&1 || true
  if command -v nmcli >/dev/null 2>&1; then
    nmcli --show-secrets connection show > "$backup_dir/nmcli-connections.txt" 2>&1 || true
  fi
  apt-mark showmanual > "$backup_dir/apt-manual.txt" 2>/dev/null || true
  dpkg-query -W > "$backup_dir/dpkg-query.txt" 2>/dev/null || true
  uname -a > "$backup_dir/uname.txt"
  echo "MUR_PROVISION: backup=${backup_dir}"
}

ensure_nm_connection() {
  local connection="$1" interface="$2" address="$3" gateway="$4" dns="$5" search="$6" current
  if nmcli -t -f NAME connection show | grep -Fxq "$connection"; then
    current="$connection"
  else
    current="$(nmcli -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)"
    if [[ -n "$current" && "$current" != "--" ]]; then
      nmcli connection modify "$current" connection.id "$connection"
      current="$connection"
    else
      nmcli connection add type ethernet ifname "$interface" con-name "$connection"
      current="$connection"
    fi
  fi
  nmcli connection modify "$current" connection.interface-name "$interface" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "$address" \
    ipv4.never-default no \
    ipv4.gateway "$gateway" \
    ipv4.dns "$dns" \
    ipv4.dns-search "$search" \
    ipv4.route-metric 100 \
    ipv6.method disabled
}

ensure_robot_connection() {
  local current
  if nmcli -t -f NAME connection show | grep -Fxq "$ROBOT_CONNECTION"; then
    current="$ROBOT_CONNECTION"
  else
    current="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_INTERFACE" 2>/dev/null || true)"
    if [[ -n "$current" && "$current" != "--" ]]; then
      nmcli connection modify "$current" connection.id "$ROBOT_CONNECTION"
      current="$ROBOT_CONNECTION"
    else
      nmcli connection add type ethernet ifname "$ROBOT_INTERFACE" con-name "$ROBOT_CONNECTION"
      current="$ROBOT_CONNECTION"
    fi
  fi
  nmcli connection modify "$current" connection.interface-name "$ROBOT_INTERFACE" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "$ROBOT_ADDRESS" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.dns-search "" \
    ipv4.never-default yes \
    ipv4.route-metric 200 \
    ipv6.method disabled
  nmcli connection up "$current" >/dev/null
}

ensure_hosts_block() {
  local tmp_file
  tmp_file="$(mktemp)"
  awk -v expected_hostname="$HOSTNAME_EXPECTED" '
    $0 == "# BEGIN MUR ROBOT NETWORK" {skip=1; next}
    $0 == "# END MUR ROBOT NETWORK" {skip=0; next}
    !skip && $1 == "127.0.1.1" {
      print "127.0.1.1 " expected_hostname
      local_hostname_written=1
      next
    }
    !skip {print}
    END {
      if (!local_hostname_written) print "127.0.1.1 " expected_hostname
    }
  ' /etc/hosts > "$tmp_file"
  cat >> "$tmp_file" <<EOF
# BEGIN MUR ROBOT NETWORK
${MIR_ADDRESS} MiR mir
${UR_LEFT_ADDRESS} UR10_l ur10_l
${UR_RIGHT_ADDRESS} UR10_r ur10_r
# END MUR ROBOT NETWORK
EOF
  install -m 0644 "$tmp_file" /etc/hosts
  rm -f "$tmp_file"
}

ensure_realtime_limits() {
  getent group dialout >/dev/null || groupadd dialout
  getent group realtime >/dev/null || groupadd realtime
  usermod -aG dialout,realtime "$TARGET_USER"
  install -m 0644 /dev/stdin /etc/security/limits.d/99-ros-realtime.conf <<'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
}

ensure_gdm_autologin() {
  local config="/etc/gdm3/custom.conf" tmp_file
  if [[ ! -f "$config" ]]; then
    echo "MUR_PROVISION: status=fail issue=gdm_config_missing path=${config}"
    exit 2
  fi

  tmp_file="$(mktemp)"
  awk -v user="$TARGET_USER" '
    function add_missing() {
      if (!enable_written) print "AutomaticLoginEnable=true"
      if (!user_written) print "AutomaticLogin=" user
    }
    /^\[[^]]+\][[:space:]]*$/ {
      if (in_daemon) add_missing()
      in_daemon = ($0 ~ /^\[daemon\][[:space:]]*$/)
      if (in_daemon) {
        daemon_seen = 1
        enable_written = 0
        user_written = 0
      }
      print
      next
    }
    in_daemon && /^[[:space:]#;]*AutomaticLoginEnable[[:space:]]*=/ {
      if (!enable_written) print "AutomaticLoginEnable=true"
      enable_written = 1
      next
    }
    in_daemon && /^[[:space:]#;]*AutomaticLogin[[:space:]]*=/ {
      if (!user_written) print "AutomaticLogin=" user
      user_written = 1
      next
    }
    { print }
    END {
      if (in_daemon) add_missing()
      if (!daemon_seen) {
        print ""
        print "[daemon]"
        print "AutomaticLoginEnable=true"
        print "AutomaticLogin=" user
      }
    }
  ' "$config" > "$tmp_file"
  install -m 0644 "$tmp_file" "$config"
  rm -f "$tmp_file"
}

ensure_operator_display() {
  ensure_gdm_autologin

  install -d -m 0755 /etc/dconf/profile /etc/dconf/db/local.d/locks
  install -m 0644 /dev/stdin /etc/dconf/profile/user <<'EOF'
user-db:user
system-db:local
EOF
  install -m 0644 /dev/stdin /etc/dconf/db/local.d/00-mur-operator-display <<'EOF'
[org/gnome/desktop/session]
idle-delay=uint32 0

[org/gnome/desktop/screensaver]
lock-enabled=false
idle-activation-enabled=false

[org/gnome/settings-daemon/plugins/power]
idle-dim=false
sleep-inactive-ac-type='nothing'
sleep-inactive-ac-timeout=0
sleep-inactive-battery-type='nothing'
sleep-inactive-battery-timeout=0
EOF
  install -m 0644 /dev/stdin /etc/dconf/db/local.d/locks/00-mur-operator-display <<'EOF'
/org/gnome/desktop/session/idle-delay
/org/gnome/desktop/screensaver/lock-enabled
/org/gnome/desktop/screensaver/idle-activation-enabled
/org/gnome/settings-daemon/plugins/power/idle-dim
/org/gnome/settings-daemon/plugins/power/sleep-inactive-ac-type
/org/gnome/settings-daemon/plugins/power/sleep-inactive-ac-timeout
/org/gnome/settings-daemon/plugins/power/sleep-inactive-battery-type
/org/gnome/settings-daemon/plugins/power/sleep-inactive-battery-timeout
EOF
  dconf update
  systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target >/dev/null
}

check_operator_display() {
  local unit sleep_targets_ok=1
  awk -F= -v user="$TARGET_USER" '
    /^\[daemon\][[:space:]]*$/ {in_daemon=1; next}
    /^\[[^]]+\][[:space:]]*$/ {in_daemon=0}
    in_daemon && $1 ~ /^[[:space:]]*AutomaticLoginEnable[[:space:]]*$/ && $2 ~ /^[[:space:]]*true[[:space:]]*$/ {enabled=1}
    in_daemon && $1 ~ /^[[:space:]]*AutomaticLogin[[:space:]]*$/ {
      value=$2; gsub(/^[[:space:]]+|[[:space:]]+$/, "", value); if (value == user) login_user=1
    }
    END {exit !(enabled && login_user)}
  ' /etc/gdm3/custom.conf 2>/dev/null \
    && ok gdm_autologin "user=${TARGET_USER}" || fail gdm_autologin "user=${TARGET_USER}"

  grep -qx 'idle-delay=uint32 0' /etc/dconf/db/local.d/00-mur-operator-display 2>/dev/null \
    && grep -qx 'lock-enabled=false' /etc/dconf/db/local.d/00-mur-operator-display 2>/dev/null \
    && grep -qx 'idle-dim=false' /etc/dconf/db/local.d/00-mur-operator-display 2>/dev/null \
    && grep -qx "sleep-inactive-ac-type='nothing'" /etc/dconf/db/local.d/00-mur-operator-display 2>/dev/null \
    && ok operator_display_awake || fail operator_display_awake missing

  for unit in sleep.target suspend.target hibernate.target hybrid-sleep.target; do
    [[ "$(systemctl is-enabled "$unit" 2>/dev/null || true)" == "masked" ]] || sleep_targets_ok=0
  done
  [[ "$sleep_targets_ok" -eq 1 ]] \
    && ok system_sleep disabled || fail system_sleep "sleep_targets_not_all_masked"
}

configure_realtime_boot() {
  local latest_rt menu_entry
  latest_rt="$(find /boot -maxdepth 1 -type f -name 'vmlinuz-*-realtime' -printf '%f\n' \
    | sed 's/^vmlinuz-//' | sort -V | tail -n 1)"
  if [[ -z "$latest_rt" ]]; then
    echo "MUR_PROVISION: status=fail issue=realtime_kernel_image_missing"
    exit 2
  fi
  if grep -q '^GRUB_DEFAULT=' /etc/default/grub; then
    sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT=saved/' /etc/default/grub
  else
    echo 'GRUB_DEFAULT=saved' >> /etc/default/grub
  fi
  if grep -q '^GRUB_SAVEDEFAULT=' /etc/default/grub; then
    sed -i 's/^GRUB_SAVEDEFAULT=.*/GRUB_SAVEDEFAULT=true/' /etc/default/grub
  else
    echo 'GRUB_SAVEDEFAULT=true' >> /etc/default/grub
  fi
  update-grub
  menu_entry="Advanced options for Ubuntu>Ubuntu, with Linux ${latest_rt}"
  grub-set-default "$menu_entry"
  echo "MUR_PROVISION: grub_default=$(printf '%q' "$menu_entry")"
}

apply_system() {
  require_root
  profile_sanity_check
  if ! id "$TARGET_USER" >/dev/null 2>&1; then
    echo "MUR_PROVISION: status=fail issue=user_missing user=${TARGET_USER}"
    exit 2
  fi
  backup_system_state
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y software-properties-common
  add-apt-repository -y universe
  apt-get update
  apt-get install -y \
    ca-certificates curl dconf-cli git linux-headers-realtime linux-realtime network-manager \
    openssh-server rt-tests
  timedatectl set-timezone "$TIMEZONE_EXPECTED"
  hostnamectl set-hostname "$HOSTNAME_EXPECTED"
  ensure_realtime_limits
  ensure_operator_display
  ensure_hosts_block
  ensure_nm_connection "$MANAGEMENT_CONNECTION" "$MANAGEMENT_INTERFACE" "$MANAGEMENT_ADDRESS" \
    "$MANAGEMENT_GATEWAY" "$MANAGEMENT_DNS" "$MANAGEMENT_DNS_SEARCH"
  ensure_robot_connection
  configure_realtime_boot
  reboot_required=1
  echo "MUR_PROVISION: applied=true stage=system reboot_required=true"
}

check_system() {
  local codename current_hostname current_timezone kernel_version kernel_build expected_ip route
  codename="$(. /etc/os-release; echo "${VERSION_CODENAME:-}")"
  [[ "$codename" == "noble" ]] && ok ubuntu_noble || fail ubuntu_noble "detected=${codename:-unknown}"
  current_hostname="$(hostname)"
  [[ "$current_hostname" == "$HOSTNAME_EXPECTED" ]] && ok hostname || fail hostname "detected=${current_hostname}"
  current_timezone="$(timedatectl show -p Timezone --value 2>/dev/null || true)"
  [[ "$current_timezone" == "$TIMEZONE_EXPECTED" ]] && ok timezone || fail timezone "detected=${current_timezone:-unknown}"
  command -v nmcli >/dev/null 2>&1 && ok network_manager || fail network_manager missing

  expected_ip="${MANAGEMENT_ADDRESS%/*}"
  ip -o -4 address show dev "$MANAGEMENT_INTERFACE" 2>/dev/null | grep -Fq " ${MANAGEMENT_ADDRESS} " \
    && ok management_address || fail management_address "expected=${MANAGEMENT_INTERFACE}:${MANAGEMENT_ADDRESS}"
  ip -o -4 address show dev "$ROBOT_INTERFACE" 2>/dev/null | grep -Fq " ${ROBOT_ADDRESS} " \
    && ok robot_address || fail robot_address "expected=${ROBOT_INTERFACE}:${ROBOT_ADDRESS}"
  route="$(ip route get "$UR_LEFT_ADDRESS" 2>/dev/null | head -n 1 || true)"
  if [[ "$route" == *"dev ${ROBOT_INTERFACE}"* && "$route" == *"src ${ROBOT_REVERSE_IP}"* ]]; then
    ok robot_route
  else
    fail robot_route "route=$(printf '%q' "${route:-missing}")"
  fi
  getent hosts UR10_l | grep -q "${UR_LEFT_ADDRESS}" && ok ur_left_hosts || fail ur_left_hosts
  getent hosts UR10_r | grep -q "${UR_RIGHT_ADDRESS}" && ok ur_right_hosts || fail ur_right_hosts

  dpkg-query -W -f='${db:Status-Abbrev}' linux-realtime 2>/dev/null | grep -q '^ii' \
    && ok realtime_package || fail realtime_package missing
  kernel_version="$(uname -r)"
  kernel_build="$(uname -v)"
  if [[ "$kernel_version" == *-realtime ]]; then
    ok realtime_kernel "$kernel_version"
  elif [[ "$reboot_required" -eq 1 ]]; then
    warn realtime_kernel "running=${kernel_version},reboot_required"
  else
    fail realtime_kernel "running=${kernel_version}"
  fi
  if [[ "$kernel_build" == *PREEMPT_RT* ]]; then
    ok preempt_rt
  elif [[ "$reboot_required" -eq 1 ]]; then
    warn preempt_rt reboot_required
  else
    fail preempt_rt "uname_v=$(printf '%q' "$kernel_build")"
  fi
  getent group realtime | awk -F: -v user="$TARGET_USER" '$1 == "realtime" {n=split($4,a,","); for (i=1;i<=n;i++) if (a[i] == user) found=1} END {exit !found}' \
    && ok realtime_group || fail realtime_group "user=${TARGET_USER}"
  id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx dialout \
    && ok dialout_group || fail dialout_group "user=${TARGET_USER}"
  grep -q '^@realtime hard rtprio 99$' /etc/security/limits.d/99-ros-realtime.conf 2>/dev/null \
    && ok realtime_limits || fail realtime_limits missing
  if [[ "$(id -un)" == "$TARGET_USER" ]]; then
    [[ "$(ulimit -r 2>/dev/null || echo 0)" == "99" ]] && ok rtprio_session || warn rtprio_session relogin_required
    [[ "$(ulimit -l 2>/dev/null || true)" == "unlimited" ]] && ok memlock_session || warn memlock_session relogin_required
  fi
  if command -v mokutil >/dev/null 2>&1 && mokutil --sb-state 2>/dev/null | grep -qi enabled; then
    warn secure_boot enabled
  else
    ok secure_boot disabled_or_unavailable
  fi
  # The short hostname normally resolves to 127.0.1.1 through /etc/hosts on
  # Ubuntu. Query the configured DNS search domain explicitly when available.
  local resolution_name="$HOSTNAME_EXPECTED"
  if [[ -n "${MANAGEMENT_DNS_SEARCH:-}" ]]; then
    resolution_name="${HOSTNAME_EXPECTED}.${MANAGEMENT_DNS_SEARCH%.}"
  fi
  getent ahostsv4 "$resolution_name" | awk '{print $1}' | grep -qx "$expected_ip" \
    && ok hostname_resolution || warn hostname_resolution "name=${resolution_name} expected=${expected_ip}"
  check_operator_display
}

apply_display() {
  require_root
  profile_sanity_check
  if ! id "$TARGET_USER" >/dev/null 2>&1; then
    echo "MUR_PROVISION: status=fail issue=user_missing user=${TARGET_USER}"
    exit 2
  fi
  backup_system_state
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y dconf-cli
  ensure_operator_display
  echo "MUR_PROVISION: applied=true stage=display relogin_or_reboot_required=true"
}

apply_software() {
  require_root
  if [[ "$(uname -r)" != *-realtime || "$(uname -v)" != *PREEMPT_RT* ]]; then
    echo "MUR_PROVISION: status=fail issue=realtime_reboot_required running=$(uname -r)"
    exit 2
  fi
  TARGET_USER="$TARGET_USER" UPDATE_BASHRC=1 "$SCRIPT_DIR/ROS2_setup.sh"
}

check_software() {
  [[ -f /opt/ros/jazzy/setup.bash ]] && ok ros_jazzy || fail ros_jazzy missing
  command -v colcon >/dev/null 2>&1 && ok colcon || fail colcon missing
  command -v rosdep >/dev/null 2>&1 && ok rosdep || fail rosdep missing
  command -v vcs >/dev/null 2>&1 && ok vcstool || fail vcstool missing
  python3 -c 'import can' >/dev/null 2>&1 && ok python_can || fail python_can missing
  [[ -f /home/"$TARGET_USER"/colcon_ws/install/setup.bash ]] \
    && ok workspace_install || warn workspace_install build_required
}

echo "MUR_PROVISION: mode=${MODE} stage=${STAGE} profile=${PROFILE} user=${TARGET_USER} host=$(hostname)"
profile_sanity_check
if [[ "$MODE" == "apply" && "$STAGE" == "system" ]]; then apply_system; fi
if [[ "$MODE" == "apply" && "$STAGE" == "display" ]]; then apply_display; fi
if [[ "$MODE" == "apply" && "$STAGE" == "software" ]]; then apply_software; fi
if [[ "$STAGE" == "system" ]]; then
  check_system
elif [[ "$STAGE" == "display" ]]; then
  check_operator_display
else
  check_software
fi

if [[ "$failures" -gt 0 ]]; then
  echo "MUR_PROVISION: summary=fail failures=${failures} warnings=${warnings} reboot_required=${reboot_required}"
  exit 2
fi
echo "MUR_PROVISION: summary=$([[ "$warnings" -gt 0 ]] && echo warn || echo ok) failures=0 warnings=${warnings} reboot_required=${reboot_required}"
