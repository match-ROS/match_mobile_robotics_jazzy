#!/usr/bin/env bash
# Install persistent CAN setup for the default MuR BMS adapter (can0, 250 kbit/s).
set -Eeuo pipefail
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run with sudo: bash $0" >&2
  exit 2
fi
command -v ip >/dev/null
command -v systemctl >/dev/null
command -v udevadm >/dev/null

# Keep the service independent of the workspace and GUI rsync operations.
install -d /usr/local/sbin /etc/systemd/system /etc/udev/rules.d
helper="$(mktemp)"
trap 'rm -f "$helper"' EXIT
cat > "$helper" <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
# Do not interrupt an already active bus. Reject an unexpected bitrate.
if /usr/sbin/ip -o link show dev can0 | sed -n 's/^[^<]*<\([^>]*\)>.*/\1/p' | tr ',' '\n' | grep -qx UP; then
  if ! /usr/sbin/ip -details link show dev can0 | grep -Eq 'bitrate 250000([[:space:]]|$)'; then
    echo 'can0 is already UP with an unexpected bitrate; inspect before reconfiguring.' >&2
    exit 1
  fi
  exit 0
fi
/usr/sbin/ip link set dev can0 up type can bitrate 250000
SCRIPT
install -m 0755 "$helper" /usr/local/sbin/mur-bms-can-up

cat > /etc/systemd/system/mur-bms-can.service <<'UNIT'
[Unit]
Description=Configure MuR BMS SocketCAN (can0, 250 kbit/s)
BindsTo=sys-subsystem-net-devices-can0.device
After=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/mur-bms-can-up
RemainAfterExit=yes
UNIT

# Device activation starts the service on boot and USB reconnect. BindsTo
# deactivates it on unplug so the next connection can start it again.
cat > /etc/udev/rules.d/80-mur-bms-can.rules <<'RULE'
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can0", TAG+="systemd", ENV{SYSTEMD_WANTS}+="mur-bms-can.service"
RULE
systemctl daemon-reload
udevadm control --reload-rules
if [[ -e /sys/class/net/can0 ]]; then
  # An existing device is already active in systemd: explicitly start the unit.
  systemctl start mur-bms-can.service
  /usr/sbin/ip -details link show dev can0
else
  echo 'can0 absent; setup will run when the adapter is connected.'
fi
echo 'Persistent BMS CAN setup installed (boot and USB reconnect).'
