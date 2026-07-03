#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
SRC_ROOT="$(cd "${REPO_DIR}/.." && pwd)"
DEST_ROOT="${DEST_ROOT:-/opt/mur-standalone}"
DEST_WS="${DEST_ROOT}/ws"
DEST_SRC="${DEST_WS}/src"
CONFIG_DIR="${CONFIG_DIR:-/etc/mur-standalone}"
CONFIG_FILE="${CONFIG_FILE:-${CONFIG_DIR}/mir_teleop.yaml}"
SERVICE_FILE="${SERVICE_FILE:-/etc/systemd/system/mur-mir-standalone.service}"
SERVICE_NAME="${SERVICE_NAME:-mur-mir-standalone.service}"
START_SERVICE="${START_SERVICE:-true}"
FORCE_CONFIG="${FORCE_CONFIG:-false}"

PACKAGES=(
  "match_mobile_robotics_jazzy/mur_robot/mur_mir_teleop"
  "match_mobile_robotics_jazzy/mir_robot/mir_description"
  "match_mobile_robotics_jazzy/mir_robot/mir_driver"
  "match_mobile_robotics_jazzy/mir_robot/mir_msgs"
  "match_mobile_robotics_jazzy/mir_robot/sdc21x0"
  "ira_laser_tools/src/ira_laser_tools"
)

usage() {
  cat <<EOF
Usage: $0 [--no-start] [--force-config]

Deploys the protected standalone MiR teleop runtime to ${DEST_ROOT}.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --no-start)
      START_SERVICE=false
      ;;
    --force-config)
      FORCE_CONFIG=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 2
  fi
}

require_command sudo
require_command rsync
require_command colcon

echo "[deploy_mur_standalone] source repo: ${REPO_DIR}"
echo "[deploy_mur_standalone] source root: ${SRC_ROOT}"
echo "[deploy_mur_standalone] destination: ${DEST_ROOT}"

sudo mkdir -p "${DEST_SRC}" "${DEST_ROOT}/log/ros" "${CONFIG_DIR}"

for package in "${PACKAGES[@]}"; do
  if [[ ! -d "${SRC_ROOT}/${package}" ]]; then
    echo "Package source missing: ${SRC_ROOT}/${package}" >&2
    exit 2
  fi
  sudo mkdir -p "${DEST_SRC}/$(dirname "${package}")"
  sudo rsync -a --delete \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    "${SRC_ROOT}/${package}/" \
    "${DEST_SRC}/${package}/"
done

sudo chown -R root:root "${DEST_SRC}"
sudo chmod -R a+rX "${DEST_SRC}"
sudo chown -R rosmatch:rosmatch "${DEST_ROOT}/log"

if [[ "${FORCE_CONFIG}" == "true" || ! -f "${CONFIG_FILE}" ]]; then
  sudo install -m 0644 -o root -g root \
    "${PACKAGE_DIR}/config/mir_teleop.yaml" \
    "${CONFIG_FILE}"
  echo "[deploy_mur_standalone] installed config: ${CONFIG_FILE}"
else
  sudo install -m 0644 -o root -g root \
    "${PACKAGE_DIR}/config/mir_teleop.yaml" \
    "${CONFIG_FILE}.new"
  echo "[deploy_mur_standalone] kept existing config; wrote ${CONFIG_FILE}.new"
fi

sudo install -m 0644 -o root -g root \
  "${PACKAGE_DIR}/systemd/mur-mir-standalone.service" \
  "${SERVICE_FILE}"

sudo bash -lc "
  set -Eeuo pipefail
  cd '${DEST_WS}'
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon build --symlink-install --packages-up-to sdc21x0 mur_mir_teleop
"

git_sha="$(git -C "${REPO_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
timestamp="$(date --iso-8601=seconds)"
sudo tee "${DEST_ROOT}/manifest.json" >/dev/null <<EOF
{
  "deployed_at": "${timestamp}",
  "source_repo": "${REPO_DIR}",
  "source_git_sha": "${git_sha}",
  "packages": [
    "mur_mir_teleop",
    "mir_description",
    "mir_driver",
    "mir_msgs",
    "sdc21x0",
    "ira_laser_tools"
  ],
  "config": "${CONFIG_FILE}",
  "service": "${SERVICE_FILE}"
}
EOF
sudo chmod 0644 "${DEST_ROOT}/manifest.json"

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
if [[ "${START_SERVICE}" == "true" ]]; then
  sudo systemctl restart "${SERVICE_NAME}"
fi

echo "[deploy_mur_standalone] done"
echo "[deploy_mur_standalone] logs: journalctl -u ${SERVICE_NAME} -f"
