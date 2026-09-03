#!/usr/bin/env bash
set -Eeuo pipefail

# Idempotent Ubuntu 24.04 / ROS 2 Jazzy bootstrap for the MATCH workspace.
# Optional environment:
#   TARGET_USER=rosmatch    owner of the workspace and shell configuration
#   WORKSPACE_ROOT=/path    defaults to two directories above this repository
#   RUN_APT_UPGRADE=1       upgrade existing packages before installing ROS
#   UPDATE_BASHRC=0         do not add source lines to the target user's bashrc

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
RUN_APT_UPGRADE="${RUN_APT_UPGRADE:-0}"
UPDATE_BASHRC="${UPDATE_BASHRC:-1}"
TARGET_USER="${TARGET_USER:-${SUDO_USER:-$(id -un)}}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${REPO_DIR}/../.." && pwd)}"
SRC_DIR="${WORKSPACE_ROOT}/src"

if ! id "$TARGET_USER" >/dev/null 2>&1; then
  echo "Target user '${TARGET_USER}' does not exist." >&2
  exit 1
fi
TARGET_GROUP="$(id -gn "$TARGET_USER")"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

sudo_cmd() {
  if [[ "$(id -u)" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

as_target_user() {
  if [[ "$(id -un)" == "$TARGET_USER" ]]; then "$@"; else runuser -u "$TARGET_USER" -- "$@"; fi
}

if [[ ! -f /etc/os-release ]]; then
  echo "Cannot detect OS release." >&2
  exit 1
fi
. /etc/os-release
UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
if [[ "$UBUNTU_CODENAME" != "noble" ]]; then
  echo "ROS 2 Jazzy deb packages require Ubuntu 24.04 noble; detected '${UBUNTU_CODENAME}'." >&2
  exit 1
fi

sudo_cmd apt-get update
sudo_cmd apt-get install -y ca-certificates curl git locales software-properties-common
sudo_cmd locale-gen en_US en_US.UTF-8
sudo_cmd update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo_cmd add-apt-repository -y universe

ROS_APT_SOURCE_BASE_URL="https://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source"
ROS_APT_SOURCE_DEB="$(
  curl -fsSL "${ROS_APT_SOURCE_BASE_URL}/" \
    | grep -oE "ros2-apt-source_[^\"<> ]+~${UBUNTU_CODENAME}_all\\.deb" \
    | sort -V | tail -n 1
)"
if [[ -z "$ROS_APT_SOURCE_DEB" ]]; then
  echo "Could not determine the latest ros2-apt-source package for ${UBUNTU_CODENAME}." >&2
  exit 1
fi
curl -fL -o /tmp/ros2-apt-source.deb "${ROS_APT_SOURCE_BASE_URL}/${ROS_APT_SOURCE_DEB}"
sudo_cmd dpkg -i /tmp/ros2-apt-source.deb

sudo_cmd apt-get update
if [[ "$RUN_APT_UPGRADE" == "1" ]]; then sudo_cmd apt-get upgrade -y; fi
sudo_cmd apt-get install -y \
  build-essential \
  can-utils \
  cmake \
  joystick \
  python3-can \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  ros-dev-tools \
  ros-${ROS_DISTRO_NAME}-depthai-v3 \
  ros-${ROS_DISTRO_NAME}-desktop \
  ros-${ROS_DISTRO_NAME}-gz-ros2-control \
  ros-${ROS_DISTRO_NAME}-joy \
  ros-${ROS_DISTRO_NAME}-moveit-py \
  ros-${ROS_DISTRO_NAME}-nav2-bringup \
  ros-${ROS_DISTRO_NAME}-navigation2 \
  ros-${ROS_DISTRO_NAME}-pcl-conversions \
  ros-${ROS_DISTRO_NAME}-pcl-msgs \
  ros-${ROS_DISTRO_NAME}-pcl-ros \
  ros-${ROS_DISTRO_NAME}-plotjuggler-ros \
  ros-${ROS_DISTRO_NAME}-robot-localization \
  ros-${ROS_DISTRO_NAME}-ros-gz-bridge \
  ros-${ROS_DISTRO_NAME}-ros-gz-sim \
  ros-${ROS_DISTRO_NAME}-ros2-control \
  ros-${ROS_DISTRO_NAME}-ros2-controllers \
  ros-${ROS_DISTRO_NAME}-rqt-controller-manager \
  ros-${ROS_DISTRO_NAME}-rqt-robot-steering \
  ros-${ROS_DISTRO_NAME}-slam-toolbox \
  ros-${ROS_DISTRO_NAME}-srdfdom \
  ros-${ROS_DISTRO_NAME}-teleop-twist-keyboard \
  xterm

sudo_cmd install -d -o "$TARGET_USER" -g "$TARGET_GROUP" "$WORKSPACE_ROOT" "$SRC_DIR"
if [[ -f "${REPO_DIR}/workspace.repos" ]]; then
  as_target_user vcs import --skip-existing "$SRC_DIR" < "${REPO_DIR}/workspace.repos"
elif [[ -f "${REPO_DIR}/ros2.repos" ]]; then
  as_target_user vcs import --skip-existing "$SRC_DIR" < "${REPO_DIR}/ros2.repos"
fi
if [[ -f "${REPO_DIR}/.gitmodules" ]]; then
  as_target_user git -C "$REPO_DIR" submodule update --init --recursive
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo_cmd rosdep init; fi
as_target_user rosdep update

# Keep the ROS 1 reference repositories in src, but never feed their catkin
# manifests to a Jazzy rosdep run. COLCON_IGNORE is honored by colcon, whereas
# rosdep recursively scans every path explicitly passed to it.
ROSDEP_PATHS=(
  "$SRC_DIR/agentic_vision"
  "$SRC_DIR/ira_laser_tools"
  "$SRC_DIR/match_cooperative_handling"
  "$SRC_DIR/match_mobile_robotics_jazzy"
  "$SRC_DIR/match_mur_gui"
  "$SRC_DIR/oak_camera_calibration"
)
sudo_cmd rosdep install --rosdistro "$ROS_DISTRO_NAME" --from-paths "${ROSDEP_PATHS[@]}" \
  --ignore-src -r -y --skip-keys ament_python

if [[ "$UPDATE_BASHRC" == "1" ]]; then
  touch "$TARGET_HOME/.bashrc"
  grep -qxF "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" "$TARGET_HOME/.bashrc" \
    || echo "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >> "$TARGET_HOME/.bashrc"
  grep -qxF "source ${WORKSPACE_ROOT}/install/setup.bash" "$TARGET_HOME/.bashrc" \
    || echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> "$TARGET_HOME/.bashrc"
  sudo_cmd chown "$TARGET_USER:$TARGET_GROUP" "$TARGET_HOME/.bashrc"
fi

echo
echo "Bootstrap complete. Build with:"
echo "  cd ${WORKSPACE_ROOT}"
echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "  colcon build --symlink-install --event-handlers console_direct+ \\\n+    --metas ${REPO_DIR}/colcon.meta \\\n+    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
