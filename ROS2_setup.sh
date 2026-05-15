#!/usr/bin/env bash
set -euo pipefail

# Bootstrap Ubuntu 24.04 for this ROS 2 Jazzy workspace.
# Run from anywhere:
#   bash /path/to/match_mobile_robotics_jazzy/ROS2_setup.sh
#
# Optional environment switches:
#   RUN_APT_UPGRADE=1   also run apt upgrade before installing ROS packages
#   UPDATE_BASHRC=0     do not add ROS/workspace source lines to ~/.bashrc

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
RUN_APT_UPGRADE="${RUN_APT_UPGRADE:-0}"
UPDATE_BASHRC="${UPDATE_BASHRC:-1}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_DIR}/../.." && pwd)"
SRC_DIR="${WORKSPACE_ROOT}/src"

if [[ ! -f /etc/os-release ]]; then
  echo "Cannot detect OS release." >&2
  exit 1
fi

. /etc/os-release
UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"

if [[ "${UBUNTU_CODENAME}" != "noble" ]]; then
  echo "ROS 2 Jazzy deb packages target Ubuntu 24.04 noble; detected '${UBUNTU_CODENAME}'." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  ca-certificates \
  curl \
  git \
  locales \
  software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository -y universe

ROS_APT_SOURCE_BASE_URL="https://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source"
ROS_APT_SOURCE_DEB="$(
  curl -fsSL "${ROS_APT_SOURCE_BASE_URL}/" \
    | grep -oE "ros2-apt-source_[^\"<> ]+~${UBUNTU_CODENAME}_all\\.deb" \
    | sort -V \
    | tail -n 1
)"

if [[ -z "${ROS_APT_SOURCE_DEB}" ]]; then
  echo "Could not determine latest ros2-apt-source package for ${UBUNTU_CODENAME}." >&2
  exit 1
fi

curl -fL -o /tmp/ros2-apt-source.deb \
  "${ROS_APT_SOURCE_BASE_URL}/${ROS_APT_SOURCE_DEB}"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
if [[ "${RUN_APT_UPGRADE}" == "1" ]]; then
  sudo apt upgrade -y
fi

sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-dev-tools \
  ros-${ROS_DISTRO_NAME}-desktop \
  ros-${ROS_DISTRO_NAME}-gz-ros2-control \
  ros-${ROS_DISTRO_NAME}-joy \
  ros-${ROS_DISTRO_NAME}-moveit-py \
  ros-${ROS_DISTRO_NAME}-nav2-bringup \
  ros-${ROS_DISTRO_NAME}-navigation2 \
  ros-${ROS_DISTRO_NAME}-pcl-conversions \
  ros-${ROS_DISTRO_NAME}-pcl-msgs \
  ros-${ROS_DISTRO_NAME}-pcl-ros \
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
  joystick \
  xterm

if [[ -f "${REPO_DIR}/ros2.repos" ]]; then
  mkdir -p "${SRC_DIR}"
  vcs import "${SRC_DIR}" < "${REPO_DIR}/ros2.repos"
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --rosdistro "${ROS_DISTRO_NAME}" --from-paths "${SRC_DIR}" --ignore-src -r -y

if [[ "${UPDATE_BASHRC}" == "1" ]]; then
  grep -qxF "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" "${HOME}/.bashrc" \
    || echo "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >> "${HOME}/.bashrc"
  grep -qxF "source ${WORKSPACE_ROOT}/install/setup.bash" "${HOME}/.bashrc" \
    || echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> "${HOME}/.bashrc"
fi

echo
echo "Bootstrap complete. Build with:"
echo "  cd ${WORKSPACE_ROOT}"
echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "  colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
