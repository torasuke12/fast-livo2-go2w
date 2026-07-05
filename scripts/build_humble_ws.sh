#!/usr/bin/env bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_DIR}/humble_ws"

if [ "${ROS_DISTRO:-humble}" != "humble" ]; then
  echo "This workspace is pinned to ROS 2 Humble. Current ROS_DISTRO=${ROS_DISTRO:-unset}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [ ! -d "${WS_DIR}/src/realsense-ros" ] || [ ! -d "${WS_DIR}/src/librealsense" ]; then
  "${SCRIPT_DIR}/import_realsense_sources.sh"
fi

cd "${WS_DIR}"

apt-get update
rosdep update
rosdep install -y -r \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --dependency-types build \
  --dependency-types buildtool \
  --dependency-types build_export \
  --dependency-types buildtool_export \
  --dependency-types exec \
  --skip-keys "librealsense2 dkms libgl1-mesa-dev"

colcon build \
  --symlink-install \
  --packages-up-to \
    unitree_api \
    unitree_go \
    hesai_lidar \
    go2w_imu_publisher \
    realsense2_description \
    realsense2_camera \
    fast_livo2 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_UNIT_TESTS=OFF
