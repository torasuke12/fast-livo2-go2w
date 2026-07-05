#!/usr/bin/env bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_DIR}/humble_ws"

required_package_specs=(
  "fast_livo2:src/fast-livo2/package.xml"
  "hesai_lidar:src/go2w-hesai-lidar-driver/package.xml"
  "go2w_imu_publisher:src/go2w-imu-publisher/package.xml"
  "unitree_api:src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api/package.xml"
  "unitree_go:src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/package.xml"
  "realsense2_description:src/realsense-ros/realsense2_description/package.xml"
  "realsense2_camera:src/realsense-ros/realsense2_camera/package.xml"
)

list_missing_packages() {
  local missing=()
  local spec package_name package_file

  for spec in "${required_package_specs[@]}"; do
    package_name="${spec%%:*}"
    package_file="${spec#*:}"

    if [ ! -f "${WS_DIR}/${package_file}" ]; then
      missing+=("${package_name}")
    fi
  done

  if [ "${#missing[@]}" -gt 0 ]; then
    printf '%s\n' "${missing[@]}"
  fi
}

ensure_package_sources() {
  local missing_packages=()

  mapfile -t missing_packages < <(list_missing_packages)
  if [ "${#missing_packages[@]}" -eq 0 ]; then
    return
  fi

  echo "Missing ROS package sources: ${missing_packages[*]}" >&2

  if [ ! -e "${PROJECT_DIR}/.git" ] || [ ! -f "${PROJECT_DIR}/.gitmodules" ] || ! command -v git >/dev/null 2>&1; then
    echo "Initialize submodules before building:" >&2
    echo "git submodule update --init --recursive" >&2
    exit 1
  fi

  echo "Initializing git submodules..." >&2
  git -C "${PROJECT_DIR}" config --global --add safe.directory "${PROJECT_DIR}" || true
  if ! git -C "${PROJECT_DIR}" submodule update --init --recursive \
    humble_ws/src/fast-livo2 \
    humble_ws/src/go2w-hesai-lidar-driver \
    humble_ws/src/go2w-imu-publisher \
    humble_ws/src/unitree_ros2 \
    humble_ws/src/realsense-ros \
    humble_ws/src/librealsense; then
    echo "Failed to initialize submodules. Check network access and git credentials." >&2
    exit 1
  fi

  mapfile -t missing_packages < <(list_missing_packages)
  if [ "${#missing_packages[@]}" -gt 0 ]; then
    echo "Still missing ROS package sources after submodule initialization: ${missing_packages[*]}" >&2
    exit 1
  fi
}

if [ "${ROS_DISTRO:-humble}" != "humble" ]; then
  echo "This workspace is pinned to ROS 2 Humble. Current ROS_DISTRO=${ROS_DISTRO:-unset}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

ensure_package_sources

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

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "colcon build finished without creating ${WS_DIR}/install/setup.bash" >&2
  echo "Check the latest build log: ${WS_DIR}/log/latest_build/logger_all.log" >&2
  exit 1
fi

echo "Build finished. Source the workspace with:"
echo "source ${WS_DIR}/install/setup.bash"
