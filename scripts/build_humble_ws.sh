#!/usr/bin/env bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_DIR}/humble_ws"

required_package_specs=(
  "fast_livo2:src/fast-livo2/package.xml"
  "livox_ros_driver2:src/livox_ros_driver2/package.xml"
  "hesai_lidar:src/go2w-hesai-lidar-driver/package.xml"
  "go2w_imu_publisher:src/go2w-imu-publisher/package.xml"
  "unitree_api:src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api/package.xml"
  "unitree_go:src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/package.xml"
  "realsense2_description:src/realsense-ros/realsense2_description/package.xml"
  "realsense2_camera:src/realsense-ros/realsense2_camera/package.xml"
)

submodule_paths=(
  "humble_ws/src/fast-livo2"
  "humble_ws/src/go2w-hesai-lidar-driver"
  "humble_ws/src/go2w-imu-publisher"
  "humble_ws/src/unitree_ros2"
  "humble_ws/src/realsense-ros"
  "humble_ws/src/librealsense"
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

prepare_livox_manifest() {
  local driver_dir="$1"
  local ros2_manifest="${driver_dir}/package_ROS2.xml"
  local manifest="${driver_dir}/package.xml"

  if [ -f "${ros2_manifest}" ]; then
    cp -f "${ros2_manifest}" "${manifest}"
  fi
}

ensure_livox_driver_source() {
  local driver_dir="${WS_DIR}/src/livox_ros_driver2"
  local candidate
  local candidates=(
    "${PROJECT_DIR}/../livox_ros_driver2"
    "/fast_livo2/src/livox_ros_driver2"
    "/external/../livox_ros_driver2"
  )

  if [ -e "${driver_dir}" ]; then
    prepare_livox_manifest "${driver_dir}"
    return
  fi

  for candidate in "${candidates[@]}"; do
    if [ -f "${candidate}/CMakeLists.txt" ] && [ -f "${candidate}/package_ROS2.xml" ]; then
      ln -s "${candidate}" "${driver_dir}"
      prepare_livox_manifest "${driver_dir}"
      echo "Linked Livox ROS 2 driver: ${driver_dir} -> ${candidate}"
      return
    fi
  done

  echo "Missing livox_ros_driver2 source." >&2
  echo "Clone it next to this repository or under /fast_livo2/src:" >&2
  echo "  git clone https://github.com/Livox-SDK/livox_ros_driver2.git ../livox_ros_driver2" >&2
  exit 1
}

ensure_git_safe_directories() {
  local submodule_path

  git -C "${PROJECT_DIR}" config --global --add safe.directory "${PROJECT_DIR}" || true

  for submodule_path in "${submodule_paths[@]}"; do
    git -C "${PROJECT_DIR}" config --global --add safe.directory "${PROJECT_DIR}/${submodule_path}" || true
  done
}

update_submodules() {
  git -C "${PROJECT_DIR}" submodule sync --recursive -- "${submodule_paths[@]}"
  git -C "${PROJECT_DIR}" submodule update --init --recursive --force --checkout -- "${submodule_paths[@]}"
}

repair_submodules() {
  echo "Retrying submodule initialization after clearing stale submodule state..." >&2
  git -C "${PROJECT_DIR}" submodule deinit -f -- "${submodule_paths[@]}" || true
  update_submodules
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
  ensure_git_safe_directories
  if ! update_submodules; then
    echo "Failed to initialize submodules. Check network access and git credentials." >&2
    exit 1
  fi

  mapfile -t missing_packages < <(list_missing_packages)
  if [ "${#missing_packages[@]}" -gt 0 ]; then
    echo "Still missing ROS package sources after forced submodule update: ${missing_packages[*]}" >&2
    if ! repair_submodules; then
      echo "Failed to repair submodules. Check network access and git credentials." >&2
      exit 1
    fi
    mapfile -t missing_packages < <(list_missing_packages)
  fi

  if [ "${#missing_packages[@]}" -gt 0 ]; then
    echo "Still missing ROS package sources after submodule initialization: ${missing_packages[*]}" >&2
    git -C "${PROJECT_DIR}" submodule status --recursive -- "${submodule_paths[@]}" >&2 || true
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

ensure_livox_driver_source
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
    livox_ros_driver2 \
    hesai_lidar \
    go2w_imu_publisher \
    realsense2_description \
    realsense2_camera \
    fast_livo2 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble \
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
