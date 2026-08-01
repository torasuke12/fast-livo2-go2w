#!/bin/bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
FAST_LIVO2_SRC="$REPO_ROOT/humble_ws/src/fast-livo2"
LIVOX_DRIVER_SRC="$REPO_ROOT/humble_ws/src/livox_ros_driver2"
DESKTOP_WS_ROOT="$REPO_ROOT/.devcontainer/offline_fast_livo2"
DESKTOP_INSTALL="$DESKTOP_WS_ROOT/install"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -d "$FAST_LIVO2_SRC" ]; then
    echo "Error: FAST-LIVO2 source not found: $FAST_LIVO2_SRC" >&2
    echo "Did you clone submodules? Run: git submodule update --init --recursive" >&2
    exit 1
fi

prepare_livox_driver() {
    local candidate
    local candidates=(
        "$REPO_ROOT/humble_ws/src/livox_ros_driver2"
        "$REPO_ROOT/humble_ws/src/livox_ros_driver"
        "$REPO_ROOT/../livox_ros_driver2"
        "/fast_livo2/src/livox_ros_driver2"
    )

    for candidate in "${candidates[@]}"; do
        if [ -f "$candidate/CMakeLists.txt" ] && [ -f "$candidate/package_ROS2.xml" ]; then
            if [ "$candidate" != "$LIVOX_DRIVER_SRC" ] && [[ "$candidate" != "$REPO_ROOT/humble_ws/src/"* ]] && [ ! -e "$LIVOX_DRIVER_SRC" ]; then
                ln -s "$candidate" "$LIVOX_DRIVER_SRC"
                candidate="$LIVOX_DRIVER_SRC"
            fi
            cp -f "$candidate/package_ROS2.xml" "$candidate/package.xml"
            LIVOX_DRIVER_SRC="$candidate"
            return
        fi
    done

    echo "Error: livox_ros_driver2 source not found." >&2
    echo "Clone it next to this repository or mount it at /fast_livo2/src/livox_ros_driver2." >&2
    exit 1
}

bash "$REPO_ROOT/.devcontainer/configure_git_safe_directory.sh"
prepare_livox_driver

source "$ROS_SETUP"

mkdir -p "$DESKTOP_WS_ROOT"
rm -rf "$DESKTOP_WS_ROOT/build" "$DESKTOP_WS_ROOT/install" "$DESKTOP_WS_ROOT/log"

colcon --log-base "$DESKTOP_WS_ROOT/log" build \
    --symlink-install \
    --base-paths "$FAST_LIVO2_SRC" "$LIVOX_DRIVER_SRC" \
    --build-base "$DESKTOP_WS_ROOT/build" \
    --install-base "$DESKTOP_INSTALL" \
    --packages-up-to fast_livo2 \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS=humble

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "source $DESKTOP_INSTALL/setup.bash" ~/.bashrc || \
    echo "source $DESKTOP_INSTALL/setup.bash" >> ~/.bashrc

echo "Desktop offline FAST-LIVO2 environment is ready."
echo "Installed setup: $DESKTOP_INSTALL/setup.bash"
