#!/bin/bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
FAST_LIVO2_SRC="$REPO_ROOT/humble_ws/src/fast-livo2"
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

bash "$REPO_ROOT/.devcontainer/configure_git_safe_directory.sh"

source "$ROS_SETUP"

mkdir -p "$DESKTOP_WS_ROOT"
rm -rf "$DESKTOP_WS_ROOT/build" "$DESKTOP_WS_ROOT/install" "$DESKTOP_WS_ROOT/log"

colcon --log-base "$DESKTOP_WS_ROOT/log" build \
    --symlink-install \
    --base-paths "$FAST_LIVO2_SRC" \
    --build-base "$DESKTOP_WS_ROOT/build" \
    --install-base "$DESKTOP_INSTALL" \
    --packages-select fast_livo2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "source $DESKTOP_INSTALL/setup.bash" ~/.bashrc || \
    echo "source $DESKTOP_INSTALL/setup.bash" >> ~/.bashrc

echo "Desktop offline FAST-LIVO2 environment is ready."
echo "Installed setup: $DESKTOP_INSTALL/setup.bash"
