#!/bin/bash
# Launch RViz2 for live FAST-LIVO2 visualization.
#
# Usage:
#   bash scripts/fast_livo2/live_rviz.sh [--iface <desktop_interface>] [--config <rviz_config>] [--rmw auto|cyclonedds|system]

set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG_DEFAULT="$REPO_ROOT/config/fast_livo2/fast_livo2.rviz"
IFACE_DEFAULT="${CYCLONEDDS_IFACE:-wlan0}"

source_setup_safely() {
    local setup_script="$1"
    local restore_nounset=0
    local rc=0

    case $- in
        *u*)
            restore_nounset=1
            set +u
            ;;
    esac

    export COLCON_TRACE="${COLCON_TRACE-}"
    # shellcheck source=/dev/null
    source "$setup_script" || rc=$?

    if [ "$restore_nounset" -eq 1 ]; then
        set -u
    fi

    return "$rc"
}

list_available_interfaces() {
    find /sys/class/net -mindepth 1 -maxdepth 1 -printf '%f\n' | sort
}

source_ros_setup() {
    local setup
    local candidates=()

    if [ -n "${ROS_DISTRO:-}" ]; then
        candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
    fi
    candidates+=(
        /opt/ros/jazzy/setup.bash
        /opt/ros/humble/setup.bash
    )

    for setup in "${candidates[@]}"; do
        if [ -f "$setup" ]; then
            source_setup_safely "$setup"
            return
        fi
    done

    echo "Error: ros2/rviz2 not found and no ROS setup file was found under /opt/ros." >&2
    echo "Source your ROS 2 desktop setup first, for example:" >&2
    echo "  source /opt/ros/jazzy/setup.bash" >&2
    exit 1
}

cyclonedds_apt_package() {
    local distro="${ROS_DISTRO:-<ros-distro>}"
    echo "ros-${distro}-rmw-cyclonedds-cpp"
}

iface="$IFACE_DEFAULT"
rviz_cfg="$RVIZ_CFG_DEFAULT"
rmw_mode="${FAST_LIVO2_RVIZ_RMW:-auto}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --iface)
            iface="${2:?Error: --iface requires a value}"
            shift 2
            ;;
        --config)
            rviz_cfg="${2:?Error: --config requires a value}"
            shift 2
            ;;
        --rmw)
            rmw_mode="${2:?Error: --rmw requires a value}"
            shift 2
            ;;
        -h|--help)
            sed -n '2,6p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

case "$rmw_mode" in
    auto|cyclonedds|system)
        ;;
    *)
        echo "Error: --rmw must be one of: auto, cyclonedds, system" >&2
        exit 1
        ;;
esac

if ! command -v ros2 >/dev/null 2>&1 || ! command -v rviz2 >/dev/null 2>&1; then
    source_ros_setup
fi

for setup in \
    "$REPO_ROOT/humble_ws/install/setup.bash" \
    "$REPO_ROOT/.devcontainer/offline_fast_livo2/install/setup.bash"; do
    if [ -f "$setup" ]; then
        source_setup_safely "$setup"
        break
    fi
done

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH" >&2
    exit 1
fi

if [ ! -f "$rviz_cfg" ]; then
    echo "Error: RViz config not found: $rviz_cfg" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "Error: DISPLAY is not set. Start this from a desktop session or fix X11 forwarding." >&2
    exit 1
fi

if [ ! -d "/sys/class/net/$iface" ]; then
    echo "Error: network interface not found: $iface" >&2
    echo "Available interfaces:" >&2
    list_available_interfaces >&2
    exit 1
fi

if [ "$rmw_mode" = "system" ]; then
    echo "Using existing ROS 2 RMW implementation: ${RMW_IMPLEMENTATION:-system default}"
else
    if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"$iface\" priority=\"2\" multicast=\"true\" />
</Interfaces></General></Domain></CycloneDDS>"
    elif [ "$rmw_mode" = "cyclonedds" ]; then
        echo "Error: rmw_cyclonedds_cpp is not installed in this ROS 2 environment." >&2
        echo "Install it for your ROS 2 distro with:" >&2
        echo "  sudo apt install $(cyclonedds_apt_package)" >&2
        echo "Or run with --rmw system to use the currently installed RMW." >&2
        exit 1
    else
        echo "Warning: rmw_cyclonedds_cpp is not installed; using the current ROS 2 RMW." >&2
        echo "For CycloneDDS interface pinning, install:" >&2
        echo "  sudo apt install $(cyclonedds_apt_package)" >&2
    fi
fi

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-system default}"
if [ -n "${CYCLONEDDS_URI:-}" ]; then
    echo "CycloneDDS interface: $iface"
fi
echo "RViz config: $rviz_cfg"

rviz2 -d "$rviz_cfg"
