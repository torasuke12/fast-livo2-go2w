#!/bin/bash
# Play a recorded FAST-LIVO2 output bag and open RViz2.
#
# Usage:
#   bash scripts/fast_livo2/playback.sh <bag_directory> [--rviz] [--no-rviz] [--rviz-software] [--rviz-config <rviz_config>] [ros2 bag play args...]

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=()
RVIZ=true
RVIZ_SOFTWARE="${FAST_LIVO2_PLAYBACK_RVIZ_SOFTWARE:-0}"
RVIZ_CFG_ARG=""

while [ $# -gt 0 ]; do
    case "$1" in
        --rviz)
            RVIZ=true
            shift
            ;;
        --no-rviz)
            RVIZ=false
            shift
            ;;
        --rviz-software)
            RVIZ_SOFTWARE=1
            shift
            ;;
        --rviz-config)
            RVIZ_CFG_ARG="${2:?Error: --rviz-config requires a value}"
            shift 2
            ;;
        --loop)
            echo "Error: --loop is not supported in this playback helper." >&2
            exit 2
            ;;
        -h|--help)
            sed -n '1,6p' "$0"
            exit 0
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/fast_livo2/fast_livo2.rviz"
ROS_SETUP="/opt/ros/humble/setup.bash"
DESKTOP_SETUP="$REPO_ROOT/.devcontainer/offline_fast_livo2/install/setup.bash"

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

setup_sources_without_missing_files() {
    local setup_script="$1"
    local output

    output="$(bash -c "set +u; source \"$setup_script\"" 2>&1 >/dev/null || true)"
    [ -z "$output" ]
}

if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then
    BAG="$REPO_ROOT/$BAG"
fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2
    exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: metadata.yaml not found in bag directory: $BAG" >&2
    exit 1
fi

if [ -n "$RVIZ_CFG_ARG" ]; then
    if [ -f "$RVIZ_CFG_ARG" ]; then
        RVIZ_CFG="$(cd "$(dirname "$RVIZ_CFG_ARG")" && pwd)/$(basename "$RVIZ_CFG_ARG")"
    elif [ -f "$REPO_ROOT/$RVIZ_CFG_ARG" ]; then
        RVIZ_CFG="$(cd "$(dirname "$REPO_ROOT/$RVIZ_CFG_ARG")" && pwd)/$(basename "$RVIZ_CFG_ARG")"
    else
        echo "Error: RViz config not found: $RVIZ_CFG_ARG" >&2
        exit 1
    fi
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

source_setup_safely "$ROS_SETUP"

if [ -f "$DESKTOP_SETUP" ]; then
    if setup_sources_without_missing_files "$DESKTOP_SETUP"; then
        source_setup_safely "$DESKTOP_SETUP"
    else
        echo "Warning: skipping incomplete desktop workspace setup: $DESKTOP_SETUP" >&2
    fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found in PATH after sourcing $ROS_SETUP" >&2
    exit 1
fi

if [ "$RVIZ" = true ] && ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH after sourcing $ROS_SETUP" >&2
    exit 1
fi

if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION="${FAST_LIVO2_PLAYBACK_RMW:-rmw_cyclonedds_cpp}"
fi
export ROS_DOMAIN_ID="${FAST_LIVO2_PLAYBACK_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${FAST_LIVO2_PLAYBACK_LOCALHOST_ONLY:-1}"
if [ "${FAST_LIVO2_PLAYBACK_KEEP_CYCLONEDDS_URI:-0}" != "1" ]; then
    unset CYCLONEDDS_URI
fi

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "RViz enabled: $RVIZ"
echo "RViz software rendering: $RVIZ_SOFTWARE"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-system default}"
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "ros2: $(command -v ros2)"
if [ "$RVIZ" = true ]; then
    echo "rviz2: $(command -v rviz2)"
fi
echo ""

PLAY_ARGS=(--clock)
PLAY_ARGS+=("${EXTRA_ARGS[@]}")
ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
BAG_PID=$!

trap "echo 'Stopping bag player...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null" EXIT

sleep 2
if [ "$RVIZ" = true ]; then
    if [ "$RVIZ_SOFTWARE" = "1" ]; then
        export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
        export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
        export QT_OPENGL="${QT_OPENGL:-software}"
        export LIBGL_ALWAYS_SOFTWARE=1
        export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
        export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
        export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
        export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
    fi
    set +e
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
    RVIZ_RC=$?
    set -e
    if [ "$RVIZ_RC" -ne 0 ]; then
        echo "Warning: RViz exited with status $RVIZ_RC; stopping bag playback." >&2
        exit "$RVIZ_RC"
    fi
else
    wait "$BAG_PID"
    trap - EXIT
fi
