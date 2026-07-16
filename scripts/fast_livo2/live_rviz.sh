#!/bin/bash
# Launch RViz2 for live FAST-LIVO2 visualization.
#
# Usage:
#   bash scripts/fast_livo2/live_rviz.sh [--iface <desktop_interface>] [--config <rviz_config>] [--rmw auto|cyclonedds|system] [--wait-topics <seconds>] [--source-workspace auto|always|never] [--static-camera-init auto|always|never] [--check-only|--diagnose]

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

setup_sources_without_missing_files() {
    local setup_script="$1"
    local output

    output="$(bash -c "set +u; source \"$setup_script\"" 2>&1 >/dev/null || true)"
    [ -z "$output" ]
}

source_workspace_setup() {
    local mode="$1"
    local setup

    if [ "$mode" = "never" ]; then
        return
    fi

    for setup in \
        "$REPO_ROOT/humble_ws/install/setup.bash" \
        "$REPO_ROOT/.devcontainer/offline_fast_livo2/install/setup.bash"; do
        if [ ! -f "$setup" ]; then
            continue
        fi

        if [ "$mode" = "always" ] || setup_sources_without_missing_files "$setup"; then
            source_setup_safely "$setup"
        else
            echo "Warning: skipping incomplete workspace setup: $setup" >&2
            echo "Use --source-workspace always only after rebuilding the workspace install space." >&2
        fi
        return
    done
}

iface="$IFACE_DEFAULT"
rviz_cfg="$RVIZ_CFG_DEFAULT"
rmw_mode="${FAST_LIVO2_RVIZ_RMW:-auto}"
topic_wait="${FAST_LIVO2_RVIZ_WAIT_TOPICS:-5}"
topic_check=1
source_workspace="${FAST_LIVO2_RVIZ_SOURCE_WORKSPACE:-auto}"
static_camera_init="${FAST_LIVO2_RVIZ_STATIC_CAMERA_INIT:-auto}"
check_only=0
diagnose_only=0
static_tf_pid=""

cleanup() {
    if [ -n "${static_tf_pid:-}" ] && kill -0 "$static_tf_pid" 2>/dev/null; then
        kill "$static_tf_pid" 2>/dev/null || true
        wait "$static_tf_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

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
        --wait-topics)
            topic_wait="${2:?Error: --wait-topics requires a value}"
            shift 2
            ;;
        --source-workspace)
            source_workspace="${2:?Error: --source-workspace requires a value}"
            shift 2
            ;;
        --static-camera-init)
            static_camera_init="${2:?Error: --static-camera-init requires a value}"
            shift 2
            ;;
        --no-topic-check)
            topic_check=0
            shift
            ;;
        --check-only)
            check_only=1
            shift
            ;;
        --diagnose)
            diagnose_only=1
            shift
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

case "$source_workspace" in
    auto|always|never)
        ;;
    *)
        echo "Error: --source-workspace must be one of: auto, always, never" >&2
        exit 1
        ;;
esac

case "$static_camera_init" in
    auto|always|never)
        ;;
    *)
        echo "Error: --static-camera-init must be one of: auto, always, never" >&2
        exit 1
        ;;
esac

case "$topic_wait" in
    ''|*[!0-9]*)
        echo "Error: --wait-topics must be a non-negative integer number of seconds" >&2
        exit 1
        ;;
esac

if ! command -v ros2 >/dev/null 2>&1 || ! command -v rviz2 >/dev/null 2>&1; then
    source_ros_setup
fi

source_workspace_setup "$source_workspace"

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

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ "${ROS_LOCALHOST_ONLY:-0}" != "0" ]; then
    echo "Warning: ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; overriding to 0 for live network RViz." >&2
fi
export ROS_LOCALHOST_ONLY=0

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
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "ros2: $(command -v ros2)"
echo "rviz2: $(command -v rviz2)"
if [ -n "${CYCLONEDDS_URI:-}" ]; then
    echo "CycloneDDS interface: $iface"
fi
echo "RViz config: $rviz_cfg"

has_live_topics() {
    local topics="$1"
    local topic

    for topic in /cloud_registered /aft_mapped_to_init /path /rgb_img; do
        if printf '%s\n' "$topics" | grep -Fxq "$topic"; then
            return 0
        fi
    done

    return 1
}

print_missing_topics() {
    local topics="$1"
    local missing=0
    local topic

    for topic in \
        /hesai_node/points_raw \
        /go2w/imu \
        /camera/camera/color/image_raw \
        /cloud_registered \
        /aft_mapped_to_init \
        /path \
        /rgb_img \
        /tf; do
        if ! printf '%s\n' "$topics" | grep -Fxq "$topic"; then
            echo "  missing: $topic" >&2
            missing=1
        fi
    done

    return "$missing"
}

print_topic_info() {
    local topic="$1"

    echo
    echo "== $topic =="
    ros2 topic info -v "$topic" 2>&1 | sed 's/^/  /' || true
}

print_topic_sample() {
    local topic="$1"
    local tmp

    echo "-- $topic sample"
    tmp="$(mktemp)"
    if timeout 10 ros2 topic echo --once "$topic" >"$tmp" 2>&1; then
        sed -n '1,80p' "$tmp" | sed 's/^/  /'
    else
        if [ -s "$tmp" ]; then
            sed -n '1,80p' "$tmp" | sed 's/^/  /'
        else
            echo "  <no sample received within 10 seconds>"
        fi
    fi
    rm -f "$tmp"
}

print_topic_rate() {
    local topic="$1"
    local tmp

    echo "-- $topic rate"
    tmp="$(mktemp)"
    timeout --signal=INT 6 ros2 topic hz "$topic" >"$tmp" 2>&1 || true
    if [ -s "$tmp" ]; then
        sed -n '1,40p' "$tmp" | sed 's/^/  /'
    else
        echo "  <no messages observed within 6 seconds>"
    fi
    rm -f "$tmp"
}

print_topic_field() {
    local topic="$1"
    local field="$2"
    local tmp

    echo "-- $topic.$field"
    tmp="$(mktemp)"
    if timeout 10 ros2 topic echo --once --field "$field" "$topic" >"$tmp" 2>&1; then
        sed -n '1,20p' "$tmp" | sed 's/^/  /'
    else
        if [ -s "$tmp" ]; then
            sed -n '1,20p' "$tmp" | sed 's/^/  /'
        else
            echo "  <no field sample received within 10 seconds>"
        fi
    fi
    rm -f "$tmp"
}

print_topic_metadata() {
    echo
    echo "Sample metadata:"
    print_topic_field /cloud_registered header.frame_id
    print_topic_field /cloud_registered width
    print_topic_field /cloud_registered height
    print_topic_field /cloud_registered point_step
    print_topic_field /cloud_registered row_step
    print_topic_field /cloud_registered is_dense
    print_topic_field /aft_mapped_to_init header.frame_id
    print_topic_field /aft_mapped_to_init child_frame_id
    print_topic_field /aft_mapped_to_init pose.pose.position
    print_topic_field /path header.frame_id
    print_topic_field /rgb_img header.frame_id
    print_topic_field /rgb_img width
    print_topic_field /rgb_img height
    print_topic_field /rgb_img encoding
}

print_tf_sample() {
    echo
    echo "TF sample:"
    if ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
        timeout 5 ros2 run tf2_ros tf2_echo camera_init aft_mapped 2>&1 | sed -n '1,30p' | sed 's/^/  /' || true
    else
        echo "  <tf2_ros is not available>"
    fi
}

diagnose_live_topics() {
    local topic

    echo
    echo "Topic diagnostics from this RViz environment:"
    for topic in /cloud_registered /aft_mapped_to_init /path /rgb_img /tf; do
        print_topic_info "$topic"
    done

    echo
    echo "Topic rates:"
    print_topic_rate /cloud_registered
    print_topic_rate /aft_mapped_to_init
    print_topic_rate /path
    print_topic_rate /rgb_img
    print_topic_rate /tf

    print_topic_metadata
    print_tf_sample

    echo
    echo "Sample messages:"
    print_topic_sample /cloud_registered
    print_topic_sample /aft_mapped_to_init
    print_topic_sample /path
    print_topic_sample /rgb_img
    print_topic_sample /tf
}

start_static_camera_init_tf() {
    if [ "$static_camera_init" = "never" ]; then
        return
    fi

    if ! ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
        echo "Warning: tf2_ros is not available; cannot publish static camera_init frame." >&2
        return
    fi

    ros2 run tf2_ros static_transform_publisher \
        0 0 0 0 0 0 rviz_world camera_init >/tmp/fast_livo2_rviz_static_tf.log 2>&1 &
    static_tf_pid=$!
    echo "Static TF fallback enabled: rviz_world -> camera_init (pid $static_tf_pid)"
}

if [ "$topic_check" -eq 1 ]; then
    visible_topics="$(ros2 topic list 2>/dev/null || true)"
    waited=0
    while ! has_live_topics "$visible_topics" && [ "$waited" -lt "$topic_wait" ]; do
        sleep 1
        waited=$((waited + 1))
        visible_topics="$(ros2 topic list 2>/dev/null || true)"
    done

    if ! has_live_topics "$visible_topics"; then
        echo "Warning: no FAST-LIVO2 live topics are visible from this RViz environment." >&2
        echo "Check the interface, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, and CycloneDDS settings." >&2
        echo "Visible topics:" >&2
        if [ -n "$visible_topics" ]; then
            printf '%s\n' "$visible_topics" >&2
        else
            echo "  <none>" >&2
        fi
    elif print_missing_topics "$visible_topics"; then
        :
    else
        echo "Warning: some expected live topics are missing from this RViz environment." >&2
        echo "Missing topics can indicate that a sensor node is not publishing, or that RViz is using a different DDS environment than your echo terminal." >&2
    fi
fi

if [ "$diagnose_only" -eq 1 ]; then
    diagnose_live_topics
    exit 0
fi

if [ "$check_only" -eq 1 ]; then
    echo "Visible topics from this RViz environment:"
    if [ -n "${visible_topics:-}" ]; then
        printf '%s\n' "$visible_topics" | sort
    else
        echo "  <none>"
    fi
    exit 0
fi

start_static_camera_init_tf

rviz2 -d "$rviz_cfg"
