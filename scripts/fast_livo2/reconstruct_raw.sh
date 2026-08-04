#!/bin/bash
# Reconstruct FAST-LIVO2 outputs from a raw sensor bag and open RViz2.
#
# Usage:
#   bash scripts/fast_livo2/reconstruct_raw.sh <bag_directory> [ros2 bag play args...]

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=("$@")
DEFAULT_BAG_PLAY_DELAY="${FAST_LIVO2_BAG_PLAY_DELAY:-5}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/fast_livo2/fast_livo2.rviz"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/humble_ws/install/setup.bash"
    "$REPO_ROOT/.devcontainer/offline_fast_livo2/install/setup.bash"
)

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

bag_topics=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    print(t['topic_metadata']['name'])
" "$BAG/metadata.yaml" 2>/dev/null) \
    || { echo "Error: failed to parse $BAG/metadata.yaml" >&2; exit 1; }

has_topic() {
    local topic="$1"
    grep -qxF "$topic" <<< "$bag_topics"
}

print_bag_topics() {
    echo "Topics found in bag:" >&2
    while IFS= read -r t; do echo "  $t" >&2; done <<< "$bag_topics"
}

PARAMS_FILE="$REPO_ROOT/humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml"
SENSOR_PROFILE=""

if has_topic /livox/lidar && has_topic /livox/imu; then
    SENSOR_PROFILE="livox_mid360"
    PARAMS_FILE="$REPO_ROOT/humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml"
elif has_topic /hesai_node/points_raw && has_topic /go2w/imu; then
    SENSOR_PROFILE="hesai_go2w_imu"
    PARAMS_FILE="$REPO_ROOT/humble_ws/src/fast-livo2/config/go2w_hesai_fast_livo2.yaml"
else
    echo "Error: bag is missing a supported LiDAR/IMU topic pair." >&2
    echo "Expected either:" >&2
    echo "  - /livox/lidar and /livox/imu" >&2
    echo "  - /hesai_node/points_raw and /go2w/imu" >&2
    print_bag_topics
    exit 1
fi

for required in /camera/camera/color/image_raw; do
    if ! grep -qxF "$required" <<< "$bag_topics"; then
        echo "Error: bag is missing required topic: $required" >&2
        print_bag_topics
        exit 1
    fi
done

CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then
    CLOCK_ARG=()
fi

has_bag_play_delay_arg=0
for arg in "${EXTRA_ARGS[@]}"; do
    case "$arg" in
        --delay|-d|--delay=*)
            has_bag_play_delay_arg=1
            break
            ;;
    esac
done

BAG_PLAY_ARGS=("$BAG" "${CLOCK_ARG[@]}")
if [ "$has_bag_play_delay_arg" -eq 0 ]; then
    BAG_PLAY_ARGS+=(--delay "$DEFAULT_BAG_PLAY_DELAY")
fi
BAG_PLAY_ARGS+=("${EXTRA_ARGS[@]}")

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then
        WS_SETUP="$candidate"
        break
    fi
done

if [ -z "$WS_SETUP" ]; then
    echo "Error: no FAST-LIVO2 workspace setup was found." >&2
    echo "Looked for:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "Build the workspace first, then rerun this script." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix fast_livo2 >/dev/null 2>&1; then
    echo "Error: fast_livo2 is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    exit 1
fi

terminate_process_group() {
    local signal="$1"
    local pgid="${2:-}"
    if [ -n "$pgid" ]; then
        kill -"$signal" -- "-$pgid" 2>/dev/null || true
    fi
}

is_process_group_alive() {
    local pgid="${1:-}"
    [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null
}

CLEANUP_STARTED=0

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [ "$CLEANUP_STARTED" -eq 1 ]; then
        return "$status"
    fi
    CLEANUP_STARTED=1

    echo "Stopping raw-bag FAST-LIVO2 reconstruction..."
    terminate_process_group TERM "${RVIZ_PGID:-}"
    for pgid in "${BAG_PGID:-}" "${FAST_LIVO2_PGID:-}"; do
        terminate_process_group INT "$pgid"
    done

    for _ in 1 2 3 4 5; do
        local any_alive=0
        for pgid in "${RVIZ_PGID:-}" "${BAG_PGID:-}" "${FAST_LIVO2_PGID:-}"; do
            if is_process_group_alive "$pgid"; then
                any_alive=1
                break
            fi
        done
        [ "$any_alive" -eq 0 ] && break
        sleep 1
    done

    for pgid in "${RVIZ_PGID:-}" "${BAG_PGID:-}" "${FAST_LIVO2_PGID:-}"; do
        terminate_process_group KILL "$pgid"
    done

    for pid in "${RVIZ_PID:-}" "${BAG_PID:-}" "${FAST_LIVO2_PID:-}"; do
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done

    return "$status"
}

trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "FAST-LIVO2 setup: $WS_SETUP"
echo "Sensor profile: $SENSOR_PROFILE"
echo "FAST-LIVO2 params: $PARAMS_FILE"
if [ "$has_bag_play_delay_arg" -eq 0 ]; then
    echo "Bag play delay: ${DEFAULT_BAG_PLAY_DELAY}s"
else
    echo "Bag play delay: set by ros2 bag play arguments"
fi
echo "Mode: replay raw topics, run FAST-LIVO2 offline, visualize generated outputs"
echo ""

setsid ros2 launch fast_livo2 go2w_fast_livo2.launch.py \
    use_sensor_bringup:=false \
    use_sim_time:=true \
    params_file:="$PARAMS_FILE" &
FAST_LIVO2_PID=$!
FAST_LIVO2_PGID=$FAST_LIVO2_PID

sleep 3
if ! kill -0 "$FAST_LIVO2_PID" 2>/dev/null; then
    echo "Error: FAST-LIVO2 exited during startup." >&2
    wait "$FAST_LIVO2_PID" || true
    exit 1
fi

setsid ros2 bag play "${BAG_PLAY_ARGS[@]}" &
BAG_PID=$!
BAG_PGID=$BAG_PID

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

setsid rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!
RVIZ_PGID=$RVIZ_PID

wait "$RVIZ_PID"
