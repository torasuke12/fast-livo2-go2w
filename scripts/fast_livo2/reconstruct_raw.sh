#!/bin/bash
# Reconstruct FAST-LIVO2 outputs from a raw sensor bag and open RViz2.
#
# Usage:
#   bash scripts/fast_livo2/reconstruct_raw.sh <bag_directory> [--lio-only] [ros2 bag play args...]
#
# Only raw sensor topics are replayed. Recorded FAST-LIVO2 output topics in a
# mixed bag are deliberately excluded so that they cannot collide with newly
# reconstructed outputs.

set -eo pipefail
ORIGINAL_ARGS=("$@")

usage() {
    cat <<EOF
Usage: $0 <bag_directory> [--lio-only] [ros2 bag play args...]

Modes:
  default       Run LiDAR-inertial-visual odometry (LIVO).
  --lio-only    Disable image processing and run LiDAR-inertial odometry only.
  --params <yaml>          Use an explicit estimator configuration for online replay.
  --offline-correction    Show fresh LIO/LIVO, then apply offline map correction in RViz.
  --return-to-start       Verify return-room matches in offline-correction mode.
  --output-dir <new-dir>  Output for offline-correction mode (default: unique run).
  --normal-gate on|off     Override source/map normal gating; omitted uses YAML.
  --temporal-window on|off Joint pose/velocity/bias window for LIO or LIVO.
  --temporal-initialization single_scan|legacy_voxel
                          Override initialization only; requires temporal mode.
  --correspondence-mode voxel_plane|fastlio_ikdtree|fastlio_knn
                          Override LIO matching; omitted uses YAML (MID360: voxel_plane).
  --capture-diagnostics    Save isolated logs, diagnostics, provenance, and IMU stamps.
  --capture-root <dir>     Parent for unique captures (default: repo/diagnostic_captures).
                          Detail/geometry: [0,360) estimator seconds; playback continues.

Set FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID to use a separate ROS domain (default: 42).
Startup refuses a domain that already contains /laserMapping.

The bag player is restricted to the raw LiDAR/IMU/camera topics selected for
the detected sensor profile. Recorded odometry, paths, maps, images, and TF
outputs are not replayed.

Examples:
  $0 bags/raw_YYYYMMDD_HHMMSS
  $0 bags/raw_YYYYMMDD_HHMMSS --lio-only
  $0 bags/raw_YYYYMMDD_HHMMSS --lio-only --rate 0.25
EOF
}

if [ "$#" -eq 0 ]; then
    usage >&2
    exit 1
fi
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    usage
    exit 0
fi

BAG="$1"
shift
RECONSTRUCTION_MODE="${FAST_LIVO2_RECONSTRUCTION_MODE:-livo}"
NORMAL_GATE=""
CORRESPONDENCE_MODE=""
TEMPORAL_WINDOW=""
TEMPORAL_INITIALIZATION=""
PARAMS_OVERRIDE=""
CAPTURE_DIAGNOSTICS=0
OFFLINE_CORRECTION=0
RETURN_TO_START=0
OUTPUT_DIR=""
CAPTURE_ROOT=""
CAPTURE_DIR=""
TEMP_PARAMS=""
STOP_REASON="startup"
EXTRA_ARGS=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --params)
            if [ "$#" -lt 2 ] || [[ "$2" == --* ]]; then
                echo "Error: --params requires a YAML file." >&2; exit 1
            fi
            PARAMS_OVERRIDE="$2"; shift ;;
        --offline-correction) OFFLINE_CORRECTION=1 ;;
        --return-to-start) RETURN_TO_START=1 ;;
        --output-dir)
            if [ "$#" -lt 2 ] || [[ "$2" == --* ]]; then
                echo "Error: --output-dir requires a new directory." >&2; exit 1
            fi
            OUTPUT_DIR="$2"; shift ;;
        --lio-only)
            RECONSTRUCTION_MODE="lio"
            ;;
        --normal-gate)
            if [ "$#" -lt 2 ] || [[ "$2" != on && "$2" != off ]]; then
                echo "Error: --normal-gate requires on or off." >&2; exit 1
            fi
            NORMAL_GATE="$2"; shift
            ;;
        --temporal-window)
            if [ "$#" -lt 2 ] || [[ "$2" != on && "$2" != off ]]; then
                echo "Error: --temporal-window requires on or off." >&2; exit 1
            fi
            TEMPORAL_WINDOW="$2"; shift
            ;;
        --temporal-initialization)
            if [ "$#" -lt 2 ] || [[ "$2" != single_scan && "$2" != legacy_voxel ]]; then
                echo "Error: --temporal-initialization requires single_scan or legacy_voxel." >&2; exit 1
            fi
            TEMPORAL_INITIALIZATION="$2"; shift
            ;;
        --correspondence-mode)
            if [ "$#" -lt 2 ] || [[ "$2" != voxel_plane && "$2" != fastlio_ikdtree && "$2" != fastlio_knn ]]; then
                echo "Error: --correspondence-mode requires voxel_plane, fastlio_ikdtree, or fastlio_knn." >&2; exit 1
            fi
            CORRESPONDENCE_MODE="$2"; shift
            ;;
        --capture-diagnostics)
            CAPTURE_DIAGNOSTICS=1
            ;;
        --capture-root)
            if [ "$#" -lt 2 ] || [[ -z "$2" || "$2" == --* ]]; then
                echo "Error: --capture-root requires a directory." >&2; exit 1
            fi
            CAPTURE_ROOT="$2"; shift
            ;;
        --params=*|--normal-gate=*|--capture-root=*|--correspondence-mode=*|--temporal-window=*|--temporal-initialization=*)
            echo "Error: separate the option and value (--params, --normal-gate, --correspondence-mode, --temporal-window, --temporal-initialization, --capture-root)." >&2; exit 1
            ;;
        --topics|--topics=*)
            echo "Error: --topics is selected automatically to exclude recorded outputs." >&2
            exit 1
            ;;
        *)
            EXTRA_ARGS+=("$1")
            ;;
    esac
    shift
done

if [ -n "$CAPTURE_ROOT" ] && [ "$CAPTURE_DIAGNOSTICS" -ne 1 ]; then
    echo "Error: --capture-root requires --capture-diagnostics." >&2; exit 1
fi

case "$RECONSTRUCTION_MODE" in
    livo)
        FAST_LIVO2_IMG_EN=1
        ;;
    lio|lio-only)
        RECONSTRUCTION_MODE="lio"
        FAST_LIVO2_IMG_EN=0
        ;;
    *)
        echo "Error: unsupported reconstruction mode: $RECONSTRUCTION_MODE" >&2
        echo "Expected livo or lio." >&2
        exit 1
        ;;
esac

DEFAULT_BAG_PLAY_DELAY="${FAST_LIVO2_BAG_PLAY_DELAY:-5}"
DEFAULT_BAG_PLAY_RATE="${FAST_LIVO2_BAG_PLAY_RATE:-1.0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ "$OFFLINE_CORRECTION" -eq 1 ]; then
    if [ -n "$NORMAL_GATE$CORRESPONDENCE_MODE$TEMPORAL_INITIALIZATION$PARAMS_OVERRIDE" ]; then
        echo "Error: --offline-correction uses its own preset; custom parameters and normal/correspondence/initialization overrides are unsupported." >&2
        exit 1
    fi
    OFFLINE_ARGS=(--rviz)
    [ "$RECONSTRUCTION_MODE" != livo ] || OFFLINE_ARGS+=(--livo)
    [ "$RETURN_TO_START" -eq 0 ] || OFFLINE_ARGS+=(--return-to-start)
    [ "$CAPTURE_DIAGNOSTICS" -eq 0 ] || OFFLINE_ARGS+=(--capture-diagnostics)
    [ -z "$TEMPORAL_WINDOW" ] || OFFLINE_ARGS+=(--temporal-window "$TEMPORAL_WINDOW")
    # Direct bag input: only pacing is shared with ros2 bag play.
    for ((i=0; i<${#EXTRA_ARGS[@]}; i++)); do
        case "${EXTRA_ARGS[i]}" in
            --rate|-r)
                ((i+=1))
                [ "$i" -lt "${#EXTRA_ARGS[@]}" ] || { echo "Missing rate" >&2; exit 1; }
                OFFLINE_ARGS+=(--rate "${EXTRA_ARGS[i]}") ;;
            --rate=*) OFFLINE_ARGS+=(--rate "${EXTRA_ARGS[i]#*=}") ;;
            *) echo "Unsupported bag-player argument in --offline-correction mode: ${EXTRA_ARGS[i]}" >&2; exit 1 ;;
        esac
    done
    OUTPUT_DIR=${OUTPUT_DIR:-${CAPTURE_ROOT:-$REPO_ROOT/reconstruction_results}/$(basename "$BAG")-rviz-$(date +%Y%m%d-%H%M%S)-$$}
    echo "Fresh estimation -> offline correction -> RViz comparison: $OUTPUT_DIR"
    exec bash "$SCRIPT_DIR/reconstruct_offline.sh" "$BAG" "$OUTPUT_DIR" "${OFFLINE_ARGS[@]}"
fi
if [ "$RETURN_TO_START" -eq 1 ] || [ -n "$OUTPUT_DIR" ]; then
    echo "Error: --return-to-start and --output-dir require --offline-correction." >&2; exit 1
fi
ROS_SETUP="${FAST_LIVO2_ROS_SETUP:-/opt/ros/humble/setup.bash}"
CAPTURE_HELPER="$SCRIPT_DIR/reconstruction_capture.py"
RVIZ_CFG="$REPO_ROOT/config/fast_livo2/fast_livo2.rviz"
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/humble_ws/install/setup.bash"
    "$REPO_ROOT/.devcontainer/offline_fast_livo2/install/setup.bash"
)
REJECTED_WS_SETUPS=()

setup_provides_fast_livo2() {
    local setup_script="$1"

    env \
        -u AMENT_PREFIX_PATH \
        -u CMAKE_PREFIX_PATH \
        -u COLCON_PREFIX_PATH \
        -u LD_LIBRARY_PATH \
        -u PYTHONPATH \
        bash -c '
        set +u
        source "$1" >/dev/null 2>&1 || exit 1
        source "$2" >/dev/null 2>&1 || exit 1
        ros2 pkg prefix fast_livo2 >/dev/null 2>&1
    ' bash "$ROS_SETUP" "$setup_script"
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
RAW_TOPICS=()

if has_topic /livox/lidar && has_topic /livox/imu; then
    SENSOR_PROFILE="livox_mid360"
    PARAMS_FILE="$REPO_ROOT/humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml"
    RAW_TOPICS=(/livox/lidar /livox/imu)
elif has_topic /hesai_node/points_raw && has_topic /go2w/imu; then
    SENSOR_PROFILE="hesai_go2w_imu"
    PARAMS_FILE="$REPO_ROOT/humble_ws/src/fast-livo2/config/go2w_hesai_fast_livo2.yaml"
    RAW_TOPICS=(/hesai_node/points_raw /go2w/imu)
else
    echo "Error: bag is missing a supported LiDAR/IMU topic pair." >&2
    echo "Expected either:" >&2
    echo "  - /livox/lidar and /livox/imu" >&2
    echo "  - /hesai_node/points_raw and /go2w/imu" >&2
    print_bag_topics
    exit 1
fi

if [ -n "$PARAMS_OVERRIDE" ]; then
    PARAMS_FILE=$(realpath -e "$PARAMS_OVERRIDE") || exit 1
    python3 - "$PARAMS_FILE" "${RAW_TOPICS[0]}" "${RAW_TOPICS[1]}" <<'PY'
import sys,yaml
p=yaml.safe_load(open(sys.argv[1]))['laserMapping']['ros__parameters']
common=p['common']
if (common['lid_topic'],common['imu_topic']) != tuple(sys.argv[2:]):
    raise ValueError('--params LiDAR/IMU topics do not match the detected bag profile')
PY
fi

if [ "$RECONSTRUCTION_MODE" = "livo" ] && ! has_topic /camera/camera/color/image_raw; then
    echo "Error: bag is missing required topic: /camera/camera/color/image_raw" >&2
    print_bag_topics
    exit 1
fi

if [ "$RECONSTRUCTION_MODE" = "livo" ]; then
    RAW_TOPICS+=(/camera/camera/color/image_raw)
    if has_topic /camera/camera/color/camera_info; then
        RAW_TOPICS+=(/camera/camera/color/camera_info)
    fi
fi

CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then
    CLOCK_ARG=()
    RAW_TOPICS+=(/clock)
fi

has_bag_play_delay_arg=0
has_bag_play_rate_arg=0
for arg in "${EXTRA_ARGS[@]}"; do
    case "$arg" in
        --delay|-d|--delay=*)
            has_bag_play_delay_arg=1
            ;;
        --rate|-r|--rate=*)
            has_bag_play_rate_arg=1
            ;;
    esac
done

BAG_PLAY_ARGS=("$BAG" "${CLOCK_ARG[@]}")
if [ "$has_bag_play_delay_arg" -eq 0 ]; then
    BAG_PLAY_ARGS+=(--delay "$DEFAULT_BAG_PLAY_DELAY")
fi
if [ "$has_bag_play_rate_arg" -eq 0 ]; then
    BAG_PLAY_ARGS+=(--rate "$DEFAULT_BAG_PLAY_RATE")
fi
BAG_PLAY_ARGS+=("${EXTRA_ARGS[@]}")
BAG_PLAY_ARGS+=(--topics "${RAW_TOPICS[@]}")

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ ! -f "$candidate" ]; then
        continue
    fi

    if setup_provides_fast_livo2 "$candidate"; then
        WS_SETUP="$candidate"
        break
    fi

    REJECTED_WS_SETUPS+=("$candidate")
done

if [ -z "$WS_SETUP" ]; then
    echo "Error: no usable FAST-LIVO2 workspace setup was found." >&2
    if [ "${#REJECTED_WS_SETUPS[@]}" -gt 0 ]; then
        echo "Found but rejected because fast_livo2 is not discoverable:" >&2
        for candidate in "${REJECTED_WS_SETUPS[@]}"; do
            echo "  - $candidate" >&2
        done
    fi
    echo "Checked:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "" >&2
    echo "A humble_ws/install built in the robot container contains /external paths" >&2
    echo "and must be used through the container mount. From the repository root, run:" >&2
    echo "  bash docker/robot/run.sh bash scripts/fast_livo2/reconstruct_raw.sh \"$BAG\"" >&2
    echo "" >&2
    echo "For the desktop Dev Container, rebuild its local offline workspace first." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix fast_livo2 >/dev/null 2>&1; then
    echo "Error: fast_livo2 is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    exit 1
fi

if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION="${FAST_LIVO2_RECONSTRUCTION_RMW:-rmw_cyclonedds_cpp}"
fi
export ROS_DOMAIN_ID="${FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${FAST_LIVO2_RECONSTRUCTION_LOCALHOST_ONLY:-1}"
DETERMINISTIC_PROCESSING="${FAST_LIVO2_RECONSTRUCTION_DETERMINISTIC:-1}"
if [ "$DETERMINISTIC_PROCESSING" = "1" ]; then
    export FAST_LIVO2_DETERMINISTIC=1
else
    unset FAST_LIVO2_DETERMINISTIC
fi
if [ "${FAST_LIVO2_RECONSTRUCTION_KEEP_CYCLONEDDS_URI:-0}" != "1" ]; then
    unset CYCLONEDDS_URI
fi

# Check the live graph, not the CLI daemon's cached node list. A previous run
# can survive its terminal and answer parameter services for the same name;
# dump then mixes parameter names/values from different estimator processes.
# Do this before starting processes or creating/routing any capture logs.
if ! existing_nodes=$(timeout 8 ros2 node list --no-daemon --spin-time 2); then
    echo "Error: cannot inspect ROS nodes in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; startup aborted." >&2
    exit 1
fi
if grep -qxF '/laserMapping' <<< "$existing_nodes"; then
    echo "Error: /laserMapping already exists in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; startup aborted before launch/playback." >&2
    echo "Stop the previous reconstruction, or choose an unused domain with FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID=<id>." >&2
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

wait_group() {
    local pgid="$1"
    local seconds="$2"
    for ((i=0; i<seconds; i++)); do
        is_process_group_alive "$pgid" || return 0
        sleep 1
    done
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    [ "$CLEANUP_STARTED" -eq 0 ] || return "$status"
    CLEANUP_STARTED=1
    set +e
    echo "Stopping raw-bag FAST-LIVO2 reconstruction..."
    terminate_process_group TERM "${RVIZ_PGID:-}"
    # Stop input first, then let subscribers drain before closing their files.
    terminate_process_group INT "${BAG_PGID:-}"
    wait_group "${BAG_PGID:-}" 3
    if [ -n "$CAPTURE_DIR" ] && [ -n "${BAG_PID:-}" ]; then sleep 1; fi
    terminate_process_group INT "${FAST_LIVO2_PGID:-}"
    terminate_process_group INT "${MONITOR_PGID:-}"
    for pgid in "${FAST_LIVO2_PGID:-}" "${MONITOR_PGID:-}" "${RVIZ_PGID:-}"; do
        wait_group "$pgid" 5
    done
    for pgid in "${RVIZ_PGID:-}" "${BAG_PGID:-}" "${FAST_LIVO2_PGID:-}" "${MONITOR_PGID:-}"; do
        if is_process_group_alive "$pgid"; then
            [ -z "$CAPTURE_DIR" ] || echo "Forced kill of process group $pgid" >> "$CAPTURE_DIR/capture_error.txt"
            terminate_process_group KILL "$pgid"
        fi
    done
    for pid in "${RVIZ_PID:-}" "${BAG_PID:-}" "${FAST_LIVO2_PID:-}"; do
        [ -z "$pid" ] || wait "$pid" 2>/dev/null
    done
    if [ -n "${MONITOR_PID:-}" ]; then
        local monitor_status=0
        wait "$MONITOR_PID" 2>/dev/null || monitor_status=$?
        echo "{\"returncode\":$monitor_status}" > "$CAPTURE_DIR/monitor.exit.json"
    fi
    if [ -n "$CAPTURE_DIR" ]; then
        [ "$status" -eq 0 ] || [ "$STOP_REASON" = interrupt ] || STOP_REASON="failure: $STOP_REASON (shell $status)"
        python3 "$CAPTURE_HELPER" finalize --capture "$CAPTURE_DIR" --reason "$STOP_REASON"
        local capture_status=$?
        if [ "$capture_status" -ne 0 ]; then
            echo "Capture incomplete; inspect summary.json and console logs in $CAPTURE_DIR" >&2
            if [ "$status" -eq 0 ]; then status=1; fi
        fi
        echo "Reconstruction files: $CAPTURE_DIR"
    fi
    [ -z "$TEMP_PARAMS" ] || rm -f -- "$TEMP_PARAMS"
    exit "$status"
}

trap cleanup EXIT
trap 'STOP_REASON=interrupt; exit 130' INT TERM

NORMAL_OVERRIDE=()
[ -z "$NORMAL_GATE" ] || NORMAL_OVERRIDE=(--normal-gate "$NORMAL_GATE")
[ -z "$CORRESPONDENCE_MODE" ] || NORMAL_OVERRIDE+=(--correspondence-mode "$CORRESPONDENCE_MODE")
[ -z "$TEMPORAL_WINDOW" ] || NORMAL_OVERRIDE+=(--temporal-window "$TEMPORAL_WINDOW")
[ -z "$TEMPORAL_INITIALIZATION" ] || NORMAL_OVERRIDE+=(--temporal-initialization "$TEMPORAL_INITIALIZATION")
if [ "$CAPTURE_DIAGNOSTICS" -eq 1 ]; then
    CAPTURE_ROOT="${CAPTURE_ROOT:-$REPO_ROOT/diagnostic_captures}"
    BINARY="$(ros2 pkg prefix fast_livo2)/lib/fast_livo2/fastlivo2_mapping"
    CAPTURE_DIR=$(python3 "$CAPTURE_HELPER" prepare --root "$CAPTURE_ROOT" --repo "$REPO_ROOT" \
        --bag "$BAG" --params "$PARAMS_FILE" --binary "$BINARY" --img-en "$FAST_LIVO2_IMG_EN" \
        "${NORMAL_OVERRIDE[@]}" -- "${ORIGINAL_ARGS[@]}")
    PARAMS_FILE="$CAPTURE_DIR/params.yaml"
    echo "Capture directory: $CAPTURE_DIR"
    echo "Detail/geometry window: 0 <= t < 360 seconds from first LiDAR; playback is not stopped at 360."
    echo "Full captures can exceed 10 GB. Available disk space:"
    df -h "$CAPTURE_DIR"
elif [ -n "$NORMAL_GATE$CORRESPONDENCE_MODE$TEMPORAL_WINDOW$TEMPORAL_INITIALIZATION$PARAMS_OVERRIDE" ]; then
    TEMP_PARAMS=$(mktemp /tmp/fast-livo2-matching-params.XXXXXX.yaml)
    python3 "$CAPTURE_HELPER" derive --params "$PARAMS_FILE" --output "$TEMP_PARAMS" --img-en "$FAST_LIVO2_IMG_EN" "${NORMAL_OVERRIDE[@]}"
    PARAMS_FILE="$TEMP_PARAMS"
fi

start_process() {
    local name="$1"; shift
    if [ -n "$CAPTURE_DIR" ]; then
        setsid python3 "$CAPTURE_HELPER" run --capture "$CAPTURE_DIR" --name "$name" -- "$@" &
    else
        setsid "$@" &
    fi
    STARTED_PID=$!
}

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "FAST-LIVO2 setup: $WS_SETUP"
echo "Sensor profile: $SENSOR_PROFILE"
echo "Raw topics: ${RAW_TOPICS[*]}"
echo "FAST-LIVO2 params: $PARAMS_FILE"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-system default}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo "Deterministic processing: $DETERMINISTIC_PROCESSING"
if [ "$RECONSTRUCTION_MODE" = "lio" ]; then
    echo "Reconstruction mode: LIO-only (VIO disabled, common.img_en=0)"
else
    echo "Reconstruction mode: LIVO (LiDAR + IMU + camera)"
fi
if [ "$has_bag_play_delay_arg" -eq 0 ]; then
    echo "Bag play delay: ${DEFAULT_BAG_PLAY_DELAY}s"
else
    echo "Bag play delay: set by ros2 bag play arguments"
fi
if [ "$has_bag_play_rate_arg" -eq 0 ]; then
    echo "Bag play rate: ${DEFAULT_BAG_PLAY_RATE}x"
else
    echo "Bag play rate: set by ros2 bag play arguments"
fi
echo "Mode: replay raw topics, run FAST-LIVO2 offline, visualize generated outputs"
echo ""

start_process node ros2 launch fast_livo2 go2w_fast_livo2.launch.py \
    use_sensor_bringup:=false \
    use_sim_time:=true \
    reliable_sensor_qos:=true \
    img_en:="$FAST_LIVO2_IMG_EN" \
    params_file:="$PARAMS_FILE"
FAST_LIVO2_PID=$STARTED_PID
FAST_LIVO2_PGID=$FAST_LIVO2_PID

sleep 3
if ! kill -0 "$FAST_LIVO2_PID" 2>/dev/null; then
    echo "Error: FAST-LIVO2 exited during startup." >&2
    wait "$FAST_LIVO2_PID" || true
    exit 1
fi

if [ -n "$CAPTURE_DIR" ]; then
    STOP_REASON="waiting for effective parameters"
    params_ready=0
    : > "$CAPTURE_DIR/param_dump.stderr"
    for attempt in {1..10}; do
        echo "Parameter dump attempt $attempt (direct discovery)" >> "$CAPTURE_DIR/param_dump.stderr"
        # Avoid stale/broken CLI daemons. Allow time for discovery plus the
        # per-parameter service requests made by Humble's dump command.
        if timeout 15 ros2 param dump /laserMapping --no-daemon --spin-time 2 \
            > "$CAPTURE_DIR/effective_params.yaml.tmp" 2>> "$CAPTURE_DIR/param_dump.stderr"; then
            mv "$CAPTURE_DIR/effective_params.yaml.tmp" "$CAPTURE_DIR/effective_params.yaml"
            params_ready=1; break
        else
            dump_status=$?
            echo "Parameter dump attempt $attempt failed (exit $dump_status)" >> "$CAPTURE_DIR/param_dump.stderr"
        fi
        kill -0 "$FAST_LIVO2_PID" 2>/dev/null || break
        sleep 1
    done
    if [ "$params_ready" -ne 1 ]; then
        echo "Error: effective parameter dump unavailable; playback aborted. See $CAPTURE_DIR/param_dump.stderr" >&2
        tail -n 12 "$CAPTURE_DIR/param_dump.stderr" >&2
        exit 1
    fi
    python3 "$CAPTURE_HELPER" verify --capture "$CAPTURE_DIR" --effective "$CAPTURE_DIR/effective_params.yaml"
    STOP_REASON="starting IMU monitor"
    setsid python3 "$CAPTURE_HELPER" monitor --capture "$CAPTURE_DIR" --topic "${RAW_TOPICS[1]}" \
        > "$CAPTURE_DIR/monitor.console.log" 2>&1 &
    MONITOR_PID=$!; MONITOR_PGID=$MONITOR_PID
    for attempt in {1..50}; do
        [ ! -f "$CAPTURE_DIR/monitor.ready" ] || break
        kill -0 "$MONITOR_PID" 2>/dev/null || break
        sleep .2
    done
    if [ ! -f "$CAPTURE_DIR/monitor.ready" ] || ! kill -0 "$MONITOR_PID" 2>/dev/null; then
        echo "Error: IMU monitor not ready; playback aborted. See $CAPTURE_DIR/monitor.console.log" >&2
        exit 1
    fi
fi

STOP_REASON="running"
start_process player ros2 bag play "${BAG_PLAY_ARGS[@]}"
BAG_PID=$STARTED_PID
BAG_PGID=$BAG_PID

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

start_process rviz rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
RVIZ_PID=$STARTED_PID
RVIZ_PGID=$RVIZ_PID

if [ -n "$CAPTURE_DIR" ]; then
    player_finished=0
    while kill -0 "$RVIZ_PID" 2>/dev/null; do
        if ! kill -0 "$FAST_LIVO2_PID" 2>/dev/null || ! kill -0 "$MONITOR_PID" 2>/dev/null || [ -f "$CAPTURE_DIR/capture_error.txt" ]; then
            STOP_REASON="failure: node, monitor, or capture writer stopped"
            exit 1
        fi
        if [ "$player_finished" -eq 0 ] && ! kill -0 "$BAG_PID" 2>/dev/null; then
            if wait "$BAG_PID"; then
                player_finished=1
                echo "Bag playback finished. Close RViz to finish capture."
            else
                STOP_REASON="failure: bag player exited"; exit 1
            fi
        fi
        sleep 1
    done
fi
STOP_REASON="RViz closed"
wait "$RVIZ_PID"
