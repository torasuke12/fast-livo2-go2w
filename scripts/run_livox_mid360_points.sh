#!/bin/bash
# Run the Livox MID360 ROS 2 driver for FAST-LIVO2 CustomMsg input.

set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
WS_SETUP="${FAST_LIVO2_WS:-${REPO_ROOT}/humble_ws}/install/setup.bash"

source_setup() {
    local setup_file="$1"

    set +u
    # shellcheck source=/dev/null
    source "$setup_file"
    set -u
}

if ! command -v ros2 >/dev/null 2>&1 && [ -f "$ROS_SETUP" ]; then
    # Allow direct execution from a fresh shell.
    source_setup "$ROS_SETUP"
fi

if [ -f "$WS_SETUP" ]; then
    source_setup "$WS_SETUP"
elif [ -f /external/humble_ws/install/setup.bash ]; then
    source_setup /external/humble_ws/install/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found. Source the ROS 2 and workspace setup files first." >&2
    exit 1
fi

livox_prefix="$(ros2 pkg prefix livox_ros_driver2 2>/dev/null || true)"
if [ -z "$livox_prefix" ]; then
    echo "Error: livox_ros_driver2 is not available. Build the workspace with Livox driver first." >&2
    echo "Run inside the container:" >&2
    echo "  cd /external" >&2
    echo "  bash scripts/build_humble_ws.sh" >&2
    echo "  source humble_ws/install/setup.bash" >&2
    echo "Then check:" >&2
    echo "  ros2 pkg prefix livox_ros_driver2" >&2
    exit 1
fi

livox_config="${LIVOX_MID360_CONFIG:-}"
if [ -z "$livox_config" ]; then
    fast_livo2_prefix="$(ros2 pkg prefix fast_livo2 2>/dev/null || true)"
    config_candidates=(
        "${fast_livo2_prefix}/share/fast_livo2/config/MID360_config.json"
        "${REPO_ROOT}/humble_ws/src/fast-livo2/config/MID360_config.json"
        "/external/humble_ws/src/fast-livo2/config/MID360_config.json"
        "${livox_prefix}/share/livox_ros_driver2/config/MID360_config.json"
        "${REPO_ROOT}/humble_ws/src/livox_ros_driver2/config/MID360_config.json"
        "/external/humble_ws/src/livox_ros_driver2/config/MID360_config.json"
    )

    for candidate in "${config_candidates[@]}"; do
        if [ -f "$candidate" ]; then
            livox_config="$candidate"
            break
        fi
    done
fi

if [ ! -f "$livox_config" ]; then
    echo "Error: MID360 config file not found." >&2
    if [ -n "${LIVOX_MID360_CONFIG:-}" ]; then
        echo "Configured LIVOX_MID360_CONFIG does not exist: ${LIVOX_MID360_CONFIG}" >&2
    else
        echo "Checked:" >&2
        printf '  %s\n' "${config_candidates[@]}" >&2
    fi
    echo "Set LIVOX_MID360_CONFIG or rebuild FAST-LIVO2 after pulling the Livox config:" >&2
    echo "  cd /external" >&2
    echo "  bash scripts/build_humble_ws.sh" >&2
    exit 1
fi

exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -r __node:=livox_lidar_publisher \
    -p xfer_format:="${LIVOX_XFER_FORMAT:-1}" \
    -p multi_topic:="${LIVOX_MULTI_TOPIC:-0}" \
    -p data_src:="${LIVOX_DATA_SRC:-0}" \
    -p publish_freq:="${LIVOX_PUBLISH_FREQ:-10.0}" \
    -p output_data_type:="${LIVOX_OUTPUT_DATA_TYPE:-0}" \
    -p frame_id:="${LIVOX_FRAME_ID:-livox_frame}" \
    -p user_config_path:="$livox_config" \
    -p cmdline_input_bd_code:="${LIVOX_BD_CODE:-livox0000000001}"
