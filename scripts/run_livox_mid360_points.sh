#!/bin/bash
# Run the Livox MID360 ROS 2 driver for FAST-LIVO2 CustomMsg input.

set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
WS_SETUP="${FAST_LIVO2_WS:-${REPO_ROOT}/humble_ws}/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1 && [ -f "$ROS_SETUP" ]; then
    # Allow direct execution from a fresh shell.
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
fi

if [ -f "$WS_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
elif [ -f /external/humble_ws/install/setup.bash ]; then
    # shellcheck source=/dev/null
    source /external/humble_ws/install/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found. Source the ROS 2 and workspace setup files first." >&2
    exit 1
fi

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    echo "Error: livox_ros_driver2 is not available. Build the workspace with Livox driver first." >&2
    echo "Run inside the container:" >&2
    echo "  cd /external" >&2
    echo "  bash scripts/build_humble_ws.sh" >&2
    echo "  source humble_ws/install/setup.bash" >&2
    echo "Then check:" >&2
    echo "  ros2 pkg prefix livox_ros_driver2" >&2
    exit 1
fi

launch_args=(
    xfer_format:="${LIVOX_XFER_FORMAT:-1}"
    multi_topic:="${LIVOX_MULTI_TOPIC:-0}"
    data_src:="${LIVOX_DATA_SRC:-0}"
    publish_freq:="${LIVOX_PUBLISH_FREQ:-10.0}"
    output_data_type:="${LIVOX_OUTPUT_DATA_TYPE:-0}"
    frame_id:="${LIVOX_FRAME_ID:-livox_frame}"
    cmdline_input_bd_code:="${LIVOX_BD_CODE:-livox0000000001}"
)

if [ -n "${LIVOX_MID360_CONFIG:-}" ]; then
    launch_args+=(user_config_path:="${LIVOX_MID360_CONFIG}")
fi

exec ros2 launch fast_livo2 go2w_livox_mid360.launch.py "${launch_args[@]}"
