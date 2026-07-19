#!/bin/bash
# Run the Livox MID360 ROS 2 driver for FAST-LIVO2 CustomMsg input.

set -eu
set -o pipefail

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found. Source the ROS 2 and workspace setup files first." >&2
    exit 1
fi

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    echo "Error: livox_ros_driver2 is not available. Build the workspace with Livox driver first." >&2
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
