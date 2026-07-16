#!/bin/bash
# Run the Hesai XT16 driver for FAST-LIVO2 point-cloud input only.

set -eu
set -o pipefail

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found. Source the ROS 2 and workspace setup files first." >&2
    exit 1
fi

hesai_prefix="$(ros2 pkg prefix hesai_lidar)"
correction_file="${HESAI_CORRECTION_FILE:-$hesai_prefix/share/hesai_lidar/config/PandarXT-16.csv}"

if [ ! -f "$correction_file" ]; then
    echo "Error: Hesai correction file not found: $correction_file" >&2
    exit 1
fi

exec ros2 run hesai_lidar hesai_lidar_node --ros-args \
    -r __node:=hesai_node \
    -r points_raw:=/hesai_node/points_raw \
    -p server_ip:=192.168.123.20 \
    -p lidar_recv_port:=2368 \
    -p gps_port:=10110 \
    -p start_angle:=0.0 \
    -p lidar_type:=PandarXT-16 \
    -p frame_id:=hesai_lidar \
    -p pcldata_type:=0 \
    -p publish_type:=points \
    -p timestamp_type:=realtime \
    -p lidar_correction_file:="$correction_file" \
    -p coordinate_correction_flag:=false
