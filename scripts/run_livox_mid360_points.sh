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

read_mid360_network_config() {
    local config_file="$1"

    if ! command -v python3 >/dev/null 2>&1; then
        return 1
    fi

    python3 - "$config_file" <<'PY'
import json
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
try:
    config = json.loads(config_path.read_text())
except Exception as exc:
    print(f"failed to parse {config_path}: {exc}", file=sys.stderr)
    sys.exit(2)

mid360 = config.get("MID360", {})
host = mid360.get("host_net_info", {})

host_ips = []
for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip", "log_data_ip"):
    value = str(host.get(key, "")).strip()
    if value and value not in host_ips:
        host_ips.append(value)

host_ports = []
for key in ("cmd_data_port", "push_msg_port", "point_data_port", "imu_data_port", "log_data_port"):
    try:
        value = int(host.get(key, 0))
    except (TypeError, ValueError):
        continue
    if value > 0 and value not in host_ports:
        host_ports.append(value)

lidar_ips = []
for lidar_config in config.get("lidar_configs", []):
    value = str(lidar_config.get("ip", "")).strip()
    if value and value not in lidar_ips:
        lidar_ips.append(value)

print(" ".join(host_ips))
print(" ".join(str(port) for port in host_ports))
print(" ".join(lidar_ips))
PY
}

host_ip_is_assigned() {
    local host_ip="$1"

    if [ "$host_ip" = "0.0.0.0" ]; then
        return 0
    fi

    command -v ip >/dev/null 2>&1 || return 0

    ip -o -4 addr show | awk -v check_ip="$host_ip" '
        {
            split($4, address, "/")
            if (address[1] == check_ip) {
                found = 1
            }
        }
        END {
            exit(found ? 0 : 1)
        }
    '
}

print_available_ipv4_addresses() {
    command -v ip >/dev/null 2>&1 || return 0

    ip -o -4 addr show | awk '{ print "  " $2 ": " $4 }' >&2 || true
}

find_udp_port_conflicts() {
    local host_ports="$1"
    local host_port

    command -v ss >/dev/null 2>&1 || return 0

    for host_port in $host_ports; do
        ss -H -lunp 2>/dev/null | awk -v port="$host_port" '
            $4 ~ ":" port "$" {
                print
            }
        '
    done
}

preflight_livox_network_config() {
    local config_file="$1"
    local parsed_config host_ips host_ports lidar_ips host_ip conflicts
    local parsed_lines=()

    echo "Using MID360 config: ${config_file}" >&2

    if ! parsed_config="$(read_mid360_network_config "$config_file")"; then
        echo "Warning: could not parse MID360 network config for preflight checks." >&2
        return 0
    fi

    mapfile -t parsed_lines <<< "$parsed_config"
    host_ips="${parsed_lines[0]:-}"
    host_ports="${parsed_lines[1]:-}"
    lidar_ips="${parsed_lines[2]:-}"

    if [ -n "$host_ips" ]; then
        echo "Configured MID360 host IP(s): ${host_ips}" >&2
    fi
    if [ -n "$lidar_ips" ]; then
        echo "Configured MID360 LiDAR IP(s): ${lidar_ips}" >&2
    fi

    for host_ip in $host_ips; do
        if ! host_ip_is_assigned "$host_ip"; then
            echo "Error: MID360 host IP is not assigned on this host: ${host_ip}" >&2
            echo "The Livox SDK cannot bind UDP sockets to an IP address that is not on a local interface." >&2
            echo "Available IPv4 addresses:" >&2
            print_available_ipv4_addresses
            echo "Fix the network config, for example:" >&2
            echo "  sudo ip addr add ${host_ip}/24 dev <livox_ethernet_interface>" >&2
            echo "or set LIVOX_MID360_CONFIG to a MID360_config.json whose host_net_info IP matches this host." >&2
            exit 1
        fi
    done

    conflicts="$(find_udp_port_conflicts "$host_ports" || true)"
    if [ -n "$conflicts" ]; then
        echo "Error: one or more Livox UDP host ports are already in use." >&2
        echo "$conflicts" | sed 's/^/  /' >&2
        echo "Stop the existing Livox driver before starting a new one, for example:" >&2
        echo "  pkill -f livox_ros_driver2_node" >&2
        exit 1
    fi
}

driver_pid=""
driver_started_with_setsid=0

terminate_driver() {
    local signal="$1"

    if [ -z "$driver_pid" ]; then
        return 0
    fi

    if [ "$driver_started_with_setsid" -eq 1 ]; then
        kill "-${signal}" "-${driver_pid}" 2>/dev/null || true
    else
        kill "-${signal}" "${driver_pid}" 2>/dev/null || true
    fi
}

shutdown_driver() {
    local exit_status="$1"
    local attempt

    trap - INT TERM
    echo "Stopping livox_ros_driver2..." >&2

    terminate_driver TERM
    for attempt in {1..30}; do
        if [ -z "$driver_pid" ] || ! kill -0 "$driver_pid" 2>/dev/null; then
            exit "$exit_status"
        fi
        sleep 0.2
    done

    echo "livox_ros_driver2 did not exit after SIGTERM; sending SIGKILL." >&2
    terminate_driver KILL
    wait "$driver_pid" 2>/dev/null || true
    exit "$exit_status"
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
        "${REPO_ROOT}/config/livox/MID360_config.json"
        "/external/config/livox/MID360_config.json"
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

preflight_livox_network_config "$livox_config"

driver_cmd=(
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args
    -r __node:=livox_lidar_publisher
    -p "xfer_format:=${LIVOX_XFER_FORMAT:-1}"
    -p "multi_topic:=${LIVOX_MULTI_TOPIC:-0}"
    -p "data_src:=${LIVOX_DATA_SRC:-0}"
    -p "publish_freq:=${LIVOX_PUBLISH_FREQ:-10.0}"
    -p "output_data_type:=${LIVOX_OUTPUT_DATA_TYPE:-0}"
    -p "frame_id:=${LIVOX_FRAME_ID:-livox_frame}"
    -p "user_config_path:=${livox_config}"
    -p "cmdline_input_bd_code:=${LIVOX_BD_CODE:-livox0000000001}"
)

trap 'shutdown_driver 130' INT
trap 'shutdown_driver 143' TERM

if command -v setsid >/dev/null 2>&1; then
    setsid "${driver_cmd[@]}" &
    driver_started_with_setsid=1
else
    "${driver_cmd[@]}" &
fi
driver_pid="$!"

set +e
wait "$driver_pid"
driver_status="$?"
set -e

driver_pid=""
exit "$driver_status"
