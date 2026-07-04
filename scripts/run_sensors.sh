#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_DIR}/humble_ws"

set +u
source /opt/ros/humble/setup.bash
set -u

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "Workspace is not built yet. Run scripts/build_humble_ws.sh first." >&2
  exit 1
fi

set +u
source "${WS_DIR}/install/setup.bash"
set -u

exec ros2 launch fast_livo2 go2w_sensors.launch.py "$@"
