#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -n "${FAST_LIVO2_WS:-}" ] && [ -f "${FAST_LIVO2_WS}/install/setup.bash" ]; then
  source "${FAST_LIVO2_WS}/install/setup.bash"
fi

exec "$@"

