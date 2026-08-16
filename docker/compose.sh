#!/usr/bin/env bash
set -eu
set -o pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Compatibility wrapper for users who still want Docker Compose. Leave
# DOCKER_API_VERSION unset by default so the client can negotiate with both
# older and newer daemons. Pin it in the calling environment only when needed.

if [ -z "${FAST_LIVO2_XAUTHORITY:-}" ]; then
  if [ -n "${XAUTHORITY:-}" ] && [ -r "$XAUTHORITY" ]; then
    export FAST_LIVO2_XAUTHORITY="$XAUTHORITY"
  else
    export FAST_LIVO2_XAUTHORITY=/dev/null
  fi
fi

exec docker compose -f "${script_dir}/docker-compose.yml" "$@"
