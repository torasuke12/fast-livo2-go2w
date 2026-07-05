#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Compatibility wrapper for users who still want Docker Compose.
# Some JetPack/Ubuntu Docker daemons only support Docker API 1.43. Newer
# Docker CLI/Compose releases may default to a newer API and fail before they
# can negotiate with the daemon.
export DOCKER_API_VERSION="${DOCKER_API_VERSION:-1.43}"

exec docker compose -f "${script_dir}/docker-compose.yml" "$@"
