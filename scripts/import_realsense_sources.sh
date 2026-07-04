#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_DIR}/humble_ws"

mkdir -p "${WS_DIR}/src"

if ! command -v vcs >/dev/null 2>&1; then
  echo "vcs is required. In the Docker image it is provided by python3-vcstool." >&2
  exit 1
fi

vcs import "${WS_DIR}/src" < "${PROJECT_DIR}/dependencies.repos"

