#!/bin/bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export DOCKER_API_VERSION="${DOCKER_API_VERSION:-1.43}"

IMAGE="${FAST_LIVO2_GO2W_IMAGE:-fast-livo2-go2w:latest}"
PLATFORM="${FAST_LIVO2_GO2W_PLATFORM:-}"

platform_args=()
if [ -n "$PLATFORM" ]; then
  platform_args+=(--platform "$PLATFORM")
fi

docker build \
  "${platform_args[@]}" \
  -f "$SCRIPT_DIR/Dockerfile" \
  -t "$IMAGE" \
  "$REPO_ROOT"
