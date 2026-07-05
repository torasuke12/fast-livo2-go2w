#!/bin/bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export DOCKER_API_VERSION="${DOCKER_API_VERSION:-1.43}"

IMAGE="${FAST_LIVO2_GO2W_IMAGE:-fast-livo2-go2w:latest}"
CONTAINER_NAME="${FAST_LIVO2_GO2W_CONTAINER:-fast-livo2-go2w}"
DOCKER_RUNTIME="${DOCKER_RUNTIME:-nvidia}"
XAUTH=/tmp/.docker.xauth

if [ -n "${DISPLAY:-}" ]; then
  xhost +local:docker >/dev/null 2>&1 || true

  if command -v xauth >/dev/null 2>&1; then
    touch "$XAUTH"
    xauth_list="$(xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^..../ffff/' || true)"
    if [ -n "$xauth_list" ]; then
      echo "$xauth_list" | xauth -f "$XAUTH" nmerge - >/dev/null 2>&1 || true
    fi
    chmod a+r "$XAUTH"
  fi
fi

runtime_args=()
if [ -n "$DOCKER_RUNTIME" ]; then
  runtime_args+=(--runtime="$DOCKER_RUNTIME")
fi

x11_args=()
if [ -n "${DISPLAY:-}" ]; then
  x11_args+=(
    --env="DISPLAY=$DISPLAY"
    --env="QT_X11_NO_MITSHM=1"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
  )
  if [ -f "$XAUTH" ]; then
    x11_args+=(
      --env="XAUTHORITY=$XAUTH"
      --volume="$XAUTH:$XAUTH:rw"
    )
  fi
fi

docker run -it --rm \
  --name "$CONTAINER_NAME" \
  --privileged \
  "${runtime_args[@]}" \
  --net=host \
  --ipc=host \
  --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  --env="RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  --env="FAST_LIVO2_WS=/external/humble_ws" \
  --env="CYCLONEDDS_URI=file:///external/humble_ws/src/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml" \
  "${x11_args[@]}" \
  --volume="/dev:/dev" \
  --volume="$REPO_ROOT:/external:rw" \
  --workdir="/external" \
  "$IMAGE" \
  "${@:-bash}"
