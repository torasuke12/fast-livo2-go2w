#!/bin/bash
set -eu
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

IMAGE="${FAST_LIVO2_GO2W_IMAGE:-fast-livo2-go2w:latest}"
CONTAINER_NAME="${FAST_LIVO2_GO2W_CONTAINER:-fast-livo2-go2w}"
DOCKER_RUNTIME="${DOCKER_RUNTIME-auto}"
TEMP_XAUTH=""

cleanup() {
  if [ -n "$TEMP_XAUTH" ] && [ -f "$TEMP_XAUTH" ]; then
    rm -f -- "$TEMP_XAUTH"
  fi
}
trap cleanup EXIT

runtime_args=()
if [ "$DOCKER_RUNTIME" = "auto" ]; then
  docker_runtimes="$(docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
  if [[ "$docker_runtimes" == *'"nvidia"'* ]]; then
    DOCKER_RUNTIME="nvidia"
  else
    DOCKER_RUNTIME=""
  fi
fi
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

  if [ -n "${XAUTHORITY:-}" ] && [ -r "$XAUTHORITY" ]; then
    x11_args+=(
      --env="XAUTHORITY=/tmp/.host.xauthority"
      --volume="$XAUTHORITY:/tmp/.host.xauthority:ro"
    )
  elif command -v xauth >/dev/null 2>&1; then
    TEMP_XAUTH="$(mktemp /tmp/fast_livo2.xauth.XXXXXX)"
    xauth_list="$(xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' || true)"
    if [ -n "$xauth_list" ] && \
        printf '%s\n' "$xauth_list" | xauth -f "$TEMP_XAUTH" nmerge - >/dev/null 2>&1; then
      chmod a+r "$TEMP_XAUTH"
      x11_args+=(
        --env="XAUTHORITY=/tmp/.host.xauthority"
        --volume="$TEMP_XAUTH:/tmp/.host.xauthority:ro"
      )
    else
      echo "Warning: no X11 authorization cookie was available for DISPLAY=$DISPLAY." >&2
      echo "RViz2 may not start. Check DISPLAY and XAUTHORITY in the host terminal." >&2
    fi
  else
    echo "Warning: XAUTHORITY is unavailable and xauth is not installed." >&2
    echo "RViz2 may not start." >&2
  fi
fi

livox_driver_host="${LIVOX_ROS_DRIVER2_HOST:-$REPO_ROOT/../livox_ros_driver2}"
livox_args=()
if [ -d "$livox_driver_host" ]; then
  livox_args+=(--volume="$livox_driver_host:/fast_livo2/src/livox_ros_driver2:rw")
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
  "${livox_args[@]}" \
  --volume="/dev:/dev" \
  --volume="$REPO_ROOT:/external:rw" \
  --workdir="/external" \
  "$IMAGE" \
  "${@:-bash}"
