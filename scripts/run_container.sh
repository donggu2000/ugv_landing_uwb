#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

IMAGE_NAME="${IMAGE_NAME:-ugv_landing_sim:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-ugv_landing_sim}"
WORKSPACE_DIR="$(pwd)"

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "Image $IMAGE_NAME not found. Building it first..."
  docker build -t "$IMAGE_NAME" -f docker/Dockerfile .
fi

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker exec -it "$CONTAINER_NAME" /bin/bash
  exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker start -ai "$CONTAINER_NAME"
  exit 0
fi

xhost +local:docker >/dev/null 2>&1 || true
xhost +local:root >/dev/null 2>&1 || true

docker run -it \
  --name "$CONTAINER_NAME" \
  --gpus all \
  --privileged \
  --net=host \
  --ipc=host \
  --env="DISPLAY=${DISPLAY:-:0}" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="XDG_RUNTIME_DIR=/tmp/runtime-root" \
  --env="GZ_SIM_RESOURCE_PATH=/workspaces/ugv_landing_sim/src/ugv_landing_sim/worlds:/workspaces/ugv_landing_sim/src/ugv_landing_sim/models" \
  --env="IGN_GAZEBO_RESOURCE_PATH=/workspaces/ugv_landing_sim/src/ugv_landing_sim/worlds:/workspaces/ugv_landing_sim/src/ugv_landing_sim/models" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v "$WORKSPACE_DIR":/workspaces/ugv_landing_sim \
  -v "$HOME/.Xauthority":/root/.Xauthority:ro \
  -w /workspaces/ugv_landing_sim \
  "$IMAGE_NAME" \
  /bin/bash
