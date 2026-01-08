#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

IMAGE_NAME="d415-rviz-test"

echo "[start] Building ${IMAGE_NAME} with --network host (required on this setup)"
docker build --network host -t "${IMAGE_NAME}" .

echo "[start] Starting container..."
docker run --rm -it --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v $PWD/workspace/src:/workspace/src/user_code \
  -v /dev/bus/usb:/dev/bus/usb \
  ${IMAGE_NAME}
