#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-object_detection}"
CONTAINER_NAME="${CONTAINER_NAME:-camera}"
SHARED_DIR="${SHARED_DIR:-/home/lab/object_Detection_shared}"
HOST_NAME="${HOST_NAME:-host-pc}"
HOST_IP="${HOST_IP:-192.168.12.162}"

sudo docker run -it --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  --name "$CONTAINER_NAME" \
  --add-host "$HOST_NAME:$HOST_IP" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -v "$SHARED_DIR:/home" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/local/zed/resources:/usr/local/zed/resources \
  -v /dev/bus/usb:/dev/bus/usb \
  "$IMAGE"
