#!/usr/bin/env bash
# Burger1 already has the calibrated Raspberry Pi libcamera cam runtime.
set -euo pipefail
CAMERA_BINARY="${CAMERA_BINARY:-$HOME/camera-build/libcamera/build/src/apps/cam/cam}"
if [[ ! -x "$CAMERA_BINARY" ]]; then
  echo "Missing Burger1 runtime: $CAMERA_BINARY. Restore the runtime used for calibration." >&2
  exit 1
fi
echo "Burger1 camera runtime found: $CAMERA_BINARY"
echo "Use start_camera.sh; no camera_ros rebuild is needed."
