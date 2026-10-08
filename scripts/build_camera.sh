#!/usr/bin/env bash
set -eo pipefail
[[ $# == 1 ]] || { echo 'Usage: build_camera.sh RUNTIME_DIRECTORY' >&2; exit 2; }
source "$1/runtime.env"
export PKG_CONFIG_PATH="$AMR_CAMERA/install/libcamera/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$AMR_CAMERA/install/libcamera/lib:${LD_LIBRARY_PATH:-}"
if [[ ! -f "$AMR_CAMERA/install/libcamera/lib/pkgconfig/libcamera.pc" ]]; then
  bash "$AMR_CAMERA/setup_camera.sh"
fi
g++ -std=c++17 -O2 -pthread "$AMR_CAMERA/native_camera.cpp" \
  $(pkg-config --cflags --libs libcamera) -o "$AMR_CAMERA/native_camera"
echo "Camera executable built: $AMR_CAMERA/native_camera"
