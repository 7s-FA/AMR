#!/usr/bin/env bash
# ========================================================================
# 역할: 카메라 라이브러리(libcamera 라즈베리파이판, camera_ros) 설치·빌드. 처음 설치 때 한 번 (robot.py camera-build).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Run on burger1 (Pi 5, Ubuntu 24.04 / ROS Jazzy).
# Install the Raspberry Pi fork in a private prefix, not /usr or /opt/ros.
set -eo pipefail
CAMERA_ROOT="${CAMERA_ROOT:-${AMR_CAMERA}}"
LIBCAMERA_COMMIT=6c1dd9d55573010f710c9e190a73e7e76f0d9432
CAMERA_ROS_COMMIT=f4023dc09cfc5fd36fedbb3656ad3a98733e297a
source /opt/ros/jazzy/setup.bash
sudo apt-get install -y git cmake g++ meson ninja-build pkg-config \
  libyaml-dev python3-yaml python3-ply python3-jinja2 libgnutls28-dev \
  libudev-dev libevent-dev libboost-dev libboost-log-dev nlohmann-json3-dev \
  ros-jazzy-camera-ros
mkdir -p "$CAMERA_ROOT/src" "$CAMERA_ROOT/build" "$CAMERA_ROOT/install"
checkout() {
  local url="$1" path="$2" commit="$3"
  if [[ ! -d "$path/.git" ]]; then
    git clone --depth 1 "$url" "$path"
  fi
  if [[ -n "$(git -C "$path" status --porcelain --untracked-files=no)" ]]; then
    echo "Refusing to overwrite modified source: $path" >&2
    exit 1
  fi
  git -C "$path" fetch --depth 1 origin "$commit"
  git -C "$path" checkout --detach "$commit"
}
checkout https://github.com/raspberrypi/libcamera.git \
  "$CAMERA_ROOT/src/libcamera" "$LIBCAMERA_COMMIT"
checkout https://github.com/christianrauch/camera_ros.git \
  "$CAMERA_ROOT/src/camera_ros" "$CAMERA_ROS_COMMIT"
reconfigure=()
if [[ -f "$CAMERA_ROOT/build/libcamera/meson-private/coredata.dat" ]]; then
  reconfigure=(--reconfigure)
fi
meson setup "${reconfigure[@]}" "$CAMERA_ROOT/build/libcamera" "$CAMERA_ROOT/src/libcamera" \
  --prefix="$CAMERA_ROOT/install/libcamera" --libdir=lib --buildtype=release \
  --force-fallback-for=libpisp \
  -Dpipelines=rpi/pisp -Dipas=rpi/pisp -Dcam=enabled -Dqcam=disabled \
  -Dgstreamer=disabled -Dpycamera=disabled -Ddocumentation=disabled \
  -Dtest=false -Dlc-compliance=disabled
ninja -C "$CAMERA_ROOT/build/libcamera" -j2
meson install -C "$CAMERA_ROOT/build/libcamera"
export PKG_CONFIG_PATH="$CAMERA_ROOT/install/libcamera/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$CAMERA_ROOT/install/libcamera/lib:${LD_LIBRARY_PATH:-}"
cmake -S "$CAMERA_ROOT/src/camera_ros" -B "$CAMERA_ROOT/build/camera_ros" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX="$CAMERA_ROOT/install/camera_ros"
cmake --build "$CAMERA_ROOT/build/camera_ros" -j2
cmake --install "$CAMERA_ROOT/build/camera_ros"
echo "Camera runtime installed under $CAMERA_ROOT. Run start_camera.sh."
