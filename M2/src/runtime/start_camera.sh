#!/usr/bin/env bash
# Run ON burger2 after setup_camera.sh; see docs/DOCKING_VISION.md.
set -eo pipefail
# Low-rate standby; terminal controller restores the original 15 fps first.
CAMERA_PROFILE="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/burger2-camera-profile"
if [[ "$(cat "$CAMERA_PROFILE" 2>/dev/null || echo idle)" == active ]];then
 export CAMERA_FRAME_US=66667 CAMERA_CAPTURE_FPS=15
else
 export CAMERA_FRAME_US=500000 CAMERA_CAPTURE_FPS=2
fi
source /opt/ros/jazzy/setup.bash
CAMERA_ROOT="${CAMERA_ROOT:-$HOME/final_robot_camera}"
if [[ ! -f "$CAMERA_ROOT/install/camera_ros/share/camera_ros/local_setup.bash" ||
      ! -f "$CAMERA_ROOT/install/libcamera/lib/libcamera.so" ]]; then
  echo "Missing private camera runtime. Run: bash $CAMERA_ROOT/setup_camera.sh" >&2
  exit 1
fi
source "$CAMERA_ROOT/install/camera_ros/share/camera_ros/local_setup.bash"
export LD_LIBRARY_PATH="$CAMERA_ROOT/install/libcamera/lib:${LD_LIBRARY_PATH:-}"
export LIBCAMERA_IPA_MODULE_PATH="$CAMERA_ROOT/install/libcamera/lib/libcamera/ipa"
export LIBCAMERA_IPA_PROXY_PATH="$CAMERA_ROOT/install/libcamera/libexec/libcamera"
export LIBCAMERA_IPA_CONFIG_PATH="$CAMERA_ROOT/install/libcamera/share/libcamera/ipa"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$SCRIPT_DIR/docking_network.py" && -f "$SCRIPT_DIR/docking.yaml" ]]; then
  /usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
    --role robot --local-shm --output "$SCRIPT_DIR/dds_camera.xml"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_camera.xml"
fi
exec ros2 run camera_ros camera_node --ros-args \
  -r __ns:=/burger2 -r __node:=camera \
  -p camera:="${CAMERA_ID:-0}" -p width:=640 -p height:=480 \
  -p sensor_mode:="${CAMERA_SENSOR_MODE:-1640:1232}" \
  -p format:=BGR888 -p frame_id:=burger2_camera_optical_frame \
  -p FrameDurationLimits:="[${CAMERA_FRAME_US:-66667}, ${CAMERA_FRAME_US:-66667}]" \
  -p jpeg_quality:="${CAMERA_JPEG_QUALITY:-70}" \
  -p qos_overrides./burger2/camera/image_raw.publisher.reliability:=best_effort \
  -p qos_overrides./burger2/camera/image_raw.publisher.depth:=1 \
  -p qos_overrides./burger2/camera/image_raw/compressed.publisher.reliability:=best_effort \
  -p qos_overrides./burger2/camera/image_raw/compressed.publisher.depth:=1 "$@"
