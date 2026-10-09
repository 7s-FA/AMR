#!/usr/bin/env bash
# ========================================================================
# 역할: 카메라 프로그램(native_camera) 실행 → 공유메모리로 프레임 제공. burger2-camera.service 의 본체 (ROS 사용 안 함).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# No ROS environment, Image topics or DDS participant is started for video.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT=$(python3 - "$HERE/docking.yaml" <<'PY'
import sys,yaml
c=yaml.safe_load(open(sys.argv[1]));r=c['cmd_topic'].strip('/').split('/')[0]
assert r in ('burger2','burger2');print(r)
PY
)
RUNTIME=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
export LD_LIBRARY_PATH="$HERE/install/libcamera/lib:${LD_LIBRARY_PATH:-}"
export LIBPISP_BE_CONFIG_FILE="$HERE/install/libcamera/share/libpisp/backend_default_config.json"
export LIBCAMERA_IPA_MODULE_PATH="$HERE/install/libcamera/lib/libcamera/ipa"
export LIBCAMERA_IPA_PROXY_PATH="$HERE/install/libcamera/libexec/libcamera"
export LIBCAMERA_IPA_CONFIG_PATH="$HERE/install/libcamera/share/libcamera/ipa"
exec "$HERE/native_camera" "$RUNTIME/$ROBOT-camera.frames" "$RUNTIME/$ROBOT-camera-control.sock"
