#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/nav_env.bash"
CAMERA_ROOT=/home/ubuntu/final_robot_camera
python3 "$CAMERA_ROOT/docking_network.py" --config "$CAMERA_ROOT/docking.yaml" --role robot --output "$HERE/dds_rest.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_rest.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /usr/bin/python3 -u "$HERE/rest_ready_worker.py"
