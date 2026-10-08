#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/nav_env.bash"
CAMERA_ROOT=${AMR_CAMERA}
python3 "$CAMERA_ROOT/docking_network.py" --config "$CAMERA_ROOT/docking.yaml" --role robot --output "$HERE/dds_rest.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_rest.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec /usr/bin/python3 -u "$HERE/rest_ready_worker.py"
