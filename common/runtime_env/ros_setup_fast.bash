# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Fast ROS setup; hooks/rebuild changes regenerate the private cache.
_burger_setup_dir=$(cd "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_burger_setup_root=$(cd "$_burger_setup_dir/.." && pwd)
_burger_setup_cache=$(/usr/bin/python3 "$_burger_setup_dir/ros_setup_cache.py" "$_burger_setup_root") || return 1
if [[ "${BURGER_ROS_SETUP_CACHE:-}" != "$_burger_setup_cache" ]];then
 source "$_burger_setup_cache" || return 1
 export BURGER_ROS_SETUP_CACHE="$_burger_setup_cache"
fi
unset _burger_setup_dir _burger_setup_root _burger_setup_cache
