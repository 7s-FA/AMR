# Fast ROS setup; hooks/rebuild changes regenerate the private cache.
_burger_setup_dir=$(cd "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_burger_setup_root=$(cd "$_burger_setup_dir/.." && pwd)
_burger_setup_cache=$(/usr/bin/python3 "$_burger_setup_dir/ros_setup_cache.py" "$_burger_setup_root") || return 1
if [[ "${BURGER_ROS_SETUP_CACHE:-}" != "$_burger_setup_cache" ]];then
 source "$_burger_setup_cache" || return 1
 export BURGER_ROS_SETUP_CACHE="$_burger_setup_cache"
fi
unset _burger_setup_dir _burger_setup_root _burger_setup_cache
