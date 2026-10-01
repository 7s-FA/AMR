# Robot-local ROS environment; assembled by materialize.py.
source "$(dirname -- "${BASH_SOURCE[0]}")/pc_env.bash"
source "$(dirname -- "${BASH_SOURCE[0]}")/burger2_remote.bash"
export BURGER2_NAV_PARAMS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/nav2_burger2_params.yaml"
export BURGER2_ROS_DOMAIN_ID="${BURGER2_ROS_DOMAIN_ID:-40}"
export ROS_DOMAIN_ID="$BURGER2_ROS_DOMAIN_ID"
export BURGER2_WAYPOINTS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/waypoints_burger2.yaml"
