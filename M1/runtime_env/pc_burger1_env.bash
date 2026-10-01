# Robot-local ROS environment; assembled by materialize.py.
source "$(dirname -- "${BASH_SOURCE[0]}")/pc_env.bash"
source "$(dirname -- "${BASH_SOURCE[0]}")/burger1_remote.bash"
export BURGER1_NAV_PARAMS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/nav2_burger1_params.yaml"
export BURGER1_ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}"
export ROS_DOMAIN_ID="$BURGER1_ROS_DOMAIN_ID"
export BURGER1_WAYPOINTS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/waypoints_burger1.yaml"
