#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Source this file inside the robot SSH shell. No motion runs on loading.
alias b1_ready='bash ${AMR_WORKSPACE}/robot/burger1/navigation/ready_watch.sh'
alias b1_parked='bash ${AMR_WORKSPACE}/robot/burger1/navigation/confirm_parked.sh '
alias b1_mat='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh mat'
alias b1_asm='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh asm'
alias b1_park='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh park'
alias b1_rest='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh rest'
alias b1_stop='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh stop'
alias b1_status='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh status'
alias b1_dock='bash ${AMR_WORKSPACE}/robot/burger1/navigation/manage.sh dock'
alias b1_park_only='bash ${AMR_WORKSPACE}/robot/burger1/navigation/manage.sh park'
alias b1_rest_ir='bash ${AMR_WORKSPACE}/robot/burger1/navigation/rest.sh start'
alias b1_way1='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh 1'
alias b1_way2='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh 2'
alias b1_way3='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh 3'
alias b1_way4='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh 4'
alias b1_way12='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh 12'
alias b1_way1_mat='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way1_mat'
alias b1_way2_mat='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way2_mat'
alias b1_way3_asm='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way3_asm'
alias b1_way3_exit='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way3_exit'
alias b1_way1_park='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way1_park'
alias b1_way4_park='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way4_park'
alias b1_way4_rest='bash ${AMR_WORKSPACE}/robot/burger1/navigation/run_selected_waypoints.sh way4_rest'
alias b1_off='bash ${AMR_WORKSPACE}/robot/burger1/navigation/mission.sh stop; bash ${AMR_WORKSPACE}/robot/burger1/navigation/manage.sh off'
alias b1_log='journalctl --user -u burger1-mission.service -f'
alias b1_dock_log='journalctl --user -u burger1-docking.service -f'
alias b1_help='cat ${AMR_WORKSPACE}/COMMANDS_KO.md'
alias b1_mode='python3 ${AMR_WORKSPACE}/robot/burger1/navigation/operation.py mode'
