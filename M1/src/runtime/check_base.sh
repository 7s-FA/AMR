#!/usr/bin/env bash
# ========================================================================
# 역할: 버거3(STM32) 본체 점검 (읽기만). Agent·STM32 토픽·odom·라이다·sensor_state 주기, 토크 상태를 본다.
# 실행: bash check_base.sh
# ========================================================================
source /opt/ros/jazzy/setup.bash
source "$HOME/stm32_bridge_ws/install/setup.bash" 2>/dev/null
[[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID=40
NS=${AMR_ROBOT:-burger1}
echo "Agent: $(docker ps --filter name=f411-agent --format '{{.Status}}' 2>&1)"
echo "base 서비스: $(systemctl --user is-active burger1-base.service)"
for t in /stm32/joint_states /$NS/odom /$NS/scan /$NS/sensor_state /$NS/imu; do
  printf '%-22s ' "$t"; timeout 4 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo '수신 없음'
done
printf 'IMU 상태: '; timeout 3 ros2 topic echo /stm32/imu/status std_msgs/msg/String --qos-reliability best_effort --once 2>/dev/null | grep data || echo '없음'
for s in left right; do printf "%s 바퀴 상태(1=토크on 0=off <0=오류): " $s; timeout 3 ros2 topic echo /stm32/${s}_wheel_state std_msgs/msg/Int32 --once 2>/dev/null | grep data || echo '없음'; done
printf 'sensor_state torque: '; timeout 3 ros2 topic echo /$NS/sensor_state --once 2>/dev/null | grep -E 'torque|encoder' | tr '\n' ' '; echo
