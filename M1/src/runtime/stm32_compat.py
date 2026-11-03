#!/usr/bin/env python3
# ========================================================================
# 역할: 버거3(STM32F411) 호환 노드. OpenCR(turtlebot3_ros)이 주던 두 가지를 같은 이름으로 대신 제공한다.
#   - /<ns>/sensor_state (turtlebot3_msgs/SensorState): torque, left_encoder, right_encoder
#       엔코더 = 바퀴 각도(rad) / 0.001533981 (XL430 1틱, AMR 코드의 ENCODER_METRES_PER_TICK 와 같은 단위)
#       토크  = /stm32/left_wheel_state, right_wheel_state 가 둘 다 1 이고 0.5초 안에 받은 값일 때만 True
#   (2026-10-15) sensor_state 는 20Hz 타이머로 최신 값을 발행. 토크 on 인데 바퀴 각도가 0.25초 넘게 안 오면 발행 안 함
#   - /<ns>/motor_power (std_srvs/SetBool): /stm32/xl430_cmd 로 "motor on"/"motor off" 전송 후 실제 상태 확인
# 실행: base.launch.py (start_base.sh) 가 띄운다. 단독: python3 stm32_compat.py --ros-args -r __ns:=/burger1
# 배터리·버튼·소리 값은 만들지 않는다(battery=0, 배터리 경고는 STM32 가 처리).
# ========================================================================
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from turtlebot3_msgs.msg import SensorState

RAD_PER_TICK = 0.001533981     # 2*pi/4096
STATE_FRESH_S = 2.0            # 마지막 토크 상태를 유지하는 시간 (2026-10-15: 0.5→2.0, 주행 중 상태 메시지가 늦게 와 '토크 꺼짐'으로 출차가 멈추던 문제. 실제로 끄면 STM32 가 0 을 바로 보냄)
PUBLISH_HZ = 20.0
READ_ERROR_HOLD_S = 1.0        # (2026-10-16) STM32 가 모터 상태를 못 읽으면 음수(-4 등)를 보냄. 마지막 정상 값을 이 시간까지 유지
                               # (10-15·16 출차 중 -4 가 0.13초 한 번 와서 '토크 꺼짐'으로 멈추던 문제). 계속 음수면 그대로 알림
JOINT_STALE_S = 0.25           # 토크 켜짐인데 바퀴 각도가 이보다 오래되면 발행 안 함              # sensor_state 발행 주기
JOINT_DROP_AGE_S = 1.0         # 이보다 오래된 joint_states 만 버림 (2026-10-15: 0.3→1.0, 정상 메시지까지 버려 간격이 벌어지던 문제)


def to_int32(value):
    return (int(value) + 2**31) % 2**32 - 2**31


class Stm32Compat(Node):
    def __init__(self):
        super().__init__('stm32_compat')
        transport = self.declare_parameter('transport_namespace', '/stm32').value.rstrip('/')
        self.lock = threading.Lock()
        self.wheel = {'left': (None, 0.0), 'right': (None, 0.0)}   # (상태값, 받은 시각)
        self.joint_time, self.encoders = 0.0, (0, 0)
        self.last_good, self.read_errors = {}, 0   # 바퀴별 마지막 정상 상태값(0/1)·읽기 실패 횟수   # 마지막 바퀴 각도 시각·엔코더
        group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(SensorState, 'sensor_state', 10)
        self.cmd = self.create_publisher(String, transport + '/xl430_cmd', 10)
        self.create_subscription(Int32, transport + '/left_wheel_state', lambda m: self.on_wheel('left', m), 10)
        self.create_subscription(Int32, transport + '/right_wheel_state', lambda m: self.on_wheel('right', m), 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint, 10)  # 어댑터가 reliable 로 보냄 → 빠짐없이 받기
        self.create_service(SetBool, 'motor_power', self.on_motor_power, callback_group=group)
        self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)   # OpenCR 처럼 일정 주기로 최신 값 발행 (2026-10-15)
        self.get_logger().info('stm32_compat ready: sensor_state, motor_power (OpenCR 대체)')

    def on_wheel(self, side, msg):
        now = time.monotonic()
        with self.lock:
            if msg.data < 0:   # 읽기 실패: 직전 정상 값이 READ_ERROR_HOLD_S 안이면 그 값을 유지
                good = self.last_good.get(side)
                self.read_errors += 1
                if good is not None and now - good[1] <= READ_ERROR_HOLD_S:
                    self.wheel[side] = (good[0], now)
                    return
            else:
                self.last_good[side] = (msg.data, now)
            self.wheel[side] = (msg.data, now)

    def torque(self):
        now = time.monotonic()
        with self.lock:
            return all(v == 1 and now - t < STATE_FRESH_S for v, t in self.wheel.values())

    def on_joint(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if stamp > 0 and now_ros - stamp > JOINT_DROP_AGE_S:
            return
        pos = {}
        for name, value in zip(msg.name, msg.position):
            if 'left' in name and math.isfinite(value):
                pos['left'] = value
            elif 'right' in name and math.isfinite(value):
                pos['right'] = value
        if len(pos) != 2:
            return
        with self.lock:
            self.joint_time = time.monotonic()
            self.encoders = (to_int32(round(pos['left'] / RAD_PER_TICK)), to_int32(round(pos['right'] / RAD_PER_TICK)))

    def on_timer(self):
        # 토크 켜짐: 바퀴 각도가 0.25초 안에 왔을 때만 발행 (멈춘 엔코더로 출차 거리를 잘못 재지 않게, 끊기면 소비자가 실패 처리)
        # 토크 꺼짐: STM32 가 바퀴 각도를 안 보내므로 토크 상태만 발행 (host_prepare 가 확인 후 motor_power 호출)
        torque = self.torque()
        with self.lock:
            joint_age = time.monotonic() - self.joint_time
            encoders = self.encoders
            wheel_fresh = all(time.monotonic() - t < STATE_FRESH_S for _, t in self.wheel.values())
        if torque and joint_age > JOINT_STALE_S:
            return
        if not torque and not wheel_fresh:
            return
        out = SensorState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.torque = torque
        out.left_encoder, out.right_encoder = encoders
        self.pub.publish(out)

    def on_motor_power(self, request, response):
        want = bool(request.data)
        self.cmd.publish(String(data='motor on' if want else 'motor off'))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            time.sleep(0.05)
            if self.torque() == want:
                response.success, response.message = True, 'torque ' + ('on' if want else 'off')
                return response
        with self.lock:
            states = {k: v for k, (v, _) in self.wheel.items()}
        response.success = False
        response.message = f'torque not confirmed within 2s (wheel states {states}; 1=on 0=off <0=UART error)'
        return response


def main():
    rclpy.init()
    node = Stm32Compat()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
