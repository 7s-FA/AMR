#!/usr/bin/env python3
# ========================================================================
# 역할: 재시도 전 확인 (읽기 전용). 명령 권한이 아직 유효한지, 최신 odom 으로 정지했는지, 모터 토크·지도 위치가 정상인지 본다.
#       위치를 새로 넣거나 모터를 움직이지 않는다.
# 실행: sequence_runner.py 의 recover() 가 단계 실패 후 재시도 직전에 실행.
# 사용처: sequence_runner.py 가 authorization() 을 import (재시도 권한 확인).
# ========================================================================
"""Bounded, read-only recovery check. No pose seeding, GPIO or velocity publisher."""
import os
import argparse
import json
import math
import time
from pathlib import Path
from mission_retry import RetryCancelled


# 재시도 권한: 같은 명령 ID 이고 정지 래치가 없어야 한다 (개별 시험은 래치만 확인).
def authorization(data, task_id, source, now=None):
    from action_gate import load_gate
    gate = load_gate(Path(data)/'action_gate.json')
    if gate is None:
        return source == 'manual'
    if gate.get('estop', True):
        return False
    if source == 'manual':
        return not gate.get('goal_id')
    return gate.get('goal_id') == task_id


# 제한 시간 안에 권한·정지·토크·모드 idle·지도 위치를 확인 (도킹 단계면 IR 도 확인).
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['burger2'], required=True)
    parser.add_argument('--source', choices=['manual', 'host'], required=True)
    parser.add_argument('--command-id', default='')
    parser.add_argument('--stage', default='navigation')
    parser.add_argument('--timeout', type=float, default=5.)
    args = parser.parse_args()
    data = Path(__file__).absolute().parents[3]/'data'/args.robot
    import rclpy
    from nav_msgs.msg import Odometry
    from turtlebot3_msgs.msg import SensorState
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener
    rclpy.init(args=['--ros-args', '-r', '/tf:=/burger2/tf', '-r', '/tf_static:=/burger2/tf_static'])
    node = rclpy.create_node('mission_retry_check', namespace='/burger2')
    buffer = Buffer(); listener = TransformListener(buffer, node, spin_thread=False)
    last = {}; stopped = [None]; stamp_ns = [-1]

    # 메시지 나이(초).
    def age(stamp):
        return (node.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9

    # odom 수신: 정지 유지 시각 갱신.
    def odom(msg):
        ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now = time.monotonic(); v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if ns <= stamp_ns[0] or not -.1 <= age(msg.header.stamp) <= .3:
            return
        stamp_ns[0] = ns
        if 'odom' not in last or now-last['odom'] > .3: stopped[0] = None
        last['odom'] = now
        if math.isfinite(v) and math.isfinite(w) and abs(v) <= .01 and abs(w) <= .04:
            if stopped[0] is None: stopped[0] = now
        else: stopped[0] = None

    # sensor_state 수신: 토크 기록.
    def motor(msg):
        if -.1 <= age(msg.header.stamp) <= .5:
            last['motor'] = time.monotonic(); last['torque'] = bool(msg.torque)

    # motion_owner/status 수신: 모드 기록.
    def owner(msg):
        last['owner'] = time.monotonic(); last['mode'] = msg.data

    node.create_subscription(Odometry, 'odom', odom, qos_profile_sensor_data)
    node.create_subscription(SensorState, 'sensor_state', motor, qos_profile_sensor_data)
    node.create_subscription(String, 'motion_owner/status', owner, 1)
    end = time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic() < end:
            if not authorization(data, args.command_id, args.source):
                raise RetryCancelled('재개 승인 없음')
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            gate = json.loads((data/'action_gate.json').read_text()) if (data/'action_gate.json').exists() else {}
            heartbeat = args.source != 'host' or 0 <= now-float(gate.get('heartbeat', -1e9)) <= 1.
            if not (heartbeat and now-last.get('odom', -1e9) <= .3
                    and stopped[0] is not None and now-stopped[0] >= .3
                    and now-last.get('motor', -1e9) <= .5 and last.get('torque')
                    and now-last.get('owner', -1e9) <= 1. and last.get('mode') == 'idle'):
                continue
            try:
                transform = buffer.lookup_transform('burger2/map', 'burger2/base_footprint', rclpy.time.Time())
                t, q = transform.transform.translation, transform.transform.rotation
                if not (-.1 <= age(transform.header.stamp) <= .5 and
                        all(math.isfinite(v) for v in (t.x,t.y,q.x,q.y,q.z,q.w)) and
                        abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) <= .01):
                    continue
            except Exception: continue
            if args.stage.startswith('terminal_'):
                # Released controllers leave GPIO free. Contact is never a reason
                # to restart forward motion, nor fabricated arrival evidence.
                import importlib.util
                import yaml
                camera = Path(os.environ['AMR_CAMERA'])
                cfg = yaml.safe_load((camera/'docking.yaml').read_text())
                spec = importlib.util.spec_from_file_location('retry_gpio',camera/'ir_sensor.py')
                gpio_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(gpio_module)
                gpio = gpio_module.GPIOInput(cfg['gpio_chip'],cfg['gpio_pin'])
                try:
                    if gpio.high(): raise RetryCancelled('IR 접촉 상태: 재접근하지 않습니다.')
                finally: gpio.close()
            print('재시도 준비 완료: 정지·최신 odom/TF·모터 토크·idle 제어권 확인', flush=True)
            return 0
        print('재시도 대기 시간 초과: 정지/위치/모터/제어권 확인 불가', flush=True)
        return 1
    except RetryCancelled as exc:
        print(str(exc), flush=True); return 130
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': raise SystemExit(main())
