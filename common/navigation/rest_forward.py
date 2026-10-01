#!/usr/bin/env python3
"""Robot-local straight drive; IR arrival plus fresh odometry confirms a complete stop."""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path


def settings(cfg):
    values = {key: float(cfg['rest_control'][key]) for key in
              ('speed_mps', 'max_time_s', 'odom_timeout_s', 'stopped_hold_s', 'stop_timeout_s')}
    if (not all(math.isfinite(v) and v > 0 for v in values.values())
            or values['speed_mps'] > .08 or values['odom_timeout_s'] > .3
            or values['stopped_hold_s'] < .3
            or values['stop_timeout_s'] <= values['stopped_hold_s']):
        raise ValueError('Invalid rest_control settings')
    return values


class StopMonitor:
    def __init__(self, timeout, hold):
        self.timeout, self.hold = timeout, hold
        self.received = self.since = self.stamp = None
        self.age_at_receive = 0.

    def update(self, stamp, age, linear, angular, now):
        if (not all(math.isfinite(x) for x in (stamp, age, linear, angular))
                or stamp <= 0 or not -.1 <= age <= self.timeout
                or (self.stamp is not None and stamp <= self.stamp)):
            self.since = None
            return
        if not self.fresh(now):
            self.since = None
        self.stamp, self.received = stamp, now
        self.age_at_receive = max(0., age)
        if abs(linear) <= .01 and abs(angular) <= .04:
            if self.since is None:
                self.since = now
        else:
            self.since = None

    def fresh(self, now):
        return self.received is not None and 0 <= now-self.received and now-self.received+self.age_at_receive <= self.timeout

    def stopped(self, now):
        return self.fresh(now) and self.since is not None and now-self.since >= self.hold


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--check', action='store_true', help='설정 확인만; GPIO/모터 접근 없음')
    a = p.parse_args()
    import yaml
    cfg = yaml.safe_load(Path(a.config).read_text())
    params = settings(cfg)
    speed, limit = params['speed_mps'], params['max_time_s']
    if a.check:
        print(f"{cfg['cmd_topic']}: 직진 {speed:.4f} m/s, IR GPIO{cfg['gpio_pin']} HIGH 정지, 제한 {limit:g}s")
        return 0
    sys.path.insert(0, str(Path(a.config).parent))
    from ir_sensor import GPIOInput
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from communication_guard import GraphGuard
    gpio = node = guard = None
    stopping = False
    result = 1
    def stop_signal(*_):
        nonlocal stopping
        stopping = True
    rclpy.init(args=[])
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        gpio = GPIOInput(cfg['gpio_chip'], cfg['gpio_pin'])
        robot = cfg['cmd_topic'].strip('/').split('/')[0]
        node = rclpy.create_node('rest_forward', namespace='/' + robot)
        monitor = StopMonitor(params['odom_timeout_s'], params['stopped_hold_s'])
        def on_odom(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            v = msg.twist.twist
            monitor.update(stamp, age, math.hypot(v.linear.x, v.linear.y), v.angular.z, time.monotonic())
        node.create_subscription(Odometry, cfg['odom_topic'], on_odom, qos_profile_sensor_data)
        pub = node.create_publisher(TwistStamped, cfg['cmd_topic'], 1)
        def publish(v):
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = cfg['base_frame']
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = 0.0
            pub.publish(msg)
        guard = GraphGuard(cfg['cmd_topic'], node.get_name(), node.get_namespace(), 'rest')
        def graph_ready():
            error = guard.snapshot()['error']
            if error == 'cmd_vel_has_other_publisher_or_graph_not_ready':
                raise RuntimeError('다른 속도 명령 발행자가 있습니다. 먼저 해당 주행을 종료하세요.')
            return error is None
        wait_deadline = time.monotonic() + 8
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            publish(0)
            if graph_ready() and monitor.fresh(time.monotonic()):
                break
            if time.monotonic() >= wait_deadline:
                raise RuntimeError('모터 명령 구독자 또는 최신 odom을 확인하지 못했습니다.')
        if stopping:
            return 130
        node.get_logger().info(f'휴식장소 직진 시작: {speed:.4f} m/s; IR HIGH 즉시 정지')
        started = time.monotonic()
        ir_since = None
        next_graph = started
        next_publish = started
        while not stopping and rclpy.ok():
            now = time.monotonic()
            high = gpio.high()
            if high and ir_since is None:
                ir_since = now
                monitor.since = None  # Require a stationary hold after IR detection.
                publish(0)
            if not monitor.fresh(now):
                raise RuntimeError('odom_timeout')
            if ir_since is not None:
                publish(0)
                if not high:
                    raise RuntimeError('ir_lost_before_stationary')
                if monitor.stopped(now):
                    print(json.dumps({'state': 'RESTED', 'reason': 'ir_high_and_stationary',
                                      'stopped': True, 'ir_high': True}), flush=True)
                    result = 0
                    break
                if now-ir_since >= params['stop_timeout_s']:
                    raise RuntimeError('stationary_confirmation_timeout')
                rclpy.spin_once(node, timeout_sec=.02)
                continue
            if now-started >= limit:
                raise RuntimeError('rest_travel_timeout')
            v = speed
            if now >= next_graph:
                if not graph_ready():
                    raise RuntimeError('모터 연결 끊김: 정지')
                next_graph = now + .2
            if now >= next_publish:
                publish(v)
                next_publish = now + .05
            rclpy.spin_once(node, timeout_sec=.01)
        if stopping:
            result = 130
        return result
    finally:
        if node is not None and rclpy.ok():
            # Repeat zeros on normal arrival, Ctrl-C, service stop, and GPIO errors.
            end = time.monotonic() + .5
            while time.monotonic() < end:
                publish(0)
                rclpy.spin_once(node, timeout_sec=.02)
        if guard is not None:
            guard.close()
        if gpio is not None:
            gpio.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(json.dumps({'state': 'FAULT', 'reason': str(exc), 'stopped': False}), flush=True)
        sys.exit(1)
