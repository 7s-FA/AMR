#!/usr/bin/env python3
"""Robot-local straight drive; direct GPIO HIGH stops. No camera/Nav2/odom."""
import argparse
import math
import signal
import sys
import time
from pathlib import Path


def drive_command(high, elapsed, speed, limit):
    if high:
        return 0.0, 'IR 감지: 휴식장소 도착'
    if elapsed >= limit:
        return 0.0, 'IR 미감지: 최대 이동 시간 초과'
    return speed, None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--check', action='store_true', help='설정 확인만; GPIO/모터 접근 없음')
    a = p.parse_args()
    import yaml
    cfg = yaml.safe_load(Path(a.config).read_text())
    speed = float(cfg['control']['final_speed_mps'])
    limit = float(cfg['control'].get('max_total_time_s', 90))
    if not math.isfinite(speed) or not 0 < speed <= .08 or not math.isfinite(limit) or limit <= 0:
        raise ValueError('Invalid rest speed/time limit')
    if a.check:
        print(f"{cfg['cmd_topic']}: 직진 {speed:.4f} m/s, IR GPIO{cfg['gpio_pin']} HIGH 정지, 제한 {limit:g}s")
        return 0
    sys.path.insert(0, str(Path(a.config).parent))
    from ir_sensor import GPIOInput
    import rclpy
    from geometry_msgs.msg import TwistStamped
    gpio = node = None
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
        pub = node.create_publisher(TwistStamped, cfg['cmd_topic'], 1)
        def publish(v):
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = cfg['base_frame']
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = 0.0
            pub.publish(msg)
        def graph_ready():
            publishers = node.get_publishers_info_by_topic(cfg['cmd_topic'])
            if any(x.node_name != node.get_name() or x.node_namespace != node.get_namespace() for x in publishers):
                raise RuntimeError('다른 속도 명령 발행자가 있습니다. 먼저 해당 주행을 종료하세요.')
            subscribers = node.get_subscriptions_info_by_topic(cfg['cmd_topic'])
            return bool(subscribers) and all(x.topic_type == 'geometry_msgs/msg/TwistStamped' for x in subscribers)
        wait_deadline = time.monotonic() + 8
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if gpio.high():
                publish(0)
                node.get_logger().info('IR 이미 감지됨: 전진하지 않습니다.')
                result = 0
                return result
            if graph_ready():
                break
            if time.monotonic() >= wait_deadline:
                raise RuntimeError('모터 명령 구독자를 확인하지 못했습니다.')
        if stopping:
            return 130
        node.get_logger().info(f'휴식장소 직진 시작: {speed:.4f} m/s; IR HIGH 즉시 정지')
        started = time.monotonic()
        next_graph = started
        next_publish = started
        while not stopping and rclpy.ok():
            now = time.monotonic()
            v, reason = drive_command(gpio.high(), now-started, speed, limit)
            if reason:
                publish(0)
                node.get_logger().info(reason)
                result = 0 if v == 0 and reason.startswith('IR 감지') else 1
                break
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
        print(f'휴식장소 이동 중단: {exc}', file=sys.stderr)
        sys.exit(1)
