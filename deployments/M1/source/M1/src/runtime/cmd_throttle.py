#!/usr/bin/env python3
# ========================================================================
# 역할: STM32 로 가는 속도 명령 줄이기 (2026-10-15). motion_owner 는 주행 중 /burger1/cmd_vel 을 50Hz 로 보내는데,
#       브리지가 이를 모두 115200bps 시리얼로 넘겨 통신이 밀렸다(명령 0.6초 지연, 바퀴 데이터 수 초 끊김).
#       이 노드가 /burger1/cmd_vel(TwistStamped) 을 받아 최신 값만 최대 CMD_HZ(기본 15Hz)로 /burger1/cmd_vel_stm32 에 넘긴다.
#       - 정지(0) 명령, 부호가 바뀌는 명령은 즉시 전달 / 원래 시각(stamp) 그대로 전달 (브리지의 오래된 명령 거부 유지)
#       - STM32 300ms 감시: 15Hz(67ms)면 충분. 입력이 끊기면 아무것도 보내지 않아 STM32 가 스스로 멈춘다.
# 실행: base.launch.py (브리지 어댑터는 cmd_vel → cmd_vel_stm32 로 이름만 바꿔 띄움). AMR·브리지 코드는 그대로.
# ========================================================================
import math
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

CMD_HZ = float(os.environ.get('F411_CMD_HZ', '15'))
FRESH_S = 0.35      # motion_owner 의 명령 유효시간과 같음


def sign(x, eps=1e-6):
    return 0 if abs(x) < eps else (1 if x > 0 else -1)


class CmdThrottle(Node):
    def __init__(self):
        super().__init__('cmd_throttle')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_stm32', 1)
        self.create_subscription(TwistStamped, 'cmd_vel', self.on_cmd, 1)
        self.latest = None            # (msg, received_monotonic)
        self.sent = None              # (v, w, sent_monotonic)
        self.pending = False
        self.count_in = self.count_out = 0
        self.create_timer(1.0 / CMD_HZ, self.on_timer)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'cmd_throttle: cmd_vel → cmd_vel_stm32, 최대 {CMD_HZ:.0f}Hz (정지·방향 전환은 즉시)')

    def send(self, msg, now):
        self.pub.publish(msg)
        self.sent = (msg.twist.linear.x, msg.twist.angular.z, now)
        self.pending = False
        self.count_out += 1

    def on_cmd(self, msg):
        now = time.monotonic()
        self.count_in += 1
        v, w = msg.twist.linear.x, msg.twist.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            return
        self.latest, self.pending = (msg, now), True
        if self.sent is None:
            self.send(msg, now); return
        sv, sw, st = self.sent
        stop = sign(v) == 0 and sign(w) == 0 and (sign(sv) != 0 or sign(sw) != 0)
        flip = sign(v) * sign(sv) < 0 or sign(w) * sign(sw) < 0
        start = (sign(sv) == 0 and sign(sw) == 0) and (sign(v) != 0 or sign(w) != 0)
        if stop or flip or start or now - st >= 1.0 / CMD_HZ:
            self.send(msg, now)

    def on_timer(self):
        if not self.pending or self.latest is None:
            return
        msg, at = self.latest
        now = time.monotonic()
        if now - at > FRESH_S:
            self.pending = False; return
        if now - self.sent[2] >= 1.0 / CMD_HZ * 0.9:
            self.send(msg, now)

    def report(self):
        if self.count_in:
            self.get_logger().info(f'10초 동안 입력 {self.count_in}개 → STM32 쪽 {self.count_out}개')
        self.count_in = self.count_out = 0


def main():
    rclpy.init()
    node = CmdThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
