#!/usr/bin/env python3
# ========================================================================
# 역할: burger1 IR 센서(GPIO17, Pi5 RP1 칩) 읽기. HIGH 면 도착(정지) 신호.
# 사용처: docking_node.py·docking_standby.py·rest_forward.py·retry_ready.py 가 GPIOInput 클래스를 import.
#       단독 실행(main) 하면 IR 상태를 토픽으로 발행하는 진단용 노드 (운용 중에는 쓰지 않음).
# ========================================================================
"""Publish physical GPIO HIGH as Bool true. This node does not command motors."""
import argparse
import math
import sys


# GPIO 입력 1개를 잡고 읽는 클래스 (libgpiod v1/v2 모두 지원).
class GPIOInput:
    # RP1 GPIO 칩인지 확인 후 핀을 입력으로 요청.
    def __init__(self, chip='/dev/gpiochip4', pin=17):
        import gpiod
        self.chip = self.line = self.request = None
        self.gpiod = gpiod
        try:
            self.chip = gpiod.Chip(chip)
            label = (self.chip.get_info().label if hasattr(self.chip, 'get_info')
                     else self.chip.label())
            if 'rp1' not in label.lower():
                raise ValueError(f'{chip} label={label!r} is not the expected RP1 GPIO chip')
            if hasattr(gpiod, 'request_lines'):
                self.chip.close()
                self.chip = None
                self.request = gpiod.request_lines(
                    chip, consumer='burger1_ir_high',
                    config={pin: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT)})
            else:
                self.line = self.chip.get_line(pin)
                if self.line.is_used():
                    raise RuntimeError(f'GPIO{pin} is already in use by {self.line.consumer()}')
                self.line.request(consumer='burger1_ir_high', type=gpiod.LINE_REQ_DIR_IN)
            self.pin = pin
        except Exception:
            self.close()
            raise

    # 현재 HIGH 인지.
    def high(self):
        if self.request is not None:
            return self.request.get_value(self.pin) == self.gpiod.line.Value.ACTIVE
        return bool(self.line.get_value())

    # GPIO 해제.
    def close(self):
        if self.request is not None:
            self.request.release()
            self.request = None
        if self.line is not None:
            if self.line.is_requested():
                self.line.release()
            self.line = None
        if self.chip is not None:
            self.chip.close()
            self.chip = None


# 진단용: IR 상태를 주기적으로 Bool 토픽 발행.
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chip', default='/dev/gpiochip4')
    parser.add_argument('--pin', type=int, default=17, help='BCM/RP1 line offset, not header pin number')
    parser.add_argument('--topic', default='/burger1/ir/high')
    parser.add_argument('--poll-hz', type=float, default=100.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.poll_hz) or not 1 <= args.poll_hz <= 1000 or args.pin < 0:
        parser.error('poll-hz must be 1..1000 and pin must be nonnegative')
    gpio = node = None
    rclpy = None
    try:
        gpio = GPIOInput(args.chip, args.pin)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import Bool
        rclpy.init(args=[])
        node = rclpy.create_node('ir_sensor', namespace='/burger1')
        publisher = node.create_publisher(Bool, args.topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE))
        previous = None

        # GPIO 읽고 발행.
        def sample():
            nonlocal previous
            value = gpio.high()  # Read failure exits; never synthesize a LOW.
            publisher.publish(Bool(data=value))
            if value != previous:
                node.get_logger().info(f'GPIO{args.pin}={"HIGH (STOP)" if value else "LOW"}')
                previous = value

        sample()
        node.create_timer(1. / args.poll_hz, sample)
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'IR input failed: {exc}. Consumer must stop on missing IR data.', file=sys.stderr)
        return 1
    finally:
        if gpio is not None:
            gpio.close()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
