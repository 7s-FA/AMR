#!/usr/bin/env python3
"""Raspberry Pi GPIO linear motor console test (seconds, no ROS required)."""

import argparse
import math
import select
import sys
import termios
import time
import tty


class Motor:
    def __init__(self, chip, dry_run=False):
        self.factory = None
        self.a = self.b = None
        if not dry_run:
            from gpiozero import DigitalOutputDevice
            from gpiozero.pins.lgpio import LGPIOFactory

            self.factory = LGPIOFactory(chip=chip)
            try:
                self.a = DigitalOutputDevice(20, initial_value=False,
                                             pin_factory=self.factory)
                self.b = DigitalOutputDevice(21, initial_value=False,
                                             pin_factory=self.factory)
            except BaseException:
                self.close()
                raise

    def stop(self):
        if self.a is not None:
            self.a.off()
        if self.b is not None:
            self.b.off()

    def drive(self, direction):
        self.stop()
        time.sleep(0.05)  # Both outputs off before reversing direction.
        if direction == 'up' and self.b is not None:
            self.b.on()
        elif direction == 'down' and self.a is not None:
            self.a.on()

    def close(self):
        self.stop()
        for device in (self.a, self.b, self.factory):
            if device is not None:
                device.close()


class Controller:
    def __init__(self, motor, home_seconds=12.0):
        self.motor = motor
        self.home_seconds = home_seconds
        self.state = 'idle'
        self.deadline = 0.0
        self.exit_after_home = False

    def home(self, exiting=False):
        self.exit_after_home |= exiting
        if self.state == 'down':
            return  # Repeated reset/q must not extend the homing timer.
        self.motor.drive('down')
        self.deadline = time.monotonic() + self.home_seconds
        self.state = 'down'
        print(f'\n하강 {self.home_seconds:g}초: 바닥 복귀 중...', flush=True)

    def command(self, command):
        command = command.strip().lower()
        if command in ('q', 'r', 'reset', '초기화', '0'):
            self.home(exiting=command == 'q')
            return
        try:
            seconds = float(command)
        except ValueError:
            print('\n초 단위 숫자, r(초기화), q(복귀 후 종료)를 입력하세요.')
            return
        if not math.isfinite(seconds) or not 0 < seconds <= 20:
            print('\n상승 시간은 0보다 크고 20 이하인 유한한 숫자여야 합니다.')
            return
        if self.state != 'idle':
            print('\n이동 중입니다. 완료 후 숫자를 입력하세요. q/r은 즉시 처리됩니다.')
            return
        self.motor.drive('up')
        self.deadline = time.monotonic() + seconds
        self.state = 'up'
        print(f'\n현재 위치에서 {seconds:g}초 상승합니다.', flush=True)

    def tick(self):
        if self.state != 'idle' and time.monotonic() >= self.deadline:
            previous = self.state
            self.motor.stop()
            self.state = 'idle'
            print('\n' + ('상승 완료. 현재 위치 유지.' if previous == 'up'
                           else '하강 시간 완료. 바닥 도착은 육안으로 확인하세요.'), flush=True)
        return self.exit_after_home and self.state == 'idle'


def run(controller):
    fd = sys.stdin.fileno()
    terminal = sys.stdin.isatty()
    saved = termios.tcgetattr(fd) if terminal else None
    buffer = ''
    eof = False
    try:
        if terminal:
            tty.setcbreak(fd)  # q/r are read immediately, without Enter.
        controller.home()
        print('숫자+Enter: 해당 초만큼 추가 상승 | r: 초기화 | q: 복귀 후 종료', flush=True)
        while True:
            try:
                if controller.tick():
                    return
                if eof:
                    time.sleep(0.01)
                    continue
                ready, _, _ = select.select([sys.stdin], [], [], 0.01)
                if not ready:
                    continue
                # os.read avoids TextIO buffering hiding pending keystrokes.
                import os
                char = os.read(fd, 1).decode('ascii', errors='ignore')
                if not char:
                    eof = True
                    controller.home(exiting=True)
                elif char.lower() in ('q', 'r'):
                    buffer = ''
                    controller.command(char)
                elif char in ('\n', '\r'):
                    if buffer.strip():
                        controller.command(buffer)
                    buffer = ''
                elif char in ('\x7f', '\b'):
                    buffer = buffer[:-1]
                    if terminal:
                        print('\b \b', end='', flush=True)
                elif char.isprintable():
                    buffer += char
                    if terminal:
                        print(char, end='', flush=True)
            except KeyboardInterrupt:
                controller.home(exiting=True)
    finally:
        controller.motor.stop()
        if saved is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chip', type=int, default=4)
    parser.add_argument('--dry-run', action='store_true', help='GPIO 없이 동작 흐름 확인')
    args = parser.parse_args()
    motor = Motor(args.chip, args.dry_run)
    try:
        run(Controller(motor))
    finally:
        motor.close()


if __name__ == '__main__':
    main()
