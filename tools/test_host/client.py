#!/usr/bin/env python3
"""Small standalone Burger Action client; no SSH or direct velocity commands."""
import argparse
import json
import math
import signal
import time

COMMANDS = {'mat': 'GO_TO_MAT', 'asm': 'GO_TO_ASM', 'rest': 'GO_TO_REST',
            'park': 'GO_TO_PARK', 'stop': 'EMER_STOP', 'restart': 'RESTART'}


def request_values(robot, command, speed):
    if robot not in ('M1', 'M2'):
        raise ValueError('Robot must be M1 or M2')
    command = COMMANDS.get(command, command)
    if command not in COMMANDS.values():
        raise ValueError('Unknown command')
    if not math.isfinite(speed) or not 0 <= speed <= 100:
        raise ValueError('Speed must be between 0 and 100 percent')
    if command.startswith('GO_TO_') and speed == 0:
        raise ValueError('Movement speed must be greater than zero')
    return '/' + robot + '/data', command, speed if command.startswith('GO_TO_') else 0.


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot', choices=['M1', 'M2'])
    parser.add_argument('command', choices=list(COMMANDS) + list(COMMANDS.values()) + ['status'])
    parser.add_argument('--speed', type=float, default=100., help='Maximum-speed percentage (default: 100)')
    parser.add_argument('--timeout', type=float, default=650., help='Result wait limit in seconds')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be positive and finite')
    try:
        values = request_values(args.robot, args.command, args.speed) if args.command != 'status' else None
    except ValueError as exc:
        parser.error(str(exc))
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from host_pkg.action import Burger
    interrupted = False
    def interrupt(*_):
        nonlocal interrupted
        interrupted = True
    old = signal.signal(signal.SIGINT, interrupt)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('manual_test_host_' + args.robot.lower())
    client = ActionClient(node, Burger, '/' + args.robot + '/data')
    node.create_subscription(String, '/' + args.robot + '/mission/diagnostics',
                             lambda msg: print('상태: ' + msg.data, flush=True), 10)
    def wait(future, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        return future.result() if future.done() else None
    try:
        if args.command == 'status':
            print('새 상태 이벤트 수신 중 (저장된 상태 조회 아님). 종료: Ctrl+C', flush=True)
            while rclpy.ok() and not interrupted:
                rclpy.spin_once(node, timeout_sec=.1)
            return 0
        action, command, speed = values
        print(f'{action}: {command}, 속도 {speed:g}%', flush=True)
        if not client.wait_for_server(timeout_sec=5.):
            print('Action 서버 없음: 로봇 서버·ROS_DOMAIN_ID·네트워크를 확인하세요.')
            return 2
        if interrupted:
            return 130
        goal = Burger.Goal(); goal.command = command; goal.cmd_val = speed
        last_feedback = 0.
        def feedback(msg):
            nonlocal last_feedback
            if time.monotonic() - last_feedback < 1.:
                return
            last_feedback = time.monotonic(); f = msg.feedback
            print(f'위치 ({f.robot_x:.3f}, {f.robot_y:.3f}) {f.robot_theta} {f.message}', flush=True)
        handle = wait(client.send_goal_async(goal, feedback_callback=feedback), 10.)
        if handle is None:
            print('접수 응답 없음: 접수 여부 불명. 재전송 전에 stop으로 정지를 확인하세요.')
            return 2
        if not handle.accepted:
            print('명령 거절: process 모드·busy·정지 래치를 확인하세요.')
            return 1
        print('접수됨: ' + bytes(handle.goal_id.uuid).hex(), flush=True)
        future = handle.get_result_async(); deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not future.done() and not interrupted and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            print('취소 요청 중…', flush=True)
            wait(handle.cancel_goal_async(), 5.)
            result = wait(future, 10.)
            print('취소 후 결과: ' + (str(result) if result is not None else '응답 없음; 정지 확인 필요'))
            return 130 if interrupted else 2
        result = future.result()
        print(json.dumps({'status': result.status, 'success': result.result.success,
                          'message': result.result.message}, ensure_ascii=False), flush=True)
        return 0 if result.status == 4 and result.result.success else 1
    finally:
        client.destroy(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old)


if __name__ == '__main__':
    raise SystemExit(main())
