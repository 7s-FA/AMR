#!/usr/bin/env python3
"""Warm Nav2 with exclusive motor ownership; localization stays continuous."""
import argparse
import json
from pathlib import Path
import subprocess
import time



def wait_for_nav2(read_state, resume, paused=False, timeout=45, clock=time.monotonic, sleep=time.sleep):
    """Require a real ACTIVE reply; preserve the reason for missing responses."""
    deadline = clock() + timeout
    resumed = False
    state = None
    last_error = None
    while True:
        try:
            state = read_state()
            last_error = None
        except RuntimeError as exc:
            state = None
            last_error = str(exc)
        if state == 3:
            return
        if state == 2 and paused and not resumed:
            resume()
            resumed = True
        if clock() >= deadline:
            detail = last_error or ('lifecycle state=' + str(state))
            raise RuntimeError('Nav2 준비 상태 시간 초과: ' + detail)
        sleep(.1)


def prepare_with_recovery(prepare, restart, paused=False):
    """Retry preparation once, while the caller retains idle motor ownership."""
    try:
        prepare(paused)
    except RuntimeError as first:
        restart(str(first))
        try:
            prepare(False)
        except RuntimeError as second:
            raise RuntimeError('Nav2 1회 복구 후 준비 실패: ' + str(second)) from second


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser()
    parser.add_argument('robot')
    parser.add_argument('mode', choices=['prepare', 'nav', 'direct', 'idle'], nargs='?', default='idle')
    parser.add_argument('--serve', action='store_true')
    args = parser.parse_args()
    import rclpy
    from std_srvs.srv import Trigger
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    from action_msgs.srv import CancelGoal
    from action_msgs.msg import GoalStatusArray, GoalStatus
    from rclpy.qos import QoSProfile, DurabilityPolicy

    cache = Path(__file__).resolve().parents[3]/'data'/args.robot/'navigation_mode.json'
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    def nav_pid():
        return subprocess.check_output(['systemctl', '--user', 'show', args.robot+'-nav2.service',
                                        '-p', 'MainPID', '--value'], text=True).strip()
    def record(ready):
        cache.parent.mkdir(parents=True, exist_ok=True)
        temp = cache.with_suffix('.tmp')
        temp.write_text(json.dumps({'boot': boot, 'pid': nav_pid(), 'paused': not ready,
                                   'ready': ready, 'updated_monotonic': time.monotonic()}))
        temp.replace(cache)

    rclpy.init(args=['--ros-args', '-r', '/tf:=/'+args.robot+'/tf',
                    '-r', '/tf_static:=/'+args.robot+'/tf_static'])
    node = rclpy.create_node('motion_mode_client', namespace='/'+args.robot)
    service_clients = {}
    def call(cls, path, request, timeout=10):
        first = path not in service_clients
        if first: service_clients[path] = node.create_client(cls, path)
        client = service_clients[path]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(path+' 서비스 준비 시간 초과')
        if first:
            rclpy.spin_once(node, timeout_sec=.15)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future); future.cancel()
            node.destroy_client(client); service_clients.pop(path, None)
            raise RuntimeError(path+' 응답 시간 초과')
        if future.result() is None: raise RuntimeError(path+' 응답 없음')
        return future.result()

    def cancel_navigation_actions():
        """Cancel all old Nav2 goals before granting either command source.

        Motor ownership is already idle. Requests run together instead of
        waiting for one server after another. Accepted cancellation must also
        reach a terminal action status before navigation can be granted again.
        """
        actions = ('navigate_to_pose', 'navigate_through_poses', 'spin',
                   'drive_on_heading', 'backup', 'follow_waypoints', 'assisted_teleop')
        clients = {name: node.create_client(CancelGoal, name+'/_action/cancel_goal')
                   for name in actions}
        statuses = {}; subscriptions = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def update(name):
            def callback(msg):
                statuses[name] = {bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}
            return callback
        try:
            for name in actions:
                subscriptions.append(node.create_subscription(
                    GoalStatusArray, name+'/_action/status', update(name), qos))
            discovery_end = time.monotonic()+2.
            core = ('navigate_to_pose', 'navigate_through_poses', 'spin', 'drive_on_heading', 'backup')
            while not all(clients[name].service_is_ready() for name in core):
                if time.monotonic() >= discovery_end:
                    raise RuntimeError('Nav2 목표 취소 서비스 연결 없음')
                rclpy.spin_once(node, timeout_sec=.05)
            pending = {name: client.call_async(CancelGoal.Request())
                       for name, client in clients.items() if client.service_is_ready()}
            deadline = time.monotonic()+3.
            while not all(f.done() for f in pending.values()):
                if time.monotonic() >= deadline:
                    raise RuntimeError('이전 Nav2 목표 취소 응답 시간 초과')
                rclpy.spin_once(node, timeout_sec=.03)
            cancelled = {}
            for name, future in pending.items():
                result = future.result()
                if result is None or result.return_code not in (
                        CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED):
                    raise RuntimeError(name+' 이전 목표 취소 실패')
                cancelled[name] = [bytes(g.goal_id.uuid) for g in result.goals_canceling]
            terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            while any(statuses.get(name, {}).get(goal) not in terminal
                      for name, goals in cancelled.items() for goal in goals):
                if time.monotonic() >= deadline:
                    raise RuntimeError('이전 Nav2 목표 취소 완료 시간 초과')
                rclpy.spin_once(node, timeout_sec=.03)
        finally:
            for sub in subscriptions: node.destroy_subscription(sub)
            for client in clients.values(): node.destroy_client(client)

    buf = None; listener = None
    def perform(mode):
        nonlocal started, buf, listener
        started = time.monotonic(); args.mode = mode
        result = call(Trigger, 'motion_owner/idle', Trigger.Request())
        if not result.success: raise RuntimeError(result.message)
        ready = False
        if args.mode in ('nav', 'prepare'):
            from tf2_ros import Buffer, TransformListener
            if buf is None:
                buf = Buffer(); listener = TransformListener(buf, node, spin_thread=False)
            deadline = time.monotonic()+12
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                try:
                    tf = buf.lookup_transform(args.robot+'/map', args.robot+'/base_footprint', rclpy.time.Time())
                    age = (node.get_clock().now().nanoseconds
                           - tf.header.stamp.sec*10**9-tf.header.stamp.nanosec)/1e9
                    if -.8 <= age <= .8: break
                except Exception: pass
            else: raise RuntimeError('최신 지도 위치 없음: 라이다/위치 추정 연결을 확인하세요.')
            old_pid = nav_pid()
            try: known = json.loads(cache.read_text())
            except (OSError, ValueError): known = {}
            was_paused = (old_pid not in ('', '0') and known.get('pid') == old_pid
                          and known.get('boot', boot) == boot and known.get('paused') is True)
            def prepare_navigation(paused):
                subprocess.run(['systemctl', '--user', 'start', args.robot+'-nav2.service'], check=True)
                def resume():
                    request = ManageLifecycleNodes.Request(); request.command = request.RESUME
                    if not call(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes', request, 20).success:
                        raise RuntimeError('Nav2 주행 기능 재개 실패')
                wait_for_nav2(lambda: call(GetState, 'bt_navigator/get_state', GetState.Request(), 2).current_state.id,
                              resume, paused=paused)
                cancel_navigation_actions()

            def restart_navigation(reason):
                # Never restart localization or reissue a mission. Keep motors idle.
                idle = call(Trigger, 'motion_owner/idle', Trigger.Request())
                if not idle.success: raise RuntimeError(idle.message)
                node.get_logger().warn(reason + ' / Nav2만 1회 재시작하여 준비 복구')
                record(False)
                for path in list(service_clients):
                    if not path.startswith('motion_owner/'):
                        node.destroy_client(service_clients.pop(path))
                subprocess.run(['systemctl', '--user', 'restart', args.robot+'-nav2.service'], check=True, timeout=30)

            prepare_with_recovery(prepare_navigation, restart_navigation, was_paused)
            ready = True
        elif subprocess.run(['systemctl', '--user', 'is-active', '--quiet', args.robot+'-nav2.service']).returncode == 0:
            try:
                state = call(GetState, 'bt_navigator/get_state', GetState.Request(), 2).current_state.id
                if state == 3:
                    cancel_navigation_actions()
                    ready = True
                elif state != 2:
                    raise RuntimeError('Nav2 대기 상태 불완전')
            except RuntimeError as exc:
                node.get_logger().warn(str(exc)+' / 주행 서비스만 종료, 위치 추정 유지')
                subprocess.run(['systemctl', '--user', 'stop', args.robot+'-nav2.service'], check=True)
        # Warm navigation has no old goals. Docking/rest alone own motor output.
        if args.mode in ('nav', 'direct'):
            result = call(Trigger, 'motion_owner/'+args.mode, Trigger.Request())
            if not result.success: raise RuntimeError(result.message)
        record(ready)
        print(f'{args.robot} 명령 제어권: {"idle (Nav2 준비 유지)" if args.mode == "prepare" else args.mode}'
              f' / 전환 {time.monotonic()-started:.2f}초', flush=True)
    try:
        if args.serve:
            import socket, os
            address = str(Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))/(args.robot+'-nav-control.sock'))
            if Path(address).exists(): Path(address).unlink()
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            server.bind(address); os.chmod(address, 0o600); server.listen(8); server.setblocking(False)
            print('Navigation control ready: '+address, flush=True)
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.01)
                    try: connection, _ = server.accept()
                    except BlockingIOError: continue
                    with connection:
                        connection.settimeout(2.)
                        try:
                            raw = b''
                            while b'\n' not in raw and len(raw) < 1024:
                                chunk = connection.recv(1024)
                                if not chunk: break
                                raw += chunk
                            command = json.loads(raw)
                            mode = command['mode']
                            if mode not in ('prepare', 'nav', 'direct', 'idle'): raise ValueError('Invalid mode')
                            perform(mode)
                            reply = {'success': True, 'mode': mode, 'seconds': round(time.monotonic()-started, 3)}
                        except Exception as exc:
                            # A failed handoff never grants a new motor source.
                            try: call(Trigger, 'motion_owner/idle', Trigger.Request(), 2)
                            except Exception: pass
                            reply = {'success': False, 'message': str(exc)}
                            node.get_logger().error(str(exc))
                        try: connection.sendall((json.dumps(reply)+'\n').encode())
                        except OSError: pass
            finally:
                server.close(); Path(address).unlink(missing_ok=True)
        else:
            perform(args.mode)
    finally:
        # TF subscriptions must be detached before destroying the ROS context.
        if listener is not None:
            listener.unregister(); del listener
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
