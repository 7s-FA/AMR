#!/usr/bin/env python3
# ========================================================================
# 역할: 주행 모드 전환기. 'nav/prepare/direct/idle' 요청을 받아 Nav2 준비 확인·이전 목표 취소 후 motion_owner 모드를 바꾼다.
#       Nav2 가 일시정지 상태면 재개(RESUME), 응답이 없으면 Nav2 서비스만 1회 재시작해 복구한다 (위치추정은 유지).
# 실행: <로봇>-nav-control.service → nav_control_service.sh → python3 motion_mode.py burgerN --serve (유닉스 소켓 대기)
# 호출 관계: motion_client.py(set_mode.sh)가 소켓으로 요청 → 여기서 motion_owner/idle|nav|direct 서비스 호출.
# 부하: TF 구독은 nav/prepare 때만 만들고 30초 안 쓰면 해제, 대기 루프는 0.05초 주기 (2026-10-09 최적화).
# ========================================================================
"""Warm Nav2 with exclusive motor ownership; localization stays continuous."""
import argparse
import os
import json
from pathlib import Path
import subprocess
import time



# Nav2 가 아직 준비 중(응답 대기)이라는 뜻의 예외. 이 경우 Nav2 를 재시작하지 않는다.
class NavigationPending(RuntimeError):
    """Initialization/feedback is pending, not grounds to restart a running stack."""


# bt_navigator 상태가 ACTIVE(3)가 될 때까지 대기. 일시정지(2)면 한 번 재개를 요청한다.
def wait_for_nav2(read_state, resume, paused=False, timeout=120, clock=time.monotonic, sleep=time.sleep):
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
            raise NavigationPending('Nav2 준비 대기 시간 초과 (실행 중인 서비스 유지): ' + detail)
        sleep(.1)


# Nav2 준비를 시도하고, 실패하면 Nav2 만 재시작한 뒤 한 번 더 시도한다.
def prepare_with_recovery(prepare, restart, paused=False):
    """Retry preparation once, while the caller retains idle motor ownership."""
    try:
        prepare(paused)
    except NavigationPending:
        raise
    except RuntimeError as first:
        restart(str(first))
        try:
            prepare(False)
        except RuntimeError as second:
            raise RuntimeError('Nav2 1회 복구 후 준비 실패: ' + str(second)) from second


# 노드·서비스 클라이언트·액션 상태 구독을 만들고, --serve 면 소켓 요청을 계속 처리한다.
def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser()
    parser.add_argument('robot')
    parser.add_argument('mode', choices=['prepare', 'nav', 'direct', 'idle'], nargs='?', default='idle')
    parser.add_argument('--serve', action='store_true')
    args = parser.parse_args()
    import rclpy
    from startup_state import wait_response, wait_responses
    from std_srvs.srv import Trigger
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    from action_msgs.srv import CancelGoal
    from action_msgs.msg import GoalStatusArray, GoalStatus
    from rclpy.qos import QoSProfile, DurabilityPolicy

    cache = Path(__file__).absolute().parents[3]/'data'/args.robot/'navigation_mode.json'
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    # systemd 로 Nav2 서비스의 현재 PID 조회 (재시작 여부 판단용).
    def nav_pid():
        return subprocess.check_output(['systemctl', '--user', 'show', args.robot+'-nav2.service',
                                        '-p', 'MainPID', '--value'], text=True).strip()
    # data/<로봇>/navigation_mode.json 에 준비 상태(부팅 ID, Nav2 PID, 일시정지 여부) 저장.
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
    # 서비스 호출 공통 함수 (클라이언트 재사용, 제한 시간 안에 응답 대기).
    def call(cls, path, request, timeout=10):
        if path not in service_clients: service_clients[path] = node.create_client(cls, path)
        return wait_response(service_clients[path],request,
            lambda duration:rclpy.spin_once(node,timeout_sec=duration),timeout,path)

    cancel_clients = {}; statuses = {}; status_subscriptions = []
    actions = ('navigate_to_pose', 'navigate_through_poses', 'spin', 'precision_spin',
               'departure_spin', 'drive_on_heading', 'backup', 'follow_waypoints', 'assisted_teleop')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Nav2 액션별 목표 상태(_action/status)를 받아 저장하는 콜백 생성.
    def update_status(name):
        def callback(msg):
            statuses[name] = {bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}
        return callback
    for name in actions:
        cancel_clients[name]=node.create_client(CancelGoal,name+'/_action/cancel_goal')
        status_subscriptions.append(node.create_subscription(
            GoalStatusArray,name+'/_action/status',update_status(name),qos))

    # 남아 있는 Nav2 목표(주행·회전·후진 등)를 한꺼번에 취소하고 모두 끝날 때까지 확인.
    def cancel_navigation_actions():
        """Cancel all old Nav2 goals before granting either command source.

        Motor ownership is already idle. Requests run together instead of
        waiting for one server after another. Accepted cancellation must also
        reach a terminal action status before navigation can be granted again.
        """
        clients = cancel_clients
        try:
            discovery_end = time.monotonic()+2.
            core = ('navigate_to_pose', 'navigate_through_poses', 'spin', 'drive_on_heading', 'backup')
            while not all(clients[name].service_is_ready() for name in core):
                if time.monotonic() >= discovery_end:
                    raise RuntimeError('Nav2 목표 취소 서비스 연결 없음')
                rclpy.spin_once(node, timeout_sec=.05)
            pending = {name: client.call_async(CancelGoal.Request())
                       for name, client in clients.items() if client.service_is_ready()}
            acknowledgments = wait_responses(
                pending, clients, lambda dt: rclpy.spin_once(node, timeout_sec=dt),
                10., '이전 Nav2 목표 취소')
            cancelled = {}
            for name, result in acknowledgments.items():
                if result is None or result.return_code not in (
                        CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED):
                    raise RuntimeError(name+' 이전 목표 취소 실패')
                cancelled[name] = [bytes(g.goal_id.uuid) for g in result.goals_canceling]
            deadline = time.monotonic()+5.
            terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            while any(statuses.get(name, {}).get(goal) not in terminal
                      for name, goals in cancelled.items() for goal in goals):
                if time.monotonic() >= deadline:
                    raise RuntimeError('이전 Nav2 목표 취소 완료 시간 초과')
                rclpy.spin_once(node, timeout_sec=.03)
        finally:
            # Node teardown destroys these persistent clients/subscriptions.
            pass

    # TF(지도 위치) 구독은 메시지가 초당 50개 이상이라 파이썬에서 계속 받으면 CPU를 많이 쓴다.
    # 그래서 nav/prepare 전환에서 위치를 확인할 때만 만들고, 30초 동안 안 쓰면 serve 루프에서 해제한다.
    buf = listener = None
    tf_used_at = float('-inf')
    TF_IDLE_RELEASE_S = 30.
    # TF 구독을 해제한다 (대기 중 CPU 절약).
    def release_tf():
        nonlocal buf, listener
        if listener is not None:
            listener.unregister()
        buf = listener = None
    # 모드 전환 1회 수행: idle → (nav/prepare 면 지도 위치·Nav2 준비 확인·목표 취소) → 요청 모드 → 상태 저장.
    def perform(mode):
        nonlocal started, buf, listener, tf_used_at
        started = time.monotonic(); args.mode = mode
        result = call(Trigger, 'motion_owner/idle', Trigger.Request())
        if not result.success: raise RuntimeError(result.message)
        ready = False
        if args.mode in ('nav', 'prepare'):
            from tf2_ros import Buffer, TransformListener
            if buf is None:
                buf = Buffer(); listener = TransformListener(buf, node, spin_thread=False)
            tf_used_at = time.monotonic()
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
            # Nav2 서비스가 없으면 시작하고, ACTIVE 확인 후 이전 목표를 취소한다.
            def prepare_navigation(paused):
                from ready_parallel import start_missing
                start_missing(args.robot+'-nav2.service')
                # 일시정지된 Nav2 를 lifecycle_manager 로 재개.
                def resume():
                    request = ManageLifecycleNodes.Request(); request.command = request.RESUME
                    if not call(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes', request, 20).success:
                        raise RuntimeError('Nav2 주행 기능 재개 실패')
                wait_for_nav2(lambda: call(GetState, 'bt_navigator/get_state', GetState.Request(), 10).current_state.id,
                              resume, paused=paused)
                cancel_navigation_actions()

            # 복구용: 모터 idle 유지, Nav2 가 멈춰 있을 때만 다시 시작 (실행 중인 Nav2 는 재시작하지 않음).
            def restart_navigation(reason):
                # Never restart localization or reissue a mission. Keep motors idle.
                idle = call(Trigger, 'motion_owner/idle', Trigger.Request())
                if not idle.success: raise RuntimeError(idle.message)
                node.get_logger().warn(reason + ' / Nav2만 1회 재시작하여 준비 복구')
                record(False)
                for path in list(service_clients):
                    if not path.startswith('motion_owner/'):
                        node.destroy_client(service_clients.pop(path))
                state = subprocess.check_output(['systemctl', '--user', 'show', args.robot+'-nav2.service', '-p', 'ActiveState', '--value'], text=True).strip()
                if state not in ('inactive', 'failed'):
                    raise RuntimeError(reason + ' / 실행 중인 Nav2를 재시작하지 않습니다.')
                subprocess.run(['systemctl', '--user', 'start', args.robot+'-nav2.service'], check=True, timeout=30)

            prepare_with_recovery(prepare_navigation, restart_navigation, was_paused)
            ready = True
        elif subprocess.run(['systemctl', '--user', 'is-active', '--quiet', args.robot+'-nav2.service']).returncode == 0:
            try:
                state = call(GetState, 'bt_navigator/get_state', GetState.Request(), 10).current_state.id
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
            import socket
            address = str(Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))/(args.robot+'-nav-control.sock'))
            if Path(address).exists(): Path(address).unlink()
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            server.bind(address); os.chmod(address, 0o600); server.listen(8); server.setblocking(False)
            print('Navigation control ready: '+address, flush=True)
            try:
                while rclpy.ok():
                    # 0.05초마다 깨어나 접속을 확인한다 (예전 0.01초 → 대기 중 깨어남 1/5).
                    rclpy.spin_once(node, timeout_sec=.05)
                    if listener is not None and time.monotonic()-tf_used_at > TF_IDLE_RELEASE_S:
                        release_tf()
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
        release_tf()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
