#!/usr/bin/env python3
# ========================================================================
# 역할: burger2 웨이포인트 대기 작업자. 경로 코드(nav2_waypoints)와 ROS 연결을 미리 만들어 두고, 요청이 오면 같은 main() 을
#       같은 인자로 바로 실행한다 (주행 시작 시간 단축). 경로·움직임 상태는 재사용하지 않는다.
# 실행: burger2-waypoint-ready.service → start_waypoint_worker.sh → waypoint_worker.py (ready_parallel·ensure_waypoint_ready.sh 가 켬)
# 호출 관계: run_waypoints.sh → waypoint_client.py → 소켓($XDG_RUNTIME_DIR/burger2-waypoint-ready.sock) → 여기.
# 부하: 30초 동안 요청이 없으면 센서 구독을 끊고, 요청이 오면 다시 연결해 최신 값을 기다린다 (2026-10-09 최적화).
# 주의: 경로 코드 파일이 바뀌면(해시 다름) 재시작 전까지 요청을 거부한다.
# ========================================================================
"""Reuse only the Burger2 waypoint ROS connection, not routes or motion state."""
import argparse
import codecs
import contextlib
import hashlib
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import sys
import threading
import time


# 서비스 클라이언트 연결은 유지하되, 요청 1건이 끝나면 남은 응답 대기만 정리하는 래퍼.
class ServiceLease:
    """Keep DDS discovery, while discarding each call's unfinished read requests."""
    # 실제 클라이언트와 대기 목록.
    def __init__(self, client):
        self.client = client
        self.pending = []

    # 나머지 속성은 실제 클라이언트로 넘긴다.
    def __getattr__(self, name):
        return getattr(self.client, name)

    # 요청을 보내고 대기 목록에 추가.
    def call_async(self, request):
        future = self.client.call_async(request)
        self.pending.append(future)
        return future

    # 끝나지 않은 요청 정리.
    def release(self):
        for future in self.pending:
            if not future.done():
                self.client.remove_pending_request(future)
                future.cancel()
        self.pending.clear()


# 지정한 서비스 클라이언트만 재사용하는 내비게이터 클래스를 만든다 (응답은 캐시하지 않음).
def prepared_navigator_class(base, endpoints):
    """Only reuse specified service transports. Responses are NEVER cached."""
    # 미리 만든 서비스 연결을 재사용하는 WaypointNavigator.
    class PreparedNavigator(base):
        # 재사용할 서비스 클라이언트를 미리 만든다.
        def __init__(self, **kwargs):
            self.prepared_services = {}
            super().__init__(**kwargs)
            for service_type, name in endpoints:
                self.create_client(service_type, name)

        # 재사용 대상이면 저장된 클라이언트를 돌려준다.
        def create_client(self, service_type, name, *args, **kwargs):
            key = (service_type, name)
            if key not in endpoints:
                return super().create_client(service_type, name, *args, **kwargs)
            if key not in self.prepared_services:
                self.prepared_services[key] = ServiceLease(
                    super().create_client(service_type, name, *args, **kwargs))
            return self.prepared_services[key]

        # 재사용 대상은 없애지 않고 대기 요청만 정리.
        def destroy_client(self, client):
            if isinstance(client, ServiceLease):
                client.release()
                return True
            return super().destroy_client(client)
    return PreparedNavigator


# 요청 검사: run/probe/verify, 2초 이내, burger2 네임스페이스 인자 하나.
def validate_request(request, now):
    if request.get('op') not in ('run', 'probe', 'verify'):
        raise ValueError('Unknown waypoint operation')
    issued = request.get('issued')
    if isinstance(issued, bool) or not isinstance(issued, (int, float)) or not 0 <= now-issued <= 2:
        raise ValueError('Expired request; no queued motion')
    if request['op'] == 'run':
        argv = request.get('argv')
        if not isinstance(argv, list) or len(argv) > 100 or not all(isinstance(x, str) for x in argv):
            raise ValueError('Invalid waypoint arguments')
        if '--namespace' not in argv or argv[argv.index('--namespace')+1] != 'burger2':
            raise ValueError('Worker belongs to burger2 only')
        if argv.count('--namespace') != 1 or any(x.startswith('--namespace=') for x in argv):
            raise ValueError('Ambiguous namespace')


# 파일 SHA-256 해시.
def signature(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# 대기 중에는 주행용 센서(odom·엔코더·라이다·경로·TF) 구독을 끊어 CPU를 아낀다.
# 이 메시지들은 초당 100개 이상이라 대기 중에도 받기만 하면 라즈베리파이 한 코어의 약 20%를 쓴다.
# 마지막 요청 후 SENSOR_IDLE_RELEASE_S 동안 요청이 없으면 끊고, 다음 요청이 오면 다시 연결해 최신 값을 기다린다.
# (대기 프로그램 없이 처음 주행할 때도 구독은 0에서 시작하므로 같은 조건이다.)
SENSOR_IDLE_RELEASE_S = 30.
SENSOR_RESUME_TIMEOUT_S = 3.
SENSOR_SUBSCRIPTIONS = ('plan_subscription', 'odom_subscription', 'encoder_subscription', 'scan_subscription')


# 센서 구독 해제 (대기 중 CPU 절약).
def pause_sensors(nav):
    """구독을 해제하고 받은 값도 지운다. 다시 연결하면 반드시 새 메시지로 판단한다."""
    for name in SENSOR_SUBSCRIPTIONS:
        subscription = getattr(nav, name, None)
        if subscription is not None:
            nav.destroy_subscription(subscription)
            setattr(nav, name, None)
    if nav.tf_listener is not None:
        nav.tf_listener.unregister()
        nav.tf_listener = None
    nav.odom_received_at = nav.encoder_received_at = nav.scan_received_at = nav.plan_received_at = None
    nav.encoder_message = nav.scan_message = nav.plan_message = None
    nav.odom_stopped = False
    nav.plan_points = ()


# 센서 구독 재생성 후 최신 값 대기 (최대 3초).
def resume_sensors(module, nav, rclpy):
    """구독을 다시 만들고 odom·엔코더·라이다·지도 위치가 들어올 때까지 최대 3초 기다린다."""
    if nav.odom_subscription is not None:
        return
    qos = module.qos_profile_sensor_data
    nav.plan_subscription = nav.create_subscription(module.NavPath, 'plan', nav._on_plan, qos)
    nav.odom_subscription = nav.create_subscription(module.Odometry, 'odom', nav._on_odom, qos)
    nav.encoder_subscription = nav.create_subscription(module.SensorState, 'sensor_state', nav._on_encoder, qos)
    nav.scan_subscription = nav.create_subscription(module.LaserScan, 'scan', nav._on_scan, qos)
    nav.tf_buffer = module.Buffer()
    nav.tf_listener = module.TransformListener(nav.tf_buffer, nav)
    started = time.monotonic()
    deadline = started+SENSOR_RESUME_TIMEOUT_S
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=.02)
        if (nav.odom_received_at is not None and nav.encoder_received_at is not None
                and nav.scan_received_at is not None
                and nav.tf_buffer.can_transform(module.frame_name(nav, 'map'),
                                                module.frame_name(nav, 'base_footprint'), module.rclpy.time.Time())):
            print(f'센서 재연결 완료 ({time.monotonic()-started:.2f}초)', flush=True)
            return
    # 여기서 멈추지 않는다. 경로 코드가 각 단계에서 최신 값을 다시 확인하고, 없으면 움직이기 전에 실패한다.
    print('센서 재연결 대기 3초 초과: 경로 코드의 최신 값 확인에 맡김', flush=True)


# 요청한 쪽 연결이 끊기면 interrupt() 로 주행을 중단시킨다.
def monitor_disconnect(connection, finished, interrupt):
    while not finished.wait(.05):
        try:
            readable, _, _ = select.select([connection], [], [], 0)
            if readable and connection.recv(1, socket.MSG_PEEK) == b'':
                interrupt(); return
        except OSError:
            if not finished.is_set(): interrupt()
            return


# 실행 중 출력(print·ROS 로그)을 요청한 쪽 소켓으로도 보낸다 (서비스 로그에도 남김).
@contextlib.contextmanager
def forwarded_output(connection, lock):
    """Forward Python and rcutils logs; keep the same logs in the service journal."""
    sys.stdout.flush(); sys.stderr.flush()
    saved_out, saved_err = os.dup(1), os.dup(2)
    reader, writer = os.pipe()
    # 출력 파이프를 읽어 저널과 소켓으로 전달.
    def drain():
        decoder = codecs.getincrementaldecoder('utf-8')('replace')
        while True:
            raw = os.read(reader, 65536)
            if not raw: break
            os.write(saved_err, raw)
            text = decoder.decode(raw)
            if text:
                try:
                    with lock: connection.sendall((json.dumps({'type': 'log', 'text': text})+'\n').encode())
                except OSError: pass  # The disconnect monitor cancels the route.
    thread = threading.Thread(target=drain, daemon=True); thread.start()
    os.dup2(writer, 1); os.dup2(writer, 2); os.close(writer)
    try: yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(saved_out, 1); os.dup2(saved_err, 2)
        thread.join(timeout=3)
        if thread.is_alive():
            raise RuntimeError('Waypoint log reader did not finish')
        os.close(reader); os.close(saved_out); os.close(saved_err)


# 요청 1건 실행: 수락 응답 → 연결 감시 시작 → 경로 main() 실행.
def execute_request(module, nav, request, connection):
    lock = threading.Lock(); finished = threading.Event()
    watcher = threading.Thread(target=monitor_disconnect,
        args=(connection, finished, lambda: os.kill(os.getpid(), signal.SIGUSR1)), daemon=True)
    connection.sendall(b'{"type":"accepted"}\n')
    watcher.start()
    try:
        with forwarded_output(connection, lock):
            return module.main(argv=['nav2_waypoints', *request['argv']], prepared_nav=nav)
    finally:
        finished.set(); watcher.join(timeout=.2)


# 경로 코드 로드 → 내비게이터 준비 → 소켓 대기 루프 (요청마다 검사·센서 재연결·실행·결과 전송).
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--entry', default=os.environ.get('BURGER_WAYPOINT_ENTRY_PATH'))
    a = p.parse_args()
    if not a.entry: raise ValueError('BURGER_WAYPOINT_ENTRY_PATH is required')
    started = time.monotonic()
    loader = importlib.machinery.SourceFileLoader('burger2_waypoint_program', a.entry)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec); loader.exec_module(module)
    if 'prepared_nav' not in __import__('inspect').signature(module.main).parameters:
        raise RuntimeError('Waypoint entry does not support an explicit reusable navigator')
    code_hash = signature(a.entry)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from nav2_msgs.srv import ManageLifecycleNodes
    rclpy.init(args=['--ros-args', '-r', '/tf:=/burger2/tf', '-r', '/tf_static:=/burger2/tf_static'],
                signal_handler_options=SignalHandlerOptions.NO)
    endpoints = {(GetState, name+'/get_state') for name in
                 ('amcl', 'bt_navigator', 'controller_server', 'velocity_smoother', 'collision_monitor')}
    endpoints.update({(GetParameters, name+'/get_parameters') for name in
                      ('controller_server', 'behavior_server')})
    endpoints.add((ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes'))
    nav = prepared_navigator_class(module.WaypointNavigator, endpoints)(namespace='burger2')
    owner = {'mode': None, 'received': 0.}
    nav.create_subscription(String, 'motion_owner/status',
        lambda msg: owner.update(mode=msg.data, received=time.monotonic()), qos_profile_sensor_data)
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/'+str(os.getuid())))
    address = runtime/'burger2-waypoint-ready.sock'; meta = address.with_suffix('.json')
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stopping = False; executing = False
    # 종료 신호: 대기 중이면 루프 종료, 실행 중이면 주행 중단.
    def stop(*_):
        nonlocal stopping
        stopping = True
        if executing: raise KeyboardInterrupt
    # 연결 끊김 신호(SIGUSR1): 실행 중이면 주행 중단.
    def disconnected(*_):
        if executing: raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGUSR1, disconnected)
    try:
        address.unlink(missing_ok=True); server.bind(str(address)); os.chmod(address, 0o600)
        server.listen(1); server.setblocking(False)
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        meta.write_text(json.dumps({'pid': os.getpid(), 'boot': boot, 'entry': a.entry,
                                   'sha256': code_hash, 'startup_s': time.monotonic()-started}))
        print('Burger2 waypoint connection prepared; no movement sent', flush=True)
        last_request = time.monotonic()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(nav, timeout_sec=.05)
            if nav.odom_subscription is not None and time.monotonic()-last_request > SENSOR_IDLE_RELEASE_S:
                pause_sensors(nav)
            try: connection, _ = server.accept()
            except BlockingIOError: continue
            last_request = time.monotonic()
            with connection:
                connection.settimeout(2)
                accepted = False
                try:
                    request = json.loads(connection.makefile('rb').readline(16384))
                    validate_request(request, time.monotonic())
                    if signature(a.entry) != code_hash:
                        raise RuntimeError('Waypoint code changed; restart prepared worker')
                    resume_sensors(module, nav, rclpy)
                    if request['op'] in ('probe', 'verify'):
                        if request['op'] == 'verify':
                            # Diagnostic reads only: no goal, initial pose or speed command.
                            for name in ('amcl', 'bt_navigator'):
                                client = nav.create_client(GetState, name+'/get_state')
                                try:
                                    if not client.wait_for_service(timeout_sec=3):
                                        raise RuntimeError(name+' lifecycle connection missing')
                                    future = client.call_async(GetState.Request())
                                    rclpy.spin_until_future_complete(nav, future, timeout_sec=3)
                                    if (not future.done() or future.result() is None
                                            or future.result().current_state.label != 'active'):
                                        raise RuntimeError(name+' is not confirmed active')
                                finally:
                                    nav.destroy_client(client)
                            nav.verify_controllers({'forward'})
                            nav.verify_terminal_behavior()
                        result = {'type': 'result', 'code': 0, 'owner': owner,
                                  'namespace': nav.get_namespace(), 'motion_sent': False}
                    else:
                        if '--dry-run' not in request['argv']:
                            # Allow the just-confirmed ownership message to arrive; never bypass it.
                            end = time.monotonic()+1.
                            while (owner['mode'] != 'nav' or time.monotonic()-owner['received'] > .8) and time.monotonic() < end:
                                rclpy.spin_once(nav, timeout_sec=.02)
                            if owner['mode'] != 'nav' or time.monotonic()-owner['received'] > .8:
                                raise RuntimeError('Fresh nav ownership required')
                        # Retain only fresh sensor/TF connections; route parameters
                        # are reread by the unchanged waypoint main on EVERY call.
                        nav.goal_handle = nav.result_future = nav.feedback = nav.status = None
                        nav.plan_message = nav.plan_received_at = None; nav.plan_points = ()
                        connection.settimeout(5); executing = accepted = True
                        code = execute_request(module, nav, request, connection)
                        executing = False
                        result = {'type': 'result', 'code': int(code or 0)}
                    connection.sendall((json.dumps(result)+'\n').encode())
                except BaseException as exc:
                    executing = False
                    if accepted:
                        try: module.cancel_and_wait(nav)
                        except BaseException as cancel_error:
                            print('Waypoint cancel failed: '+str(cancel_error), flush=True)
                            stopping = True
                    print('Prepared waypoint request ended: '+str(exc), flush=True)
                    result = {'type': 'result', 'code': 130 if isinstance(exc, KeyboardInterrupt) else 1,
                              'error': str(exc), 'accepted': accepted}
                    try: connection.sendall((json.dumps(result)+'\n').encode())
                    except OSError: pass
                    if isinstance(exc, SystemExit) and not accepted: continue
                finally:
                    last_request = time.monotonic()
    finally:
        meta.unlink(missing_ok=True); address.unlink(missing_ok=True); server.close()
        if nav.tf_listener is not None: nav.tf_listener.unregister()
        nav.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
