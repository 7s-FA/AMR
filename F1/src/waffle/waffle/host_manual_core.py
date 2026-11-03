"""F1 manual control interlock, shared by the motion nodes in one process.

The existing calibrated navigation/docking/lift sequences remain responsible for
movement. A latched stop gates every actuator callback under the same lock and
cancels outstanding Nav2 goals before release is allowed.
"""
from collections import deque
from functools import wraps
import queue
import os
import threading
import time
import uuid

COMMANDS = ('GO_TO_ASM', 'GO_TO_PARK', 'GO_TO_STORAGE_A_01',
            'GO_TO_STORAGE_A_02', 'GO_TO_STORAGE_B_01', 'GO_TO_STORAGE_B_02')


LIFT_COMMANDS = {'LIFT_DOWN': 'down', 'LIFT_UP': 'up', 'LIFT_1_DOWN': '1st_floor_down', 'LIFT_1_UP': '1st_floor_up', 'LIFT_2_DOWN': '2nd_floor_down', 'LIFT_ASM': 'middle'}
LIFT_FROM = {
    'up': {'down', 'up', 'middle', '2nd_floor_down', '1st_floor_up', '1st_floor_down'},
    '1st_floor_down': {'down', '1st_floor_up', '1st_floor_down'},
    '1st_floor_up': {'down', '1st_floor_down', '1st_floor_up'},
    '2nd_floor_down': {'down', 'up', '2nd_floor_down'},
    'middle': {'down', 'up', 'middle'},
}

class ManualControl:
    def __init__(self, shared, zero):
        self.shared, self.zero = shared, zero
        self.lock = threading.RLock()
        self.server_id = uuid.uuid4().hex
        self.revision = 0
        self.generation = 0
        self.stopped = os.environ.get('F1_PRESERVE_LOCALIZATION') == '1'
        self.error = ''
        self.job = None
        self.events = deque(maxlen=40)
        self.seen = set()
        self.leases = deque(maxlen=6)
        self.goal_requests = set()
        self.nav_results = {}
        self.service_requests = set()
        self.cancel_requested = set()
        self.cancel_error = ''
        self.nodes = None

    def wrap(self, cls, method, kind='motion'):
        original = getattr(cls, method)

        @wraps(original)
        def guarded(node, *args, **kwargs):
            with self.lock:
                if method == 'on_control_selected' and getattr(args[0], '_manual_generation', -1) != self.generation:
                    return None
                if self.stopped:
                    if kind == 'service':
                        response = args[1]
                        response.success, response.message = False, 'F1 stopped'
                        return response
                    if kind == 'output':
                        node.publisher.publish(self.zero())
                    return None
                if (method == 'run' and self.nodes and node in (self.nodes[0], self.nodes[2])
                        and self.busy() and self.job['command'] in LIFT_COMMANDS):
                    return None  # Lift-only job cannot advance an old navigation/docking sequence.
                try:
                    return original(node, *args, **kwargs)
                except Exception as exc:
                    if self.nodes:
                        self.stop('제어 오류: ' + str(exc))
                    raise
        setattr(cls, method, guarded)

    def install(self, navigator, linear, docking, selector):
        for cls, methods in ((navigator, ('run', 'on_control_selected')),
                             (linear, ('run',)), (docking, ('run',)),
                             (selector, ('on_nav_velocity', 'on_dock_velocity', 'on_backup_velocity'))):
            for method in methods:
                self.wrap(cls, method)
        for method in ('on_control_request', 'on_backup_control'):
            self.wrap(selector, method, 'service')
        self.wrap(selector, 'publish_selected', 'output')

    def bind(self, navigator, linear, docking, selector):
        self.nodes = navigator, linear, docking, selector
        original = navigator.nav_to_pose_client.send_goal_async

        def send_goal(*args, **kwargs):
            future = original(*args, **kwargs)
            self.goal_requests.add(future)
            future.add_done_callback(self.goal_received)
            return future
        navigator.nav_to_pose_client.send_goal_async = send_goal
        for client in (navigator.control_client, navigator.backup_client):
            original_call = client.call_async

            def call(*args, _original=original_call, **kwargs):
                future = _original(*args, **kwargs)
                future._manual_generation = self.generation
                self.service_requests.add(future)
                future.add_done_callback(self.service_received)
                return future
            client.call_async = call

    def service_received(self, future):
        with self.lock:
            self.service_requests.discard(future)

    def goal_received(self, future):
        with self.lock:
            self.goal_requests.discard(future)
            try:
                handle = future.result()
                if handle and handle.accepted:
                    result = handle.get_result_async()
                    self.nav_results[result] = handle
                    result.add_done_callback(self.nav_finished)
                    if self.stopped:
                        self.cancel_navigation()
            except Exception as exc:
                self.cancel_error = 'Nav2 접수 결과 미확인: ' + str(exc)
                self.stop(self.cancel_error)

    def nav_finished(self, future):
        with self.lock:
            try:
                result = future.result()
                if result is None or result.status not in (4, 5, 6):
                    raise RuntimeError('terminal status missing')
            except Exception as exc:
                self.cancel_error = 'Nav2 종료 결과 미확인: ' + str(exc)
                self.stop(self.cancel_error)
                return
            self.nav_results.pop(future, None)
            self.cancel_requested.discard(future)

    def cancel_navigation(self):
        for future, handle in list(self.nav_results.items()):
            if future in self.cancel_requested:
                continue
            self.cancel_requested.add(future)
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.cancel_error = 'Nav2 취소 요청 실패: ' + str(exc)

    def event(self, request_id, command, status, message):
        item = dict(id=request_id, command=command, status=status, message=message)
        self.events.append(item)
        return item

    def stop(self, reason='운영자 정지'):
        with self.lock:
            self.stopped = True
            self.generation += 1
            self.revision += 1
            self.error = reason
            s = self.shared
            s.navigator_start = False
            if s.linear_action != 'stop':
                s.linear_state = ''  # Interrupted timed movement has no known height.
            s.linear_action = 'stop'
            s.docking_control_acquired = False
            s.docking_state = 'docking_stop'
            s.sequence_phase = ''
            if self.nodes:
                nav, linear, dock, selector = self.nodes
                # All callbacks that can restore these outputs share this lock.
                selector.mode, selector.last_command, selector.last_received = 'STOP', None, 0.0
                selector.publisher.publish(self.zero())
                linear.output_A.off()
                linear.output_B.off()
                dock.publish_velocity(0.0, 0.0)
                dock.stop_camera()
                self.cancel_navigation()
            if self.job and self.job['status'] == 'running':
                self.job.update(status='cancelled', message=reason)
                self.event(self.job['id'], self.job['command'], 'cancelled', reason)

    def stationary(self):
        if not self.nodes:
            return False
        nav = self.nodes[0]
        odom = nav.backup_odom
        return (odom is not None and time.monotonic() - nav.backup_odom_received < 0.5
                and abs(odom[3]) < 0.01 and abs(odom[4]) < 0.03)

    def can_release(self):
        return (self.stopped and not self.goal_requests and not self.nav_results
                and not self.service_requests and not self.cancel_error and self.stationary())

    def release(self):
        if not self.can_release():
            raise ValueError('Nav2 종료·실제 정지 확인 중입니다. 상태를 확인한 뒤 다시 해제하세요.')
        nav, linear, dock, selector = self.nodes
        nav.operating = False
        nav.goal_future = nav.result_future = None
        nav.active_index = None
        nav.control_mode = None
        nav.control_transition_pending = False
        nav.reset_backup()
        nav.extra_wait_started = None
        dock.state, dock.prepare_started = 'WAIT', None
        while True:
            try:
                dock.commands.get_nowait()
            except queue.Empty:
                break
        s = self.shared
        s.navigator_start = s.navigator_end = False
        s.navigator_end_msg = s.navigator_index = s.previous_command = ''
        s.sequence_phase, s.docking_state = '', 'idle'
        s.departure_reset_requested = True
        # An interrupted timed lift no longer has a trustworthy height estimate.
        s.linear_state = ''
        s.linear_preaction = s.linear_action = 'stop'
        selector.mode, selector.last_command, selector.last_received = 'STOP', None, 0.0
        self.stopped, self.error = False, ''
        self.revision += 1

    def reset_empty(self, confirmed):
        if confirmed is not True:
            raise ValueError('F1 포크·조립대 비움 확인이 필요합니다.')
        if (self.busy() or self.goal_requests or self.nav_results or self.service_requests
                or self.cancel_error or not self.stationary()
                or self.shared.linear_action != 'stop'):
            raise ValueError('F1 정지·이전 작업 종료·리니어 정지 확인 필요')
        self.stop('전체 초기화 · 남은 작업 정리')
        self.release()
        self.shared.linear_pallet_exist = False
        self.shared.storage_state = False
        self.shared.docking_place = ''
        self.shared.linear_command_sent_flag = False
        self.job = None
        self.revision += 1

    def submit(self, command, request_id, empty_confirmed=False, parked_confirmed=False, return_mode=None):
        with self.lock:
            if command == 'EMER_STOP':
                self.stop()
                return self.event(request_id, command, 'success', '정지 유지 · 이전 작업 재개 없음')
            if command == 'RESTART':
                self.release()
                return self.event(request_id, command, 'success', '정지 해제 · 새 목적지 선택 가능')
            if command == 'RESET_EMPTY':
                self.reset_empty(empty_confirmed)
                return self.event(request_id, command, 'success', '작업·팔레트 상태 정리 완료 · 이동 없음')
            if command == 'PARK_IN_PLACE':
                if (parked_confirmed is not True or self.stopped or self.busy()
                        or self.goal_requests or self.nav_results or self.service_requests
                        or self.cancel_error or not self.stationary()
                        or self.shared.linear_pallet_exist or self.shared.linear_action != 'stop'):
                    raise ValueError('초기 위치·빈 포크·정지 확인 필요')
                s = self.shared
                self.nodes[0].departure_from_park = True
                s.docking_place = 'GO_TO_PARK'
                s.navigator_start = False
                s.navigator_end, s.navigator_end_msg = True, 'Success'
                s.linear_action = 'down'
                self.revision += 1
                self.job = self.event(request_id, command, 'running', '차체 이동 생략 · 리프트 하강')
                self.job_started = time.monotonic()
                return self.job
            if command in LIFT_COMMANDS:
                s = self.shared
                if (self.stopped or self.cancel_error or self.busy() or self.goal_requests
                        or self.nav_results or self.service_requests or not self.stationary()
                        or s.navigator_start or getattr(self.nodes[0], 'operating', False)
                        or s.linear_action != 'stop'):
                    raise ValueError('F1 이동·작업 종료와 정지 해제 확인 필요')
                selector = self.nodes[3]
                selector.mode, selector.last_command, selector.last_received = 'STOP', None, 0.0
                selector.publisher.publish(self.zero())
                target = LIFT_COMMANDS[command]
                if target != 'down' and s.linear_state not in LIFT_FROM[target]:
                    raise ValueError('현재 높이에서 직접 조절 불가 · 포크 주변 확인 후 최하단 내림을 먼저 실행하세요.')
                s.navigator_start = False
                s.sequence_phase = 'manual_lift:' + target
                s.linear_preaction = 'stop'
                s.linear_command_sent_flag = False
                s.linear_action = target
                self.revision += 1
                self.job = self.event(request_id, command, 'running', '리니어 높이 조절 중')
                self.job_started = time.monotonic()
                return self.job
            if command not in COMMANDS:
                raise ValueError('지원하지 않는 F1 목적지')
            if self.stopped or self.cancel_error:
                raise ValueError('정지 상태입니다. 현장 확인 후 정지 해제가 필요합니다.')
            if self.busy() or self.goal_requests or self.nav_results or self.service_requests:
                raise ValueError('F1 작업 중입니다.')
            if not self.stationary():
                raise ValueError('F1 정지 상태·오도메트리 확인 필요')
            s = self.shared
            if return_mode is not None:
                if command != 'GO_TO_PARK' or return_mode != 'direct':
                    raise ValueError('지원하지 않는 F1 복귀 방식')
                if (not getattr(self.nodes[0], 'operator_direct_return_supported', False)
                        or s.linear_pallet_exist or s.linear_action != 'stop'):
                    raise ValueError('직접 복귀 기능·빈 포크·리니어 정지 확인 필요')
            s.operator_return_mode = return_mode  # Clear on every new command, never sticky.
            s.previous_command = s.navigator_index if s.navigator_end_msg == 'Success' else ''
            s.navigator_end, s.navigator_end_msg = False, ''
            s.navigator_index = command
            s.departure_reset_requested = True
            s.navigator_start = True
            self.revision += 1
            self.job = self.event(request_id, command, 'running', '접수')
            self.job_started = time.monotonic()
            return self.job

    def busy(self):
        return bool(self.job and self.job['status'] == 'running')

    def receive(self, request):
        with self.lock:
            request_id, command = request.get('id'), request.get('command')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 100:
                return
            if request_id in self.seen:
                return  # Never replay a received request, including a late duplicate.
            if len(self.seen) >= 10000:
                self.event(request_id, command, 'rejected', '요청 기록 한도: 서버 점검 필요')
                return
            self.seen.add(request_id)
            try:
                if command != 'EMER_STOP':
                    if (request.get('server_id') != self.server_id
                            or request.get('revision') != self.revision
                            or not any(token == request.get('lease') and time.monotonic()-stamp < 3
                                       for token, stamp in self.leases)):
                        raise ValueError('상태가 바뀌었거나 요청이 만료됐습니다. 최신 상태를 확인하세요.')
                self.submit(command, request_id, request.get('empty_confirmed') is True,
                            request.get('parked_confirmed') is True, request.get('return_mode'))
            except (ValueError, TypeError) as exc:
                self.event(request_id, command, 'rejected', str(exc))

    def snapshot(self):
        with self.lock:
            s = self.shared
            if self.busy() and self.job['command'] in LIFT_COMMANDS:
                target = LIFT_COMMANDS[self.job['command']]
                if not self.stationary():
                    self.stop('리니어 수동 조절 중 차체 정지 확인 실패')
                elif time.monotonic() - self.job_started > 30:
                    self.stop('리니어 수동 조절 제한 시간 초과')
                elif s.linear_action == 'stop':
                    if s.linear_state != target:
                        self.stop('리니어 목표 높이 기록 불일치')
                    else:
                        s.sequence_phase = ''
                        self.job.update(status='success', message='리니어 시간 기준 동작 완료 · 실제 높이 확인 필요')
                        self.event(self.job['id'], self.job['command'], 'success', self.job['message'])
                        self.revision += 1
            elif self.busy():
                if time.monotonic() - self.job_started > 600:
                    self.stop('F1 작업 제한 시간 600초 초과')
                elif s.navigator_end and s.navigator_end_msg != 'Success':
                    self.stop(s.navigator_end_msg or 'F1 이동 실패')
                elif s.docking_state == 'docking_ASM_stopped':
                    self.stop('도킹 중단 · 현장 및 F1 로그 확인 필요')
                elif s.navigator_end and s.linear_action == 'stop' and not self.nav_results:
                    self.job.update(status='success', message='이동·도킹·리니어 완료')
                    self.event(self.job['id'], self.job['command'], 'success', self.job['message'])
                    self.revision += 1
            lease = uuid.uuid4().hex
            self.leases.append((lease, time.monotonic()))
            return dict(lift_commands=list(LIFT_COMMANDS), operator_return_modes=(['direct'] if self.nodes and getattr(self.nodes[0], 'operator_direct_return_supported', False) else []),
                        protocol=1, server_id=self.server_id, revision=self.revision,
                        lease=lease, stopped=self.stopped, busy=self.busy(),
                        can_release=self.can_release(), stationary=self.stationary(),
                        job=self.job, events=list(self.events), error=self.error or self.cancel_error,
                        phase=s.sequence_phase, docking=s.docking_state,
                        linear_action=s.linear_action, linear_state=s.linear_state,
                        pallet=bool(s.linear_pallet_exist))
