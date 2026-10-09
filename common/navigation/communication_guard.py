# ========================================================================
# 역할: 통신 감시자. 별도 프로세스에서 ROS 그래프를 주기적으로 조회해 '이 속도 토픽의 발행자가 정확히 나 하나인지',
#       '모터 쪽 구독자가 TwistStamped 로 붙어 있는지'를 확인한다. 결과는 공유 메모리로 전달 (제어 콜백을 막지 않게).
# 사용처: motion_owner.py(cmd_vel), docking_standby.py·docking_node.py(cmd_vel_direct), rest_ready_worker.py·rest_forward.py.
# 참고: 카메라 쪽(camera/communication_guard.py)도 같은 파일을 링크해서 쓴다.
# ========================================================================
"""ROS graph queries run in a separate process, never in a motor/IR callback."""
import multiprocessing as mp
import time

ERRORS = {0: None, 1: 'graph_not_ready', 2: 'cmd_vel_has_other_publisher_or_graph_not_ready',
          3: 'TwistStamped_motor_subscriber_required', 4: 'graph_query_failed'}


# 공유 메모리에서 최신 결과를 안전하게 읽는다 (쓰는 중이면 다시 읽기). 너무 오래된 결과는 오류로 본다.
def read_snapshot(state, now, max_age=1.0):
    # Bounded seqlock read: the writer commits only after slow queries finish.
    for _ in range(3):
        version = state[0]
        if int(version) % 2:
            continue
        updated, code, elapsed = state[1], state[2], state[3]
        if version == state[0]:
            if updated <= 0 or now - updated > max_age:
                return {'error': 'graph_monitor_stale', 'age_s': None if updated <= 0 else now-updated,
                        'query_ms': elapsed}
            return {'error': ERRORS.get(int(code), 'graph_query_failed'),
                    'age_s': now-updated, 'query_ms': elapsed}
    return {'error': 'graph_snapshot_unavailable', 'age_s': None, 'query_ms': None}


# 감시 프로세스 본체: period 마다 발행자/구독자 조회 → 결과 코드와 시각을 공유 메모리에 기록.
def run_guard(topic, expected_name, namespace, role, period, state, stop):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(role+'_communication_guard', namespace=namespace)
    try:
        while rclpy.ok() and not stop.is_set():
            started = time.monotonic()
            try:
                publishers = node.get_publishers_info_by_topic(topic)
                subscribers = node.get_subscriptions_info_by_topic(topic)
                code = 0
                if not publishers:
                    code = 1
                elif len(publishers) != 1 or publishers[0].node_name != expected_name or publishers[0].node_namespace != namespace:
                    code = 2
                elif not subscribers or any(s.topic_type != 'geometry_msgs/msg/TwistStamped' for s in subscribers):
                    code = 3
            except Exception:
                code = 4
            completed = time.monotonic()
            version = int(state[0])
            state[0] = version+1
            state[1], state[2], state[3] = completed, code, (completed-started)*1000
            state[0] = version+2
            # Keep graph discovery responsive, with no tight polling loop.
            deadline = started+period
            while not stop.is_set() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# 감시 프로세스를 띄우고 결과를 읽게 해 주는 클래스.
class GraphGuard:
    # 로봇 네임스페이스 확인 후 spawn 방식으로 감시 프로세스 시작.
    def __init__(self, topic, expected_name, namespace, role, period=.2, max_age=1.0):
        if namespace not in ('/burger1', '/burger2') or not topic.startswith(namespace+'/'):
            raise ValueError('Communication guard topic must belong to its robot')
        ctx = mp.get_context('spawn')
        self.state = ctx.Array('d', [0., 0., 1., 0.], lock=False)
        self.stop = ctx.Event()
        self.max_age = max_age
        self.cached = None
        self.cached_at = None
        self.process = ctx.Process(target=run_guard,
            args=(topic, expected_name, namespace, role, period, self.state, self.stop), daemon=True)
        self.process.start()

    # 현재 감시 결과 (쓰는 중이라 못 읽으면 직전 결과를 유효 시간 안에서만 재사용).
    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        result = read_snapshot(self.state, now, self.max_age)
        if result['error'] == 'graph_snapshot_unavailable' and self.cached is not None:
            # An in-progress commit is not a disconnected graph. Reuse only a
            # previously committed snapshot, within the same freshness bound.
            age = now-self.cached_at
            return {**self.cached, 'age_s': age,
                    'error': self.cached['error'] if age <= self.max_age else 'graph_monitor_stale'}
        if result['age_s'] is not None:
            self.cached = result
            self.cached_at = now-result['age_s']
        return result

    # 감시 프로세스 종료 (안 끝나면 terminate → kill).
    def close(self):
        self.stop.set()
        self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.terminate(); self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.kill(); self.process.join(timeout=.5)
