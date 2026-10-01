"""ROS graph queries run in a separate process, never in a motor/IR callback."""
import multiprocessing as mp
import time

ERRORS = {0: None, 1: 'graph_not_ready', 2: 'cmd_vel_has_other_publisher_or_graph_not_ready',
          3: 'TwistStamped_motor_subscriber_required', 4: 'graph_query_failed'}


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


class GraphGuard:
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

    def close(self):
        self.stop.set()
        self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.terminate(); self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.kill(); self.process.join(timeout=.5)
