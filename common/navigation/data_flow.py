"""Bounded live telemetry; no motor commands, GPIO operations or ROS discovery."""
import time


class EdgeHeartbeat:
    """Emit edges immediately and unchanged values at a bounded heartbeat rate."""
    def __init__(self, period):
        if period <= 0:
            raise ValueError('Heartbeat period must be positive')
        self.period = period
        self.last = None
        self.at = float('-inf')

    def due(self, value, now):
        if value != self.last or now - self.at >= self.period:
            self.last, self.at = value, now
            return True
        return False


class FreshSequence:
    """Accept strictly newer valid source stamps; an old sample never renews health."""
    def __init__(self):
        self.last = -1

    def accept(self, stamp_ns, now_ns, max_age_s, future_s=.1):
        age = (now_ns-stamp_ns)/1e9
        if stamp_ns <= self.last or stamp_ns <= 0 or not -future_s <= age <= max_age_s:
            return False
        self.last = stamp_ns
        return True


class CallbackMetrics:
    """Aggregate actual callback CPU time; excludes DDS/deserialization and other nodes."""
    def __init__(self):
        self.started = time.monotonic()
        self.stats = {}

    def begin(self):
        return time.thread_time_ns(), time.monotonic_ns()

    def end(self, name, mark, age_s=None):
        cpu_ns = max(0, time.thread_time_ns()-mark[0])
        wall_ns = max(0, time.monotonic_ns()-mark[1])
        s = self.stats.setdefault(name, {'count': 0, 'cpu_ns': 0, 'wall_ns': 0,
                                        'max_wall_ns': 0, 'max_age_s': None})
        s['count'] += 1
        s['cpu_ns'] += cpu_ns
        s['wall_ns'] += wall_ns
        s['max_wall_ns'] = max(s['max_wall_ns'], wall_ns)
        if age_s is not None:
            s['max_age_s'] = max(age_s, s['max_age_s'] or 0.)

    def report(self):
        elapsed = max(.001, time.monotonic()-self.started)
        return {name: {'count': s['count'], 'cpu_ms_total': round(s['cpu_ns']/1e6, 4),
                       'elapsed_s': round(elapsed, 4), 'callback_cpu_one_core_pct':
                       round(s['cpu_ns']/1e9/elapsed*100, 4),
                       'mean_callback_wall_ms': round(s['wall_ns']/max(1,s['count'])/1e6,4),
                       'max_callback_wall_ms': round(s['max_wall_ns']/1e6,4),
                       'max_source_age_s': s['max_age_s']}
                for name, s in self.stats.items()}


def runtime_robot(config):
    robot = config['cmd_topic'].strip('/').split('/')[0]
    if robot not in ('burger1', 'burger2'):
        raise ValueError('Unknown robot namespace')
    for key in ('cmd_topic', 'motor_state_topic', 'motor_power_service', 'odom_topic'):
        if not config[key].startswith('/'+robot+'/'):
            raise ValueError('Cross-robot topic: '+key)
    if config['base_frame'] != robot+'/base_footprint':
        raise ValueError('Cross-robot base frame')
    return robot
