# ========================================================================
# 역할: 상태 방송·진단 보조 클래스 모음 (모터·GPIO 동작 없음).
# 사용처: motion_owner.py(EdgeHeartbeat), docking_node.py(EdgeHeartbeat, CallbackMetrics, runtime_robot).
# 참고: 카메라 쪽(camera/data_flow.py)도 같은 파일을 링크해서 쓴다.
# ========================================================================
"""Bounded live telemetry; no motor commands, GPIO operations or ROS discovery."""
import time


# 값이 바뀌면 바로, 안 바뀌면 period 마다 한 번만 '보내라'고 알려 주는 도우미 (방송량 줄이기).
class EdgeHeartbeat:
    """Emit edges immediately and unchanged values at a bounded heartbeat rate."""
    # 방송 주기 설정.
    def __init__(self, period):
        if period <= 0:
            raise ValueError('Heartbeat period must be positive')
        self.period = period
        self.last = None
        self.at = float('-inf')

    # 지금 보내야 하면 True.
    def due(self, value, now):
        if value != self.last or now - self.at >= self.period:
            self.last, self.at = value, now
            return True
        return False


# 콜백별 CPU·실행 시간 통계 (도킹 진단 로그용).
class CallbackMetrics:
    """Aggregate actual callback CPU time; excludes DDS/deserialization and other nodes."""
    # 통계 시작 시각과 빈 통계표.
    def __init__(self):
        self.started = time.monotonic()
        self.stats = {}

    # 콜백 시작 시점의 스레드 CPU·벽시계 시각.
    def begin(self):
        return time.thread_time_ns(), time.monotonic_ns()

    # 콜백 끝: 횟수·CPU·최대 지연 누적.
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

    # 누적 통계를 보고용 사전으로 변환.
    def report(self):
        elapsed = max(.001, time.monotonic()-self.started)
        return {name: {'count': s['count'], 'cpu_ms_total': round(s['cpu_ns']/1e6, 4),
                       'elapsed_s': round(elapsed, 4), 'callback_cpu_one_core_pct':
                       round(s['cpu_ns']/1e9/elapsed*100, 4),
                       'mean_callback_wall_ms': round(s['wall_ns']/max(1,s['count'])/1e6,4),
                       'max_callback_wall_ms': round(s['max_wall_ns']/1e6,4),
                       'max_source_age_s': s['max_age_s']}
                for name, s in self.stats.items()}


# 도킹 설정의 토픽 이름에서 로봇(burger1/2)을 뽑고, 모든 토픽이 같은 로봇 것인지 검사.
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
