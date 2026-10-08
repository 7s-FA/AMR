"""One bounded retry policy for mission steps, independent of failure codes."""
import hashlib
import json
import math
import time
from pathlib import Path


class RetryCancelled(KeyboardInterrupt):
    """Loss of the original operator's authorization must never resume motion."""


class RetryExhausted(RuntimeError):
    pass


class RetryPolicy:
    def __init__(self, retries, stop, recover, report, authorized=lambda: True,
                 deadline=math.inf, clock=time.monotonic):
        if type(retries) is not int or not 0 <= retries <= 3:
            raise ValueError('Retries must be 0..3')
        self.retries, self.stop, self.recover, self.report = retries, stop, recover, report
        self.authorized, self.deadline, self.clock = authorized, deadline, clock

    def check(self):
        if not self.authorized():
            raise RetryCancelled('명령 취소·긴급정지·제어권 변경: 재시도 금지')
        if self.clock() >= self.deadline:
            raise RetryExhausted('MISSION_RETRIES_EXHAUSTED: 전체 임무 시간 제한')

    def execute(self, stage, operation):
        last = None
        for attempt in range(self.retries + 1):
            self.check()
            try:
                if attempt:
                    self.report(stage, attempt, 'recovering', str(last))
                    self.recover(stage)
                    self.check()
                value = operation()
                if value is False:
                    raise RuntimeError('Step returned failure')
                return value
            except Exception as exc:
                last = exc
                self.report(stage, attempt, 'failed', str(exc))
                # Stop must succeed before a retry can be considered.
                self.stop()
                if 'ENCODER_DEPARTURE_FAILED' in str(exc) or 'Nav2 error_code=703' in str(exc):
                    raise RuntimeError(f'자동 재시도 금지: {stage}: {exc}') from exc
                self.check()
        raise RetryExhausted(f'MISSION_RETRIES_EXHAUSTED: {stage}, '
                             f'재시도 {self.retries}회 소진: {last}') from last


class RouteProgress:
    """A single command's completed legs, departure claim, and absolute turn target."""
    def __init__(self, path, waypoints, timeout, clock=time.monotonic):
        self.path, self.clock = Path(path), clock
        fingerprint = hashlib.sha256(json.dumps(waypoints, sort_keys=True,
                                   allow_nan=False).encode()).hexdigest()
        if self.path.exists():
            self.state = json.loads(self.path.read_text())
            if self.state.get('route') != fingerprint:
                raise ValueError('Retry route changed; no completed steps may be reused')
        else:
            self.state = dict(route=fingerprint, completed=0, deadline=clock()+timeout,
                              backup_claimed=False, backup_done=False, turn_done=False)
            self.save()
        if (type(self.state.get('completed')) is not int or
                not 0 <= self.state['completed'] <= len(waypoints) or
                not math.isfinite(self.state.get('deadline', math.nan))):
            raise ValueError('Invalid retry progress')

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.state, allow_nan=False))
        temp.replace(self.path)

    def complete(self, index):
        if index != self.state['completed'] + 1:
            raise ValueError('Waypoints must complete in order')
        self.state['completed'] = index
        self.save()

    def claim_backup(self):
        if self.state['backup_claimed']:
            raise RuntimeError('출차 후진 완료가 불확실합니다. 반복 후진하지 않습니다.')
        self.state['backup_claimed'] = True
        self.save()

    def backup_done(self):
        self.state['backup_done'] = True
        self.save()

    def turn_delta(self, yaw, requested_degrees):
        if 'turn_target' not in self.state:
            self.state['turn_target'] = yaw + math.radians(requested_degrees)
            self.save()
        delta = self.state['turn_target'] - yaw
        return math.atan2(math.sin(delta), math.cos(delta))

    def turn_done(self):
        self.state['turn_done'] = True
        self.save()
