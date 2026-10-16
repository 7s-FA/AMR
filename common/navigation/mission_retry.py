# ========================================================================
# 역할: 임무 단계 공통 재시도 정책과 경로 진행 기록(재시도 때 끝난 구간·출차를 반복하지 않게).
# 실행: 단독 실행 안 함. sequence_runner.py(RetryPolicy), nav2_waypoints.py(RouteProgress)가 사용.
# 설정: <로봇>/navigation/mission_retry.json 의 retries (0~3).
# ========================================================================
"""One bounded retry policy for mission steps, independent of failure codes."""
import hashlib
import json
import math
import time
from pathlib import Path


# 명령 취소·긴급정지·제어권 변경 시 재시도를 멈추게 하는 예외.
class RetryCancelled(KeyboardInterrupt):
    """Loss of the original operator's authorization must never resume motion."""


# 재시도 횟수나 전체 시간 제한을 다 쓴 경우의 예외.
class RetryExhausted(RuntimeError):
    pass


# 단계 실패 시 '정지 → 복구 확인 → 같은 단계 재실행'을 정해진 횟수만큼 반복.
class RetryPolicy:
    # 재시도 횟수(0~3), 정지·복구·보고 함수, 권한 확인 함수, 전체 마감 시각을 받는다.
    def __init__(self, retries, stop, recover, report, authorized=lambda: True,
                 deadline=math.inf, clock=time.monotonic):
        if type(retries) is not int or not 0 <= retries <= 3:
            raise ValueError('Retries must be 0..3')
        self.retries, self.stop, self.recover, self.report = retries, stop, recover, report
        self.authorized, self.deadline, self.clock = authorized, deadline, clock

    # 재시도 전마다 권한(취소/긴급정지 여부)과 전체 시간 제한 확인.
    def check(self):
        if not self.authorized():
            raise RetryCancelled('명령 취소·긴급정지·제어권 변경: 재시도 금지')
        if self.clock() >= self.deadline:
            raise RetryExhausted('MISSION_RETRIES_EXHAUSTED: 전체 임무 시간 제한')

    # 단계 실행. 실패하면 정지·복구 후 다시, 횟수를 다 쓰면 RetryExhausted.
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


# 명령 1건의 경로 진행 기록 파일 (완료한 웨이포인트, 출차 후진/회전 완료 여부, 회전 목표 각도).
class RouteProgress:
    """A single command's completed legs, departure claim, and absolute turn target."""
    # 기록 파일을 읽거나 새로 만든다. 경로가 바뀌었으면(지문 불일치) 거부.
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

    # 기록 파일 저장 (임시 파일→교체).
    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.state, allow_nan=False))
        temp.replace(self.path)

    # 웨이포인트 index 완료 표시 (순서대로만).
    def complete(self, index):
        if index != self.state['completed'] + 1:
            raise ValueError('Waypoints must complete in order')
        self.state['completed'] = index
        self.save()

    # 출차 후진을 시작한다고 표시. 이미 표시돼 있으면 반복 후진하지 않도록 거부.
    def claim_backup(self):
        if self.state['backup_claimed']:
            raise RuntimeError('출차 후진 완료가 불확실합니다. 반복 후진하지 않습니다.')
        self.state['backup_claimed'] = True
        self.save()

    # 출차 후진 완료 표시.
    def backup_done(self):
        self.state['backup_done'] = True
        self.save()

    # 출차 회전의 남은 각도 계산 (처음 목표 각도를 고정해 재시도해도 같은 방향으로 맞춤).
    def turn_delta(self, yaw, requested_degrees):
        if 'turn_target' not in self.state:
            self.state['turn_target'] = yaw + math.radians(requested_degrees)
            self.save()
        delta = self.state['turn_target'] - yaw
        return math.atan2(math.sin(delta), math.cos(delta))

    # 출차 회전 완료 표시.
    def turn_done(self):
        self.state['turn_done'] = True
        self.save()
