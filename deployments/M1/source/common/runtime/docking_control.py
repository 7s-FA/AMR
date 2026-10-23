# ========================================================================
# 역할: 마커 도킹 상태 기계 (ROS 와 무관한 순수 계산). 카메라 마커 위치·odom 으로 속도 명령을 계산한다.
#   상태: IDLE → ALIGN(마커 보고 정렬) → FINAL_APPROACH(직진) → STOPPING(정지 확인) → DOCKED.
#         이상 시 FAULT/STOPPED, 0 속도. 시야 끊김(VISION_WAIT)·odom 끊김(ODOM_WAIT)은 멈춘 채 기다렸다가 재개.
#   직진 정지 (2026-10-16, IR 센서 제거): 정렬 확정 순간 카메라로 잰 보드 거리(z0)에서 엔코더 이동 거리를 빼
#     남은 거리가 encoder_stop_distance_m(카메라~보드)이 되면 멈춘다. 두 가지 방식:
#       - 속도 방식(버거2, OpenCR): 직진 속도로 가다가 마지막 encoder_slow_zone_m 은 천천히, 밀림만큼 미리 정지.
#       - 펄스 이동(버거3, STM32 펌웨어 v6.7 move): 이동 펄스를 노드에 요청하고 STM32 결과(DONE)를 기다린다.
#     직진 중 바퀴가 막혀 1.5초 안 움직이면 정지(final_approach_stalled).
# 사용처: docking_node.py 의 DockingNode 가 10ms 마다 tick() 을 부른다. 설정은 docking.yaml 의 control 항목.
# 단위: SI(m, rad, s). 로봇 기준 x=앞, y=왼쪽.
# ========================================================================
"""ROS-independent docking state machine. SI units; base x forward, y left."""
from dataclasses import dataclass, fields
import math

MOVE_M_PER_PULSE = 0.033 * 2 * math.pi / 4096   # 바퀴 반지름 33mm, XL430 4096펄스/바퀴 → 약 0.0506mm
BOOL_FIELDS = ('visual_entry_alignment', 'odom_recovery_enabled', 'move_final_enabled', 'near_angular_taper')
NONNEGATIVE_FIELDS = ('alignment_min_distance_m', 'angular_kd', 'encoder_stop_lead_m', 'encoder_slow_zone_m', 'encoder_slow_lead_m')


# 각도를 -π~π 로 정규화.
def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# 값을 ±limit 안으로 자른다.
def clamp(value, limit):
    return max(-limit, min(limit, value))


# 도킹 제어 설정값 묶음 (docking.yaml control, 모드별 parking_control/assembly_control 이 덮어씀).
@dataclass
class Settings:
    # 카메라 장착 보정 (로봇 중심 기준)
    camera_forward_m: float = 0.0
    camera_left_m: float = 0.0
    camera_yaw_deg: float = 0.0
    # 정렬: 정렬 지점(카메라~보드), 허용 오차, 유지 시간, 속도
    visual_entry_alignment: bool = False  # Stop and face the approach target before translating.
    alignment_min_distance_m: float = 0.0  # 0 disables the pre-alignment clearance guard.
    staging_distance_m: float = 0.25
    lateral_tolerance_m: float = 0.005
    yaw_tolerance_deg: float = 2.0
    aligned_hold_s: float = 0.7
    align_speed_mps: float = 0.04
    final_speed_mps: float = 0.03
    # 회전 명령 한도·이득
    max_angular_rps: float = 0.25
    min_angular_rps: float = 0.03
    near_angular_rps: float = 0.16
    near_angular_taper: bool = False      # 정렬 지점보다 가까울수록 근거리 회전 한도를 줄임 (가까우면 조금만 돌아도 각도가 크게 바뀜)
    near_angular_min_scale: float = 0.4   # 최소 시야 거리에서의 근거리 회전 한도 배율
    steering_accel_rps2: float = 0.8
    far_steering_gain: float = 1.25
    angular_speed_scale: float = 1.0
    angular_kd: float = 0.0
    angular_rate_filter_s: float = 0.1
    # 입력 끊김 처리
    observation_timeout_s: float = 0.5
    vision_recovery_timeout_s: float = 2.0
    vision_recovery_hold_s: float = 0.3
    vision_recovery_fresh_s: float = 0.25
    max_vision_recoveries: int = 3
    odom_timeout_s: float = 0.3
    odom_recovery_enabled: bool = False
    odom_recovery_timeout_s: float = 2.0
    odom_recovery_hold_s: float = 0.3
    odom_recovery_fresh_s: float = 0.15
    max_odom_recoveries: int = 3
    # 안전 한도
    max_final_distance_m: float = 0.30
    max_final_time_s: float = 12.0
    max_total_distance_m: float = 1.0
    max_total_time_s: float = 90.0
    max_start_distance_m: float = 0.9
    min_visual_distance_m: float = 0.12
    stopped_hold_s: float = 0.5
    # 엔코더 거리 정지 (카메라~보드 남은 거리 목표)
    encoder_stop_distance_m: float = 0.01
    encoder_stop_lead_m: float = 0.0      # 속도 방식: 정지 명령 뒤 밀리는 거리만큼 미리 정지
    encoder_slow_zone_m: float = 0.0      # 속도 방식: 목표 앞 이 거리부터 천천히 (0 이면 안 씀)
    encoder_slow_speed_mps: float = 0.01  # 그 구간 속도 (OpenCR 은 0.01m/s 단위)
    encoder_slow_lead_m: float = 0.001    # 천천히 갈 때 미리 정지 거리
    # 펄스 이동 (STM32 move 명령)
    move_final_enabled: bool = False
    move_speed_raw: int = 44              # XL430 속도 단위(0.229rpm). 44 ≈ 3.5cm/s. 30 미만은 목표 앞에서 멈춰 DONE 이 안 날 수 있음
    move_timeout_s: float = 15.0          # move 명령 뒤 완료까지 최대 시간

    # 설정값 범위 검사 (위험한 값이면 시작 전에 거부).
    def __post_init__(self):
        for f in fields(self):
            x = getattr(self, f.name)
            if f.name in BOOL_FIELDS:
                if type(x) is not bool:
                    raise ValueError(f'{f.name} must be Boolean')
                continue
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                raise ValueError(f'{f.name} must be finite')
            if f.name in NONNEGATIVE_FIELDS:
                if x < 0:
                    raise ValueError(f'{f.name} must be nonnegative')
            elif not f.name.startswith('camera_') and x <= 0:
                raise ValueError(f'{f.name} must be positive')
        if type(self.max_vision_recoveries) is not int:
            raise ValueError('max_vision_recoveries must be an integer')
        if (type(self.max_odom_recoveries) is not int
                or self.odom_recovery_fresh_s > self.odom_timeout_s
                or self.odom_recovery_hold_s >= self.odom_recovery_timeout_s):
            raise ValueError('Invalid odometry recovery timing')
        if not (self.vision_recovery_fresh_s <= self.observation_timeout_s
                and self.vision_recovery_hold_s < self.vision_recovery_timeout_s):
            raise ValueError('Invalid vision recovery timing')
        if not (0.01 <= self.final_speed_mps <= self.align_speed_mps <= .08):
            raise ValueError('Require 0.01 <= final speed <= align speed <= 0.08 m/s')
        if self.alignment_min_distance_m and not (
                self.min_visual_distance_m < self.alignment_min_distance_m <= self.staging_distance_m):
            raise ValueError('Invalid pre-alignment clearance distance')
        if not (self.min_visual_distance_m < self.staging_distance_m < self.max_start_distance_m):
            raise ValueError('Invalid visual/staging/start distances')
        if not (self.min_angular_rps <= self.max_angular_rps <= .5):
            raise ValueError('Invalid angular speed limits')
        if not 0.1 <= self.near_angular_min_scale <= 1.0:
            raise ValueError('near_angular_min_scale must be 0.1..1.0')
        if self.encoder_stop_lead_m >= .03:
            raise ValueError('encoder_stop_lead_m must be < 0.03 m')
        if self.encoder_slow_zone_m > 0 and not (0.005 <= self.encoder_slow_speed_mps <= self.final_speed_mps
                                                  and self.encoder_slow_zone_m <= 0.05 and self.encoder_slow_lead_m < 0.01):
            raise ValueError('Invalid encoder slow zone settings')
        if type(self.move_speed_raw) is not int or not 30 <= self.move_speed_raw <= 100:
            raise ValueError('move_speed_raw must be an integer 30..100')


# 도킹 상태 기계 본체.
class DockingControl:
    ACTIVE = {'ALIGN', 'VISION_WAIT', 'ODOM_WAIT', 'FINAL_APPROACH', 'STOPPING'}

    # 초기 상태 IDLE, 입력·필터·제한 카운터 초기화.
    def __init__(self, settings=None):
        self.cfg = settings or Settings()
        self.state, self.reason = 'IDLE', 'waiting_for_start'
        self.odom = self.odom_time = None
        self.filtered_angular = 0.0
        self.observation = self.observation_time = None
        self.aligned_since = self.stopped_since = None
        self.started_at = self.final_at = None
        self.distance = self.final_distance = 0.0
        self.last_command = (0.0, 0.0)
        self.vision_recoveries = 0
        self.vision_wait_at = self.vision_good_since = self.recovery_stamp = None
        self.recovery_frames = 0
        self.last_steering_time = None
        self.odom_recoveries = 0
        self.odom_wait_at = self.odom_resume_state = self.odom_good_since = None
        self.odom_recovery_frames = 0
        self.recovery_motor_ready = True
        self.reset_final()

    # 직진 구간 상태 초기화: 정렬 때 보드 거리, 남은 거리, 막힘 감시, 펄스 이동 요청/결과.
    def reset_final(self):
        self.final_z = None              # 정렬 확정 순간 카메라~보드 거리
        self.encoder_remaining = None
        self.stall_since = None
        self.move_request = None         # {'pulses','speed','id','sent','sent_at','base','resumes','not_before','exit','reached_at'}
        self.move_state = self.move_reason = None

    # 정지: FAULT(오류) 또는 STOPPED(정상 중단)로 바꾸고 명령을 0으로.
    def halt(self, reason, fault=True):
        self.state = 'FAULT' if fault else 'STOPPED'
        self.reason = reason
        self.last_command = (0.0, 0.0)
        self.aligned_since = None

    # odom 입력 반영. 순간 점프·이동거리 누적·회전속도 필터 갱신.
    def set_odom(self, x, y, yaw, linear, angular, now):
        values = (x, y, yaw, linear, angular)
        if not all(math.isfinite(v) for v in values):
            self.halt('invalid_odometry')
            return
        if self.odom is not None and self.state in self.ACTIVE:
            step = math.hypot(x-self.odom[0], y-self.odom[1])
            if step > .10 or abs(wrap(yaw-self.odom[2])) > .35:
                self.halt('odometry_jump')
            else:
                self.distance += step
                if (self.state == 'FINAL_APPROACH' or
                        (self.state == 'ODOM_WAIT' and self.odom_resume_state == 'FINAL_APPROACH')):
                    self.final_distance += step
        if self.state == 'ODOM_WAIT':
            stationary = abs(linear) <= .01 and abs(angular) <= .04
            consecutive = (self.odom_time is not None and
                           0 < now-self.odom_time <= self.cfg.odom_recovery_fresh_s)
            if stationary and now > self.odom_wait_at:
                if not consecutive or self.odom_good_since is None:
                    self.odom_good_since, self.odom_recovery_frames = now, 1
                else:
                    self.odom_recovery_frames += 1
            else:
                self.odom_good_since, self.odom_recovery_frames = None, 0
        # 회전 속도는 odom 샘플마다 한 번 필터링. 끊긴 뒤 재개할 때 예전 값이 남지 않게 초기화.
        dt = None if self.odom_time is None else now-self.odom_time
        if dt is None or dt <= 0 or dt > self.cfg.odom_timeout_s:
            self.filtered_angular = angular
        else:
            weight = dt/(self.cfg.angular_rate_filter_s+dt)
            self.filtered_angular += weight*(angular-self.filtered_angular)
        self.odom, self.odom_time = values, now

    # 카메라 마커 관측 반영 (자세가 무효면 정렬 유지 시간 초기화).
    def set_observation(self, observation, now):
        self.observation, self.observation_time = observation, now
        if not observation.get('pose_valid', False):
            self.aligned_since = None
            self.vision_good_since = None
            self.recovery_frames = 0

    # STM32 move_status 반영 (지금 요청 번호와 같을 때만).
    def set_move_status(self, move_id, state, reason):
        req = self.move_request
        if req is None or move_id != req.get('id'):
            return
        if self.move_state == 'DONE':                 # 완료 뒤 늦게 반복되어 오는 상태(CANCELED 등)는 무시
            return
        if req.get('exit') and state == 'CANCELED':   # 도착 후 노드가 낸 토크 끄기 결과 → 실패 아님
            return
        self.move_state, self.move_reason = state, reason

    # 남은 거리(카메라~보드) = 정렬 때 보드 거리 − 직진 이동거리.
    def remaining_distance(self):
        return None if self.final_z is None else self.final_z - self.final_distance

    # 시각 stamp 가 timeout 이내인지.
    @staticmethod
    def fresh(stamp, now, timeout):
        return stamp is not None and 0 <= now-stamp <= timeout

    # odom·카메라 입력 중 오래된 것이 있으면 그 이유를 돌려준다.
    def input_error(self, now, vision=True):
        c = self.cfg
        if not self.fresh(self.odom_time, now, c.odom_timeout_s):
            return 'odometry_timeout'
        if vision and not self.fresh(self.observation_time, now, c.observation_timeout_s):
            return 'vision_timeout'
        return None

    # 마커 관측을 로봇 기준 (보드 앞방향 거리 bx, 좌우 오차 by, 각도 오차, 광학 거리 z)로 변환.
    def pose(self):
        o, c = self.observation, self.cfg
        if not o or not o.get('pose_valid'):
            return None
        try:
            x_right, _, z = o['tvec_m']
            phi = o['normal_yaw_rad']
            if not all(math.isfinite(v) for v in (x_right, z, phi)) or z <= 0:
                return None
            a = math.radians(c.camera_yaw_deg)
            bx = math.cos(a)*z + math.sin(a)*x_right + c.camera_forward_m
            by = math.sin(a)*z - math.cos(a)*x_right + c.camera_left_m
            return bx, by, wrap(a-phi), z
        except (TypeError, ValueError, KeyError):
            return None

    # 시작 조건 검사: 입력 최신, 정지 상태, 마커 4개 보임, 시작 거리 범위.
    def start_error(self, now):
        if self.state in self.ACTIVE:
            return 'already_running'
        error = self.input_error(now)
        if error:
            return error
        pose = self.pose()
        if pose is None:
            return 'four_markers_required'
        if not self.cfg.min_visual_distance_m < pose[3] <= self.cfg.max_start_distance_m:
            return 'start_distance_out_of_range'
        if abs(self.odom[3]) > .01 or abs(self.odom[4]) > .04:
            return 'robot_must_be_stationary'
        return None

    # 조건이 맞으면 ALIGN 상태로 시작.
    def start(self, now):
        error = self.start_error(now)
        if error:
            return False, error
        self.entry_heading_ready = False
        self.state, self.reason = 'ALIGN', 'visual_alignment'
        self.started_at, self.final_at = now, None
        self.distance = self.final_distance = 0.0
        self.aligned_since = self.stopped_since = None
        self.vision_recoveries = 0
        self.vision_wait_at = self.vision_good_since = self.recovery_stamp = None
        self.recovery_frames = 0
        self.odom_recoveries = 0
        self.odom_wait_at = self.odom_resume_state = self.odom_good_since = None
        self.odom_recovery_frames = 0
        self.reset_final()
        return True, self.state

    # odom 이 끊기면 멈추고 ODOM_WAIT 로 (복구 횟수 제한).
    def wait_for_odom(self, now):
        if self.odom_recoveries >= self.cfg.max_odom_recoveries:
            self.halt('odometry_recovery_limit')
            return
        self.odom_resume_state = self.state
        self.state, self.reason = 'ODOM_WAIT', 'odometry_timeout_waiting'
        self.odom_wait_at = now
        self.odom_recoveries += 1
        self.odom_good_since = self.aligned_since = self.stopped_since = None
        self.odom_recovery_frames = 0

    # ODOM_WAIT 중: 전체 시간/거리 제한 확인, 최신 정지 odom 이 이어지면 원래 단계로 복귀.
    def odom_wait_tick(self, now):
        c, phase = self.cfg, self.odom_resume_state
        if now-self.started_at >= c.max_total_time_s or self.distance >= c.max_total_distance_m:
            self.halt('total_approach_limit')
        elif phase == 'FINAL_APPROACH' and (now-self.final_at >= c.max_final_time_s
                                           or self.final_distance >= c.max_final_distance_m):
            self.halt('final_approach_limit')
        elif phase == 'STOPPING' and now-self.final_at > c.max_final_time_s+c.stopped_hold_s+2:
            self.halt('stop_not_confirmed')
        elif now-self.odom_wait_at >= c.odom_recovery_timeout_s:
            self.halt('odometry_recovery_timeout')
        else:
            fresh = self.fresh(self.odom_time, now, c.odom_recovery_fresh_s)
            if not fresh:
                self.odom_good_since, self.odom_recovery_frames = None, 0
            ready = (fresh and self.odom_recovery_frames >= 3
                     and self.odom_good_since is not None
                     and now-self.odom_good_since >= c.odom_recovery_hold_s
                     and self.recovery_motor_ready)
            if phase != 'STOPPING':
                ready = ready and self.fresh(self.observation_time, now, c.vision_recovery_fresh_s)
            if phase in ('ALIGN', 'VISION_WAIT'):
                p = self.pose()
                ready = (ready and self.observation_time > self.odom_wait_at and p is not None
                         and c.min_visual_distance_m < p[3] <= c.max_start_distance_m)
            if ready:
                self.state = 'ALIGN' if phase == 'VISION_WAIT' else phase
                self.reason = 'odometry_recovered'
                self.aligned_since = self.stopped_since = None
                self.last_steering_time = None
        return (0.0, 0.0)

    # 근거리 회전 한도. near_angular_taper 면 정렬 지점에서 최소 시야 거리까지 가까워질수록
    # near_angular_rps × (1 → near_angular_min_scale) 로 선형으로 줄인다. 정렬 지점보다 멀면 그대로.
    def near_limit(self, optical_z):
        c = self.cfg
        if not c.near_angular_taper:
            return c.near_angular_rps
        span = max(1e-6, c.staging_distance_m - c.min_visual_distance_m)
        s = max(0.0, min(1.0, (optical_z - c.min_visual_distance_m) / span))
        return c.near_angular_rps * (c.near_angular_min_scale + (1.0 - c.near_angular_min_scale) * s)

    # 회전 명령 계산 (P + 실제 회전 속도 감쇠 D, 최소 회전 속도 처리).
    def turn(self, value, settling=False):
        value -= self.cfg.angular_kd*self.filtered_angular
        # 이동 중 아주 작은 회전은 버린다 (최소 속도로 좌우 번갈아 도는 것 방지). 제자리 정렬 때는 최소 속도로 보정.
        if value == 0 or (not settling and abs(value) < self.cfg.min_angular_rps):
            return 0.0
        return math.copysign(max(self.cfg.min_angular_rps,
                                 min(abs(value), self.cfg.max_angular_rps)), value)

    # 주기마다 명령 계산 후 회전 비율·가속 제한을 적용해 최종 (v, w) 반환.
    def tick(self, now):
        previous = self.last_command
        command = self._tick(now)
        command = (command[0], command[1] * self.cfg.angular_speed_scale)
        # 정렬 중 회전만 가속 제한. 정지와 직진은 바로 적용.
        if self.state == 'ALIGN' and command != (0.0, 0.0):
            dt = .02 if self.last_steering_time is None else max(.001, min(.1, now-self.last_steering_time))
            step = self.cfg.steering_accel_rps2 * self.cfg.angular_speed_scale * dt
            command = (command[0], previous[1]+clamp(command[1]-previous[1], step))
        self.last_steering_time = now
        self.last_command = command
        return command

    # 직진 구간 정지 시작: STOPPING 으로 넘어가 정지 확인을 기다린다.
    def begin_stop(self):
        self.state, self.reason = 'STOPPING', 'encoder_distance'
        self.stopped_since = None
        return (0.0, 0.0)

    # 펄스 이동(move) 진행 판단. 결과에 따라 정지/재전송/실패. 진행 중이면 None.
    def move_tick(self, now):
        c, req = self.cfg, self.move_request
        if self.move_state == 'ERROR' and self.move_reason == 'feedback' and req.get('resumes', 0) < 2:
            # STM32 가 이동 중 XL430 위치 읽기에 한 번 실패 → 실패로 끝내지 않고 남은 거리만 다시 보낸다 (최대 2번).
            # ERROR 때 STM32 가 토크를 끄므로 노드가 토크를 켜고, 토크 확인 뒤 새 move 를 보낸다.
            remaining = req['pulses'] * MOVE_M_PER_PULSE - (self.final_distance - req.get('base', 0.0))
            if remaining <= .002:
                self.move_state, self.move_reason = 'DONE', 'feedback_error_but_reached'
            else:
                self.move_request = {'pulses': int(round(remaining / MOVE_M_PER_PULSE)), 'speed': req['speed'],
                                     'id': None, 'sent': False, 'base': self.final_distance,
                                     'resumes': req.get('resumes', 0) + 1, 'not_before': now + 0.6}
                self.move_state = self.move_reason = None
                self.reason = f'move_resume_after_feedback_{self.move_request["resumes"]}'
                return (0.0, 0.0)
        if self.move_state in ('ERROR', 'CANCELED'):
            self.halt(f'move_{self.move_state.lower()}:{self.move_reason}')
            return (0.0, 0.0)
        if self.move_state == 'DONE':
            self.encoder_remaining = self.remaining_distance()
            return self.begin_stop()
        # 바닥에서는 목표에 도착해도 STM32 가 DONE 을 안 보낼 때가 있다 (부하로 몇 펄스 모자람).
        # 목표 2mm 안에 멈춘 채 2초가 지나면 노드에 위치 모드 종료(토크 끄기/켜기)를 요청 → 노드가 DONE 을 넣는다.
        target = req.get('base', 0.0) + req['pulses'] * MOVE_M_PER_PULSE
        if req['sent'] and self.final_distance >= target - .002 and abs(self.odom[3]) < .005:
            if req.get('reached_at') is None:
                req['reached_at'] = now
            elif now-req['reached_at'] >= 2.0:
                req['exit'] = True
        else:
            req['reached_at'] = None
        limit = c.move_timeout_s + (10.0 if req.get('exit') else 0.0) + 8.0 * req.get('resumes', 0)
        if now-self.final_at >= limit:
            self.halt('move_timeout')
            return (0.0, 0.0)
        return None

    # 막힘 보호: 직진 중 바퀴가 1.5초 넘게 거의 안 움직이면(보드에 닿음 등) 실패. True 면 정지함.
    def stalled(self, now):
        req = self.move_request
        if req is not None:
            reached = self.final_distance >= req.get('base', 0.0) + req['pulses'] * MOVE_M_PER_PULSE - .002
            waiting = not req['sent'] or now-req.get('sent_at', now) < 1.5   # 위치 모드 전환·재전송 대기
            if reached or waiting:
                self.stall_since = None
                return False
        if now-self.final_at <= 1.5 or abs(self.odom[3]) >= .005:
            self.stall_since = None
            return False
        if self.stall_since is None:
            self.stall_since = now
            return False
        if now-self.stall_since >= 1.5:
            self.halt('final_approach_stalled')
            return True
        return False

    # 직진 구간 한 주기: 한도 → 펄스 이동 결과 → 막힘 → (속도 방식) 남은 거리로 감속·정지.
    def final_tick(self, now):
        c = self.cfg
        if now-self.final_at >= c.max_final_time_s or self.final_distance >= c.max_final_distance_m:
            self.halt('final_approach_limit')
            return (0.0, 0.0)
        if self.move_request is not None:
            result = self.move_tick(now)
            if result is not None:
                return result
        if self.stalled(now):
            return (0.0, 0.0)
        if self.move_request is not None:
            return (0.0, 0.0)          # 이동은 STM32 가 함. 속도 명령은 0
        self.encoder_remaining = self.remaining_distance()
        slow = c.encoder_slow_zone_m > 0 and self.encoder_remaining <= c.encoder_stop_distance_m + c.encoder_slow_zone_m
        lead = c.encoder_slow_lead_m if slow else c.encoder_stop_lead_m
        if self.encoder_remaining <= c.encoder_stop_distance_m + lead:
            return self.begin_stop()
        return (c.encoder_slow_speed_mps if slow else c.final_speed_mps), 0.0

    # 정렬 확정: 보드 거리를 기억하고 직진 시작. 펄스 이동이면 이동 펄스 요청을 만든다.
    def begin_final(self, now, optical_z):
        c = self.cfg
        self.state, self.reason = 'FINAL_APPROACH', 'alignment_confirmed'
        self.final_at, self.final_distance = now, 0.0
        self.reset_final()
        self.final_z = optical_z
        if c.move_final_enabled:
            distance = optical_z - c.encoder_stop_distance_m
            if not 0.01 <= distance <= c.max_final_distance_m:
                self.halt('move_distance_out_of_range')
                return
            self.move_request = {'pulses': int(round(distance / MOVE_M_PER_PULSE)), 'speed': c.move_speed_raw,
                                 'id': None, 'sent': False, 'base': 0.0, 'resumes': 0}

    # 상태별 실제 계산: 정렬(거리·좌우·각도 맞추기), 직진, 시간·거리 제한, 시야 끊김 처리.
    def _tick(self, now):
        zero = (0.0, 0.0)
        c = self.cfg
        if self.state not in self.ACTIVE:
            return zero
        if self.state == 'ODOM_WAIT':
            return self.odom_wait_tick(now)
        error = self.input_error(now, vision=self.state != 'STOPPING')
        if error:
            if error == 'odometry_timeout' and c.odom_recovery_enabled:
                self.wait_for_odom(now)
                return zero
            elif error == 'vision_timeout' and self.state == 'ALIGN':
                if self.vision_recoveries >= c.max_vision_recoveries:
                    self.halt('vision_recovery_limit')
                    return zero
                self.state, self.reason = 'VISION_WAIT', 'vision_timeout_waiting'
                self.vision_recoveries += 1
                self.vision_wait_at = now
                self.vision_good_since = self.recovery_stamp = self.aligned_since = None
                self.recovery_frames = 0
            elif not (error == 'vision_timeout' and self.state == 'VISION_WAIT'):
                self.halt(error)
                return zero
        if self.state == 'STOPPING':
            if abs(self.odom[3]) < .01 and abs(self.odom[4]) < .04:
                if self.stopped_since is None:
                    self.stopped_since = now
                if now-self.stopped_since >= c.stopped_hold_s:
                    self.state, self.reason = 'DOCKED', 'encoder_distance_and_stationary'
            else:
                self.stopped_since = None
            if now-self.final_at > c.max_final_time_s+c.stopped_hold_s+2:
                self.halt('stop_not_confirmed')
            return zero
        if now-self.started_at >= c.max_total_time_s or self.distance >= c.max_total_distance_m:
            self.halt('total_approach_limit')
            return zero
        if self.state == 'VISION_WAIT':
            # 0 속도 유지. 전체 시간·거리 예산은 그대로.
            if now-self.vision_wait_at >= c.vision_recovery_timeout_s:
                self.halt('vision_recovery_timeout')
                return zero
            p = self.pose()
            healthy = (self.fresh(self.observation_time, now, c.vision_recovery_fresh_s)
                       and self.observation_time > self.vision_wait_at
                       and p is not None
                       and c.min_visual_distance_m < p[3] <= c.max_start_distance_m
                       and abs(self.odom[3]) <= .01 and abs(self.odom[4]) <= .04)
            if not healthy:
                self.vision_good_since = None
                self.recovery_frames = 0
            else:
                if self.vision_good_since is None:
                    self.vision_good_since = now
                if self.observation_time != self.recovery_stamp:
                    self.recovery_frames += 1
                    self.recovery_stamp = self.observation_time
                if (self.recovery_frames >= 3
                        and now-self.vision_good_since >= c.vision_recovery_hold_s):
                    self.state, self.reason = 'ALIGN', 'vision_recovered'
                    self.aligned_since = None
            return zero
        if self.state == 'FINAL_APPROACH':
            return self.final_tick(now)
        return self.align_tick(now)

    # 정렬 단계: 보드 법선 위 정렬 지점을 향한 전진 전용 극좌표 제어, 가까우면 제자리 회전·조금씩 전진.
    def align_tick(self, now):
        zero = (0.0, 0.0)
        c = self.cfg
        p = self.pose()
        if p is None:
            self.aligned_since = None
            self.reason = 'waiting_for_four_markers'
            return zero
        bx, by, theta, optical_z = p
        if optical_z < c.min_visual_distance_m:
            self.halt('too_close_before_alignment')
            return zero
        aligned = abs(by) <= c.lateral_tolerance_m and abs(theta) <= math.radians(c.yaw_tolerance_deg)
        stage = c.staging_distance_m + c.camera_forward_m
        if aligned and optical_z <= c.staging_distance_m+.02:
            if abs(self.odom[3]) > .01 or abs(self.odom[4]) > .04:
                self.aligned_since = None
            elif self.aligned_since is None:
                self.aligned_since = now
            elif now-self.aligned_since >= c.aligned_hold_s:
                self.begin_final(now, optical_z)
            return zero
        self.aligned_since = None
        self.reason = 'visual_alignment'
        if aligned:
            return c.align_speed_mps, 0.0
        # Preserve visible space before alignment is latched. Include the worst
        # accepted camera age, command/braking latency and a small coast margin.
        # Rotation can fix heading only when already on the board normal;
        # a lateral offset needs another approach, never deeper forward creep.
        if c.alignment_min_distance_m:
            reserve = c.align_speed_mps * (c.observation_timeout_s + .2) + .005
            if optical_z <= c.alignment_min_distance_m + reserve:
                cross_track = by*math.cos(theta)-bx*math.sin(theta)
                if abs(cross_track) > c.lateral_tolerance_m:
                    self.halt('alignment_clearance_exhausted')
                    return zero
                self.reason = 'alignment_clearance_heading_only'
                return 0.0, clamp(self.turn(1.2*theta, settling=True), self.near_limit(optical_z))
        gx, gy = bx-stage*math.cos(theta), by-stage*math.sin(theta)
        # Use the visual goal bearing at the waypoint, not the map heading.
        # Hysteresis avoids alternating move/rotate commands near the boundary.
        if c.visual_entry_alignment:
            entry_angle = math.atan2(gy, gx)
            if abs(entry_angle) > math.radians(20):
                self.entry_heading_ready = False
            if not self.entry_heading_ready and gx > .02:
                if abs(entry_angle) <= math.radians(6) and abs(self.odom[4]) <= .04:
                    self.entry_heading_ready = True
                else:
                    self.reason = 'visual_entry_heading_alignment'
                    return 0.0, clamp(self.turn(1.5*entry_angle, settling=True), c.max_angular_rps)
        rho = math.hypot(gx, gy)
        # 정렬 지점 근처에선 방위각이 불안정 → 법선까지 거리(cross track)가 맞으면 제자리 방향만 맞춘다.
        cross_track = by*math.cos(theta)-bx*math.sin(theta)
        # After passing the staging point, rotation can still finish alignment
        # when already on the board normal. Do not consume more visual clearance.
        heading_only = rho < .02 or (gx <= .02 and abs(theta) < math.radians(45))
        if heading_only and abs(cross_track) <= c.lateral_tolerance_m:
            self.reason = 'near_stage_heading_alignment'
            return 0.0, clamp(self.turn(1.2*theta, settling=True), self.near_limit(optical_z))
        # 전진만 하는 로봇은 옆·뒤의 정렬 지점을 쫓을 수 없다 → 법선 위 목표를 앞으로 당기되 시야 여유는 남긴다.
        if gx <= .02 and abs(theta) < math.radians(45):
            local_stage = max(c.min_visual_distance_m+.025, min(stage, bx-.04))
            gx, gy = bx-local_stage*math.cos(theta), by-local_stage*math.sin(theta)
            if gx <= .003:
                self.halt('insufficient_visual_alignment_space')
                return zero
            rho = math.hypot(gx, gy)
            self.reason = 'near_stage_forward_alignment'
        if rho < .004:
            return 0.0, clamp(self.turn(1.2*theta, settling=True), self.near_limit(optical_z))
        alpha = math.atan2(gy, gx)
        beta = wrap(theta-alpha)
        blend = max(0., min(1., (optical_z-c.staging_distance_m)/.25))
        near = self.near_limit(optical_z)
        limit = min(c.max_angular_rps, near + blend*(c.max_angular_rps-near))
        # 먼 거리는 회전을 키우고(far_steering_gain) 정렬 지점에 가까울수록 줄인다.
        angular = clamp(self.turn((1.+(c.far_steering_gain-1.)*blend)*(1.8*alpha-.7*beta)), limit)
        if c.visual_entry_alignment:
            raw_turn = (1.+(c.far_steering_gain-1.)*blend)*(1.8*alpha-.7*beta)
            angular = clamp(raw_turn-c.angular_kd*self.filtered_angular, limit)
        linear = min(c.align_speed_mps, .4*rho)
        if c.visual_entry_alignment:
            # Trade translation for steering time while bearing/pose is wrong.
            severity = max(1., abs(alpha)/math.radians(10),
                           abs(theta)/math.radians(4), abs(cross_track)/.008)
            linear = min(linear, max(.01, c.align_speed_mps/min(severity, 4.)))
        heading_limit = 15 if self.reason == 'near_stage_forward_alignment' else 50
        if abs(alpha) > math.radians(heading_limit):
            linear = 0.0
        elif linear > 0:
            linear = max(.01, linear*max(0.0, math.cos(alpha)))   # OpenCR 속도 단위 0.01m/s
        return linear, angular
