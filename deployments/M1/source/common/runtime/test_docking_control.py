# ========================================================================
# 역할: docking_control 상태 기계 시험 (ROS 없이 모의 로봇으로). 2026-10-16 IR 제거 후 엔코더 거리 정지 기준.
#   - 속도 방식(버거2): 목표 거리 근처에서 감속·정지, 막힘 보호
#   - 펄스 이동(버거3): DONE 으로 정지, DONE 없이 도착하면 위치 모드 종료 요청, feedback 오류 뒤 남은 거리 재전송
#   - 설정 검사, 근거리 회전 한도 감소
# 실행: python3 -m pytest -q test_docking_control.py
# ========================================================================
import math
import pytest

from docking_control import DockingControl, Settings, MOVE_M_PER_PULSE

BOARD = {'pose_valid': True, 'tvec_m': [0.0, 0.0, 0.26], 'normal_yaw_rad': 0.0}


def base_settings(**kw):
    values = dict(staging_distance_m=0.25, final_speed_mps=0.035, align_speed_mps=0.045,
                  max_final_distance_m=0.45, max_final_time_s=30.0, odom_recovery_enabled=True,
                  odom_recovery_timeout_s=4.0, encoder_stop_distance_m=0.008)
    values.update(kw)
    return Settings(**values)


# 정렬된 보드를 보여 주고 시작 → 정렬 확정까지 진행한 상태기계를 돌려준다.
def aligned(settings):
    d = DockingControl(settings)
    t = 0.0
    d.set_odom(0, 0, 0, 0, 0, t)
    d.set_observation(BOARD, t)
    assert d.start(t)[0]
    while d.state == 'ALIGN' and t < 3:
        t += .01
        d.tick(t)
        d.set_odom(0, 0, 0, 0, 0, t)
        d.set_observation(BOARD, t)
    assert d.state == 'FINAL_APPROACH'
    return d, t


# 속도 방식 모의 주행: 명령 지연 delay 초, odom 28Hz, 정지 명령 뒤에도 delay 동안 계속 감.
def drive_velocity(d, t, delay=0.06, block_at=None):
    x, v, pending = 0.0, 0.0, []
    while d.state in d.ACTIVE and t < 40:
        t += .005
        pending.append((t + delay, d.tick(t)[0]))
        while pending and pending[0][0] <= t:
            v = pending.pop(0)[1]
        if block_at is not None and x >= block_at:
            v = 0.0
        x += v * .005
        d.set_odom(x, 0, 0, v, 0, t)
        d.set_observation({'pose_valid': False}, t)
    return x, t


def test_velocity_stop_with_slow_zone_is_precise():
    for delay in (0.03, 0.06, 0.09):
        d, t = aligned(base_settings(encoder_stop_lead_m=0.004, encoder_slow_zone_m=0.02,
                                     encoder_slow_speed_mps=0.01, encoder_slow_lead_m=0.001))
        x, _ = drive_velocity(d, t, delay)
        assert d.state == 'DOCKED' and d.reason == 'encoder_distance_and_stationary'
        assert abs((0.26 - x) - 0.008) < 0.0015


def test_velocity_stall_faults_quickly():
    d, t = aligned(base_settings())
    _, end = drive_velocity(d, t, block_at=0.15)
    assert d.state == 'FAULT' and d.reason == 'final_approach_stalled'
    assert end - t < 10


# 펄스 이동 모의: 노드 역할(전송·결과)을 흉내. errors: 각 요청 시작 뒤 이 거리에서 feedback 오류.
def drive_move(d, t, errors=(), done=True):
    x, nid, cur, errors = 0.0, 0, None, list(errors)
    while d.state in d.ACTIVE and t < 60:
        t += .01
        assert d.tick(t) == (0.0, 0.0)            # 펄스 이동 중 속도 명령은 항상 0
        req, v = d.move_request, 0.0
        if req and not req['sent'] and t >= req.get('not_before', 0):
            nid += 1
            req.update(sent=True, sent_at=t, id=nid)
            cur = (req, x, t)
        if cur and cur[0] is req:
            goal = cur[1] + req['pulses'] * MOVE_M_PER_PULSE
            if errors and x - cur[1] >= errors[0]:
                d.set_move_status(req['id'], 'ERROR', 'feedback')
                errors.pop(0)
                cur = None
            else:
                v = 0.035 if (t - cur[2] > .6 and x < goal - 1e-4) else 0.0
                if x >= goal - 1e-4 and t - cur[2] > 1:
                    if done:
                        d.set_move_status(req['id'], 'DONE', 'none')
                    elif req.get('exit') and d.move_state is None:
                        d.move_state = 'DONE'          # 노드가 토크 끄기/켜기 뒤 넣는 완료
        x += v * .01
        d.set_odom(x, 0, 0, v, 0, t)
        d.set_observation({'pose_valid': False}, t)
    return x


def move_settings():
    return base_settings(move_final_enabled=True)


def test_move_done_docks_at_target():
    d, t = aligned(move_settings())
    assert d.move_request['pulses'] == round((0.26 - 0.008) / MOVE_M_PER_PULSE)
    x = drive_move(d, t)
    assert d.state == 'DOCKED' and abs((0.26 - x) - 0.008) < 0.0005


def test_move_without_done_requests_exit():
    d, t = aligned(move_settings())
    drive_move(d, t, done=False)
    assert d.state == 'DOCKED'


def test_move_feedback_error_resumes_remaining_distance():
    d, t = aligned(move_settings())
    x = drive_move(d, t, errors=[0.05, 0.05])
    assert d.state == 'DOCKED' and abs((0.26 - x) - 0.008) < 0.0005


def test_move_third_feedback_error_faults():
    d, t = aligned(move_settings())
    drive_move(d, t, errors=[0.05, 0.05, 0.05])
    assert d.state == 'FAULT' and d.reason == 'move_error:feedback'


def test_late_canceled_after_exit_is_ignored():
    d, t = aligned(move_settings())
    d.move_request.update(sent=True, id=7, exit=True)
    d.set_move_status(7, 'CANCELED', 'motor_off')
    assert d.move_state is None


def test_near_angular_taper():
    d = DockingControl(base_settings(near_angular_rps=0.12, near_angular_taper=True, staging_distance_m=0.20))
    assert d.near_limit(0.30) == pytest.approx(0.12)
    assert d.near_limit(0.12) == pytest.approx(0.048)
    assert d.near_limit(0.16) == pytest.approx(0.084)


def test_invalid_settings_rejected():
    with pytest.raises(ValueError):
        base_settings(encoder_stop_distance_m=0.0)
    with pytest.raises(ValueError):
        base_settings(move_speed_raw=10)
    with pytest.raises(ValueError):
        base_settings(encoder_slow_zone_m=0.02, encoder_slow_speed_mps=0.05)
