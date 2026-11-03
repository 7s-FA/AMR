"""출차 표시 진행 단계 기록: 재시도 때 남은 출차만 하도록."""
import pytest
from test_directional_waypoints import FakeNavigator, route

POINTS = [dict(x=0, y=0, yaw=0, mode='forward'), dict(x=.3, y=0, yaw=0, mode='forward')]


def run(nav, flag, consumed, distance=.158, turn=180):
    return route.run_waypoints(nav, POINTS, 30, route.Path(route.__file__).parents[1] / 'behavior_trees',
                               pre_backup_distance=distance, pre_backup_speed=.045,
                               pre_backup_open_loop=True, pre_turn_angle_deg=turn,
                               continuous_intermediate=True,
                               departure_flag=str(flag), departure_consumed=str(consumed))


@pytest.fixture
def flag(tmp_path):
    f = tmp_path / 'departure_pending'
    f.write_text('successful_dock 2026-10-08T10:00:00\n')
    return f


def test_full_departure_success_consumes_flag(tmp_path, flag):
    nav = FakeNavigator(); calls = []
    nav.pre_backup_timed = lambda d, s, dl: calls.append(d) or d
    assert run(nav, flag, tmp_path / 'consumed') == 0
    assert calls == [pytest.approx(.158)] and len(nav.spin_requests) == 1
    assert not flag.exists() and (tmp_path / 'consumed').exists()


def test_backup_not_moving_keeps_flag_and_does_not_turn(tmp_path, flag):
    nav = FakeNavigator()
    nav.pre_backup_timed = lambda d, s, dl: 0.0
    with pytest.raises(RuntimeError, match='후진 거리 부족'):
        run(nav, flag, tmp_path / 'consumed')
    assert nav.spin_requests == [] and nav.goals == []
    assert flag.read_text().startswith('successful_dock')   # 다음 명령: 전체 출차 다시


def test_partial_backup_records_remaining(tmp_path, flag):
    nav = FakeNavigator()
    nav.pre_backup_timed = lambda d, s, dl: 0.05
    with pytest.raises(RuntimeError):
        run(nav, flag, tmp_path / 'consumed')
    text = flag.read_text().split()
    assert text[0] == 'departure_backup_partial' and float(text[1]) == pytest.approx(.108, abs=1e-3)
    assert nav.spin_requests == []


def test_turn_failure_after_backup_records_backup_done(tmp_path, flag):
    nav = FakeNavigator()
    nav.pre_backup_timed = lambda d, s, dl: d
    nav.departure_spin = lambda angle, allowance: False
    with pytest.raises(RuntimeError):
        run(nav, flag, tmp_path / 'consumed')
    assert flag.read_text().startswith('departure_backup_done')   # 다음 명령: 회전만


def test_turn_only_retry_consumes_flag(tmp_path, flag):
    flag.write_text('departure_backup_done 2026-10-08T10:45:00\n')
    nav = FakeNavigator(); calls = []
    nav.pre_backup_timed = lambda d, s, dl: calls.append(d) or d
    assert run(nav, flag, tmp_path / 'consumed', distance=0.0) == 0
    assert calls == [] and len(nav.spin_requests) == 1 and not flag.exists()


def test_navigation_failure_after_departure_does_not_repeat_departure(tmp_path, flag):
    nav = FakeNavigator(results=[route.TaskResult.FAILED] * 10)
    nav.pre_backup_timed = lambda d, s, dl: d
    assert run(nav, flag, tmp_path / 'consumed') != 0
    assert not flag.exists()   # 이미 통로로 나왔으니 다음 명령은 출차하지 않음


def test_timed_fallback_without_encoder_still_proceeds(tmp_path, flag):
    nav = FakeNavigator()
    nav.pre_backup_timed = lambda d, s, dl: None   # 시작 엔코더 없음: 기존 시간제어
    assert run(nav, flag, tmp_path / 'consumed') == 0 and not flag.exists()


def test_no_flag_path_keeps_old_behaviour():
    nav = FakeNavigator(); nav.pre_backup_timed = lambda d, s, dl: 0.0
    with pytest.raises(RuntimeError):
        route.run_waypoints(nav, POINTS, 30, route.Path(route.__file__).parents[1] / 'behavior_trees',
                            pre_backup_distance=.158, pre_backup_speed=.045, pre_backup_open_loop=True,
                            pre_turn_angle_deg=180, continuous_intermediate=True)
