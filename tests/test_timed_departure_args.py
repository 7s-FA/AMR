"""Exercise the actual shell argument builder without starting robot services."""
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('robot', ['M1', 'M2'])
@pytest.mark.parametrize('pending', [False, True])
def test_departure_uses_three_point_five_seconds(robot, pending, tmp_path):
    source = (ROOT/robot/'src/navigation/run_selected_waypoints.sh').read_text()
    function = source.split('add_departure_args() {', 1)[1].split('\n}\n', 1)[0]
    flag = tmp_path/'departure_pending'
    if pending:
        flag.touch()
    script = '''set -e
HERE=$1
ROOT=$2
ROBOT=burger1
DEPARTURE_FLAG=$3
WAYPOINT_ARGS=()
add_departure_args() {''' + function + '''
}
add_departure_args >/dev/null
printf '%s\\n' "${WAYPOINT_ARGS[@]}"
'''
    result = subprocess.run(['bash', '-c', script, 'test', str(ROOT/'common/navigation'),
                             str(tmp_path), str(flag)], check=True, capture_output=True, text=True)
    args = result.stdout.split()
    if not pending:
        assert args == []
        return
    assert '--pre-backup-open-loop' in args
    distance = float(args[args.index('--pre-backup-distance')+1])
    speed = float(args[args.index('--pre-backup-speed')+1])
    assert distance/speed == pytest.approx(3.5)
    assert args[args.index('--pre-turn-angle-deg')+1] == '180'
