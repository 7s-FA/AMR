#!/usr/bin/env python3
# ========================================================================
# 역할: 이동 임무의 유일한 접수 창구. 관제 명령(host)과 로봇 터미널 개별 시험(manual)을 같은 규칙으로 받는다.
#       모드(individual/process), 중복 명령, 이동 중 여부를 검사하고 systemd 로 <로봇>-mission 서비스를 띄운다.
# 실행: backend.py 가 import / 터미널에서 python3 operation.py submit|mode|status|stop|ready|parked / mission.sh·robot.py 가 호출.
# 호출 관계: mission_entry.sh(임무 본체 시작), manage.sh stop(정지), manage.sh ready(준비), confirm_parked.sh(주차 확인).
# 저장 파일: data/<로봇>/mission_state.json(현재 임무), commands/<id>.json(명령별 기록), operation_mode.json(모드).
# ========================================================================
"""One mission admission point for individual tests and host process commands."""
import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

STATIONS = {'mat': 'WAREHOUSE', 'asm': 'ASSEMBLY', 'rest': 'WAITING', 'park': 'HOME'}
FINISHED = {'success', 'failed', 'cancelled'}


# JSON 파일을 임시 파일에 쓴 뒤 교체 (쓰는 도중 읽어도 깨진 파일이 보이지 않게).
def atomic_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    temp.replace(path)


# 명령 ID 형식 검사 (영문/숫자/_.- 1~96자).
def command_id(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', value):
        raise ValueError('command-id는 영문/숫자/밑줄/점/하이픈 1~96자여야 합니다.')
    return value


# 한 로봇의 임무 상태·모드·잠금을 다루는 클래스.
class Operation:
    # navigation 폴더 이름으로 로봇(burger1/2)을 정하고 data 폴더 경로를 잡는다.
    def __init__(self, here, run=subprocess.run):
        self.here = Path(here).absolute()
        self.robot = self.here.parent.name
        if self.robot not in ('burger1', 'burger2'):
            raise ValueError('로봇 navigation 폴더에서 실행해야 합니다.')
        self.data = self.here.parents[2] / 'data' / self.robot
        self.run = run

    # data/<로봇>/operation.lock 파일 잠금. 여러 프로세스가 동시에 상태를 바꾸지 못하게 한다.
    @contextmanager
    def lock(self):
        self.data.mkdir(parents=True, exist_ok=True)
        with (self.data / 'operation.lock').open('a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            yield

    # data 폴더의 JSON 파일 읽기 (없으면 기본값).
    def read(self, name, default=None):
        path = self.data / name
        return json.loads(path.read_text()) if path.exists() else default

    # 현재 운용 모드 (individual=로봇 터미널 시험, process=관제 공정).
    def mode(self):
        return self.read('operation_mode.json', {'mode': 'individual'})['mode']

    # systemd 서비스 <로봇>-<unit> 이 실행 중인지 확인.
    def active(self, unit):
        r = self.run(['systemctl', '--user', 'show', f'{self.robot}-{unit}.service', '-p', 'ActiveState', '--value'], capture_output=True, text=True)
        state = r.stdout.strip()
        if state not in ('active', 'activating', 'deactivating', 'reloading', 'inactive', 'failed'):
            raise RuntimeError('서비스 상태를 확인할 수 없습니다: ' + unit)
        return state not in ('inactive', 'failed')

    # 이동 중인지 판단: 임무/도킹/rest 서비스, 이동·준비 잠금 파일, 주행·도킹 프로세스를 모두 본다.
    def busy(self, include_preparation=True):
        if any(self.active(u) for u in ('mission', 'docking', 'rest')):
            return True
        runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
        kinds = ('mission', 'motion', 'prepare') if include_preparation else ('mission', 'motion')
        for kind in kinds:
            with (runtime / f'{self.robot}-{kind}-{os.getuid()}.lock').open('a') as f:
                try:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return True
        r = self.run(['pgrep', '-u', str(os.getuid()), '-f', '[/]nav2_waypoints|[d]ocking_node.py|[r]est_ir.py'], capture_output=True, text=True)
        if r.returncode not in (0, 1):
            raise RuntimeError('주행 프로세스를 확인할 수 없습니다.')
        return r.returncode == 0

    # 명령별 기록과 현재 임무 파일을 함께 저장.
    def save_task(self, task):
        atomic_json(self.data / 'commands' / (command_id(task['command_id']) + '.json'), task)
        atomic_json(self.data / 'mission_state.json', task)

    # 현재 임무를 돌려준다. 기록 없이 서비스가 끝났으면 '중단(failed)'으로 고쳐 저장 (자동 재실행 없음).
    def current(self):
        task = self.read('mission_state.json')
        if task and task.get('command_id') and task.get('status') not in FINISHED and not self.active('mission'):
            task.update(status='failed', stage='interrupted', error='완료 기록 없이 이동 서비스가 종료되었습니다. 자동 재실행하지 않습니다.', updated_unix=time.time())
            self.save_task(task)
        return task

    # 모드 변경 (이동 중에는 거부).
    def set_mode(self, mode):
        if self.busy():
            raise RuntimeError('이동 중에는 모드를 변경할 수 없습니다. stop 후 완료를 확인하세요.')
        self.current()
        atomic_json(self.data / 'operation_mode.json', {'mode': mode, 'updated_unix': time.time()})
        return {'robot': self.robot, 'mode': mode, 'motion_sent': False}

    # 목적지(mat/asm/rest/park) 임무 접수 → systemd-run 으로 mission_entry.sh 실행. 대기열 없이 한 번에 하나만.
    def submit(self, destination, source='manual', task_id=None):
        if source == 'host' and not task_id:
            raise ValueError('공정 명령에는 command-id가 필요합니다.')
        aliases = {v: k for k, v in STATIONS.items()}
        destination = aliases.get(destination.upper(), destination.lower())
        if destination not in STATIONS:
            raise ValueError('경로: mat/WAREHOUSE, asm/ASSEMBLY, rest/WAITING, park/HOME')
        task_id = command_id(task_id or uuid.uuid4().hex)
        from action_gate import load_gate
        gate = load_gate(self.data / 'action_gate.json')
        if gate and gate.get('estop', True):
            raise RuntimeError('Emergency stop is latched; RESTART clears it without resuming motion')
        mode = self.mode()
        if (mode, source) not in (('individual', 'manual'), ('process', 'host')):
            raise RuntimeError(f'{mode} 모드는 {source} 명령을 받지 않습니다.')
        previous = self.read('commands/' + task_id + '.json')
        if previous:
            if previous['destination'] != destination or previous['request_source'] != source:
                raise ValueError('동일 command-id에 다른 명령을 사용할 수 없습니다.')
            self.current()
            return dict(self.read('commands/' + task_id + '.json'), duplicate=True)
        if self.busy():
            raise RuntimeError('이동 중입니다. 대기열에 넣지 않습니다. 완료 후 다음 경로를 보내세요.')
        self.current()
        task = {'robot': self.robot, 'command_id': task_id, 'destination': destination,
                'target_station': STATIONS[destination], 'operation_mode': mode, 'request_source': source,
                'stage': 'accepted', 'status': 'running', 'started_unix': time.time(), 'updated_unix': time.time()}
        self.save_task(task)
        try:
            self.run(['systemctl', '--user', 'reset-failed', self.robot + '-mission.service'], capture_output=True, text=True)
            self.run(['systemd-run', '--user', '--collect', '--unit=' + self.robot + '-mission',
                      '--property=KillSignal=SIGINT', '--property=TimeoutStopSec=10',
                      '--property=ExecStopPost=/bin/bash ' + str(self.here / 'manage.sh') + ' stop',
                      '/bin/bash', str(self.here / 'mission_entry.sh'), destination,
                      '--command-id', task_id, '--operation-mode', mode, '--request-source', source],
                     check=True, capture_output=True, text=True)
        except Exception as e:
            task.update(stage='start_failed', status='failed', error=str(e), updated_unix=time.time())
            self.save_task(task)
            raise
        return self.read('commands/' + task_id + '.json')

    # 임무 서비스 정지 + manage.sh stop(모터 idle, 주행/도킹 중단) 후 기록을 '운영자 중단'으로 남긴다.
    def stop(self):
        self.run(['systemctl', '--user', 'stop', self.robot + '-mission.service'], capture_output=True, text=True)
        self.run(['/bin/bash', str(self.here / 'manage.sh'), 'stop'], check=True, capture_output=True, text=True)
        task = self.read('mission_state.json')
        if task and task.get('command_id') and task.get('status') not in FINISHED:
            task.update(stage='interrupted', status='cancelled', error='운영자 중단', updated_unix=time.time())
            self.save_task(task)
        return {'robot': self.robot, 'mode': self.mode(), 'task': task, 'busy': self.busy()}

    # 운영자가 '지정 주차 위치에 서 있다'고 확인 → 초기 위치 적용, 다음 주행에서 출차를 한 번 하도록 표시.
    def parked(self):
        runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
        with (runtime / f'{self.robot}-prepare-{os.getuid()}.lock').open('a') as preparation:
            try:
                fcntl.flock(preparation, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('다른 준비 명령 실행 중입니다.')
            with self.lock():
                if self.mode() != 'individual':
                    raise RuntimeError('주차 초기 위치 확인은 개별 모드에서만 가능합니다.')
                if self.busy(include_preparation=False):
                    raise RuntimeError('이동 중에는 초기 위치를 바꿀 수 없습니다.')
            # Admission stays reserved, but a localization recovery may take operation.lock.
            r = self.run(['/bin/bash', str(self.here / 'confirm_parked.sh'), str(preparation.fileno())],
                         pass_fds=(preparation.fileno(),), capture_output=True, text=True)
            if r.returncode:
                raise RuntimeError('주차 초기 위치 확인 실패:\n' + (r.stdout + r.stderr)[-5000:])
        return {'robot': self.robot, 'mode': self.mode(), 'localization_ready': True,
                'departure_pending': True, 'motion_sent': False, 'detail': r.stdout.strip()}

    # manage.sh ready 실행: 위치추정·Nav2·카메라 등을 미리 준비 (움직이지 않음).
    def ready(self):
        if self.busy():
            raise RuntimeError('이동 중에는 준비 명령을 실행할 수 없습니다.')
        started = time.monotonic()
        r = self.run(['/bin/bash', str(self.here / 'manage.sh'), 'ready'], capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError('준비 실패:\n' + (r.stdout + r.stderr)[-5000:])
        return {'robot': self.robot, 'mode': self.mode(), 'ready': True, 'motion_sent': False,
                'elapsed_s': round(time.monotonic()-started, 2), 'detail': r.stdout.strip()}


# 터미널 명령 처리: mode / submit / status / stop / parked / ready. 결과는 JSON 한 줄로 출력.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    mode = sub.add_parser('mode'); mode.add_argument('mode', nargs='?', choices=['individual', 'process'])
    send = sub.add_parser('submit'); send.add_argument('destination')
    send.add_argument('--source', choices=['manual', 'host'], default='manual')
    send.add_argument('--command-id')
    status = sub.add_parser('status'); status.add_argument('--command-id')
    sub.add_parser('stop')
    sub.add_parser('parked', help='실제 지정 주차 위치·방향에 정지해 있음을 확인하고 초기 위치 적용')
    sub.add_parser('ready', help='위치 추정·Nav2·카메라를 미리 준비하며 주행하지 않음')
    args = parser.parse_args()
    try:
        op = Operation(Path(__file__).parent)
        # Preparation owns its own lock and performs admission under operation.lock.
        # Do not retain this parent lock while the child checks recovery admission.
        if args.action in ('ready', 'parked'):
            result = op.ready() if args.action == 'ready' else op.parked()
        else:
            with op.lock():
                if args.action == 'mode':
                    result = op.set_mode(args.mode) if args.mode else {'robot': op.robot, 'mode': op.mode()}
                elif args.action == 'submit':
                    if args.source == 'host' and not args.command_id:
                        raise ValueError('공정 명령에는 --command-id가 필요합니다.')
                    result = op.submit(args.destination, args.source, args.command_id)
                elif args.action == 'stop':
                    result = op.stop()
                else:
                    current = op.current()
                    task = op.read('commands/' + command_id(args.command_id) + '.json') if args.command_id else current
                    result = {'robot': op.robot, 'mode': op.mode(), 'busy': op.busy(), 'task': task}
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as e:
        print(json.dumps({'error': str(e)}, ensure_ascii=False))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
