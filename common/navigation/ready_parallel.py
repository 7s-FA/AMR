#!/usr/bin/env python3
# ========================================================================
# 역할: 로봇 준비를 병렬로 한 번 수행. 위치추정·본체를 먼저 확인한 뒤 카메라/도킹, rest 대기, Nav2, 액션 서버를 동시에 준비하고
#       마지막에 motion_mode 'prepare'(Nav2 준비 확인, 모터는 idle)와 웨이포인트 대기 작업자 시작까지 한다. 주행하지 않음.
# 실행: manage.sh ready (operation.py ready, host_prepare.py 가 호출). motion_mode.py 도 start_missing() 을 사용.
# ========================================================================
"""One-shot parallel startup. No routes, velocity commands or service restarts."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import subprocess
import time


# systemd 서비스가 꺼져 있을 때만 시작 (실행 중이면 그대로, 종료 중이면 오류).
def start_missing(unit, run=subprocess.run, wait=True):
    state = run(['systemctl', '--user', 'show', unit, '-p', 'ActiveState', '--value'],
                capture_output=True, text=True, check=True, timeout=5).stdout.strip()
    if state in ('active', 'activating', 'reloading'):
        return 'already running'
    if state == 'deactivating':
        raise RuntimeError(unit + ': stopping; retry after shutdown')
    run(['systemctl', '--user', 'start', *([] if wait else ['--no-block']), unit], check=True, timeout=20)
    return 'started'


# 준비 작업 여러 개를 4스레드로 동시에 실행하고 실패를 모아 보고.
def parallel_checks(jobs, optional=()):
    errors = []
    # Each job checks its own dependencies; there is no arbitrary global order.
    with ThreadPoolExecutor(max_workers=4) as pool:
        pending = {pool.submit(fn): name for name, fn in jobs.items()}
        while pending:
            done = [future for future in pending if future.done()]
            if not done:
                time.sleep(.05)
                continue
            for future in done:
                name = pending.pop(future)
                try:
                    result = future.result()
                    print(name + ': ' + str(result or 'ready'), flush=True)
                except Exception as exc:
                    message = name + ': ' + str(exc)
                    if name in optional:
                        print('optional preparation warning: ' + message + '; normal execution path retained', flush=True)
                    else:
                        errors.append(message)
                        print(message, flush=True)
        if errors:
            raise RuntimeError('; '.join(errors))


# 이동 중이면 거부 → warm.sh → 병렬 준비 → 모드 prepare → 웨이포인트 대기 서비스 시작.
def main():
    here = Path(__file__).absolute().parent
    robot = here.parent.name
    if robot not in ('burger1', 'burger2'):
        raise ValueError('Run from a robot navigation directory')
    from operation import Operation
    op = Operation(here)
    with op.lock():
        if op.busy(include_preparation=False):
            raise RuntimeError('Robot is moving; preparation refused')
    env = dict(os.environ, BURGER_PARALLEL_PREPARE='1')
    # 같은 폴더의 쉘 스크립트를 실행하는 작업 함수를 만든다.
    def script(name, timeout=100):
        def execute():
            subprocess.run(['/bin/bash', str(here/name)], env=env, check=True, timeout=timeout)
        return execute
    # Localization supplies map/TF to Nav2. Check this dependency before
    # launching the large controller/plugin graph on a cold robot.
    script('warm.sh', 130)()
    print('localization/base: ready', flush=True)
    jobs = {'camera/detection': script('ensure_docking_ready.sh', 30),
            'REST standby': script('ensure_rest_ready.sh', 30),
            'Nav2': lambda: start_missing(robot+'-nav2.service'),
            'Action': lambda: start_missing('m'+robot[-1]+'-action.service')}
    parallel_checks(jobs)
    # Ownership and real lifecycle/TF checks still gate the final handoff.
    subprocess.run(['/bin/bash', str(here/'set_mode.sh'), 'prepare'], check=True, timeout=150)
    if (here/'ensure_waypoint_ready.sh').exists():
        try:
            print('waypoint standby: '+start_missing(robot+'-waypoint-ready.service', wait=False), flush=True)
        except Exception as exc:
            print('waypoint standby warning: '+str(exc)+'; normal execution path retained', flush=True)
    print(robot + ' parallel preparation complete; startup program exits.', flush=True)


if __name__ == '__main__':
    main()
