#!/usr/bin/env python3
# ========================================================================
# 역할: 주행 모드 전환 요청 클라이언트. motion_mode.py(서버)의 유닉스 소켓에 {mode} 를 보내고 결과를 출력.
# 실행: set_mode.sh <모드> → python3 motion_client.py burgerN prepare|nav|direct|idle
#       서버가 꺼져 있으면 nav-control 서비스를 켜고 접속한다.
# ========================================================================
"""Submit a mode change to the robot-local persistent ROS worker."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time


# 소켓 접속(필요시 서비스 시작) → 모드 요청 → 성공/실패 응답 확인.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('robot', choices=('burger1', 'burger2'))
    parser.add_argument('mode', choices=('prepare', 'nav', 'direct', 'idle'))
    args = parser.parse_args()
    here = Path(__file__).absolute().parent
    unit = args.robot+'-nav-control.service'
    address = str(Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))/(args.robot+'-nav-control.sock'))
    if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode != 0:
        subprocess.run(['systemctl', '--user', 'reset-failed', unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # 등록된 서비스 파일이 있으면 그것을 켠다. 같은 이름으로 임시 서비스(systemd-run)를 만들면 실패하기 때문
        # (2026-10-09 수정: nav-control 이 꺼진 상태에서 모드 전환 요청 시 'already loaded' 오류로 실패하던 문제).
        fragment = subprocess.run(['systemctl', '--user', 'show', unit, '-p', 'FragmentPath', '--value'],
                                  capture_output=True, text=True).stdout.strip()
        if fragment:
            result = subprocess.run(['systemctl', '--user', 'start', unit])
        else:
            result = subprocess.run(['systemd-run', '--user', '--collect', '--quiet', '--unit='+unit,
                '--property=KillSignal=SIGINT', '--property=TimeoutStopSec=5',
                '--property=Restart=on-failure', '--property=RestartSec=1',
                '/bin/bash', str(here/'nav_control_service.sh')])
        if result.returncode: raise RuntimeError('Navigation control service start failed')
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    deadline = time.monotonic()+10
    while True:
        try: connection.connect(address); break
        except (FileNotFoundError, ConnectionRefusedError):
            if time.monotonic() >= deadline: raise RuntimeError('Navigation control service not ready')
            time.sleep(.05)
    with connection:
        connection.settimeout(160)  # Includes one bounded Nav2 preparation recovery.
        connection.sendall((json.dumps({'mode': args.mode})+'\n').encode())
        raw = b''
        while b'\n' not in raw and len(raw) < 65536:
            chunk = connection.recv(4096)
            if not chunk: raise RuntimeError('Navigation control connection closed; command was not retried')
            raw += chunk
        reply = json.loads(raw)
    if not reply['success']: raise RuntimeError(reply.get('message', 'Mode change failed'))
    print(f"{args.robot} 명령 제어권: {args.mode} / 전환 {reply['seconds']:.2f}초", flush=True)


if __name__ == '__main__': main()
