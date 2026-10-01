#!/usr/bin/env python3
"""Submit a mode change to the robot-local persistent ROS worker."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('robot', choices=('burger1', 'burger2'))
    parser.add_argument('mode', choices=('prepare', 'nav', 'direct', 'idle'))
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    unit = args.robot+'-nav-control.service'
    address = str(Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))/(args.robot+'-nav-control.sock'))
    if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode != 0:
        subprocess.run(['systemctl', '--user', 'reset-failed', unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
