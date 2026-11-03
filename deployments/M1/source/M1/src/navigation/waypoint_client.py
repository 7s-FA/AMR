#!/usr/bin/env python3
# ========================================================================
# 역할: 웨이포인트(또는 rest) 대기 작업자에 요청 1건을 보내고, 실행 로그를 실시간으로 받아 출력한다.
#       연결 후에는 재시도·직접 실행으로 넘어가지 않는다 (같은 이동을 두 번 하지 않게).
# 실행: run_waypoints.sh, run_rest.sh, ensure_waypoint_ready.sh(--check/--wait-ready), ensure_rest_ready.sh.
# 종료 코드: 2 = 작업자 없음(요청 안 보냄) → 호출한 스크립트가 기존 직접 실행 경로를 쓴다.
# ========================================================================
"""One explicit request, streamed logs; never retry/fallback after connecting."""
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time


# 작업자가 살아 있고, 같은 부팅·같은 코드(해시)인지 확인.
def available(address, entry):
    try:
        meta = json.loads(address.with_suffix('.json').read_text())
        if not address.exists() or os.path.abspath(meta['entry']) != os.path.abspath(entry): return False
        if meta['boot'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip(): return False
        if hashlib.sha256(entry.read_bytes()).hexdigest() != meta['sha256']: return False
        if 'config' in meta and hashlib.sha256(Path(meta['config']).read_bytes()).hexdigest() != meta['config_sha256']: return False
        os.kill(meta['pid'], 0)
        return True
    except (OSError, ValueError, KeyError): return False


# --check / --wait-ready / 실제 요청 처리.
def main():
    if len(sys.argv) < 2: return 2
    entry = Path(sys.argv[1]); args = sys.argv[2:]
    address = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/'+str(os.getuid())))/os.environ.get('BURGER_PREPARED_SOCKET_NAME', 'burger1-waypoint-ready.sock')
    if len(args) == 2 and args[0] == '--wait-ready':
        end = time.monotonic() + float(args[1])
        while True:
            if available(address, entry): return 0
            remaining = end-time.monotonic()
            if remaining <= 0: return 2
            time.sleep(min(.1, remaining))
    if not available(address, entry): return 2
    if args == ['--check']: return 0
    probe = args in (['--probe'], ['--verify'])
    request = {'op': 'verify' if args == ['--verify'] else 'probe' if probe else 'run',
               'argv': args, 'issued': time.monotonic()}
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(1)
        try: connection.connect(str(address))
        except (FileNotFoundError, ConnectionRefusedError): return 2
        # Beyond this point the request may have been accepted: no cold fallback.
        try:
            connection.sendall((json.dumps(request)+'\n').encode())
            connection.settimeout(650)
            stream = connection.makefile('rb')
            for line in stream:
                response = json.loads(line)
                if response['type'] == 'log':
                    print(response['text'], end='', flush=True)
                elif response['type'] == 'result':
                    if response.get('error'): print(response['error'], file=sys.stderr)
                    if probe: print(json.dumps(response, ensure_ascii=False))
                    # 2 is reserved for LOCAL unavailability before any send.
                    # A remotely executed parser/route exit 2 must never retry.
                    return 1 if response['code'] == 2 else response['code']
            raise RuntimeError('Prepared waypoint connection closed without result; not retried')
        except KeyboardInterrupt: return 130  # Closing the socket cancels in worker.
        except Exception as exc:
            print('Prepared waypoint request failed: '+str(exc), file=sys.stderr); return 1

if __name__ == '__main__': raise SystemExit(main())
