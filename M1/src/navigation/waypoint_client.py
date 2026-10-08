#!/usr/bin/env python3
"""One explicit request, streamed logs; never retry/fallback after connecting."""
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time


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
