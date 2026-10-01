"""Persist only terminal completion backed by the controller's final report."""
import json
import sys
import time
from collections import deque
from pathlib import Path


def terminal_result(mode, exit_code, log_file):
    if mode not in ('dock', 'park', 'rest'):
        raise ValueError('Unknown terminal mode')
    try:
        with Path(log_file).open() as stream:
            lines = list(deque(stream, maxlen=40))
    except OSError:
        lines = []
    report = {}
    for line in lines:
        try:
            item = json.loads(line[line.index('{'):])
        except (ValueError, TypeError):
            continue
        if isinstance(item, dict) and 'state' in item and 'reason' in item:
            report = item
    if mode == 'rest':
        verified = (report.get('state') == 'RESTED' and report.get('stopped') is True
                    and report.get('ir_high') is True and report.get('reason') == 'ir_high_and_stationary')
    else:
        verified = (report.get('state') == 'DOCKED'
                    and report.get('reason') == 'ir_high_and_stationary')
    success = exit_code == 0 and verified
    return {'mode': mode, 'status': 'success' if success else 'failed',
            'exit_code': exit_code, 'completed_unix': time.time(), 'log_file': str(log_file),
            'controller_state': report.get('state'), 'controller_report': report,
            'reason': report.get('reason') or (''.join(lines)[-3000:].strip() or 'terminal_completion_unverified'),
            'stopped': success, 'terminal_verified': success,
            'dock_verified': success and mode != 'rest'}


def arrival_result(mode, evidence):
    if (mode not in ('dock', 'park', 'rest') or evidence.get('mode') != mode or evidence.get('status') != 'success'
            or evidence.get('exit_code') != 0 or evidence.get('stopped') is not True
            or evidence.get('terminal_verified') is not True
            or (mode != 'rest' and evidence.get('dock_verified') is not True)):
        raise RuntimeError('Terminal completion evidence is missing or inconsistent')
    return {'success': True, 'stopped': True, 'terminal_verified': True,
            'terminal_mode': mode, 'dock_verified': mode != 'rest'}


if __name__ == '__main__':
    path, mode, code, log = sys.argv[1:]
    result = terminal_result(mode, int(code), log)
    dest = Path(path)
    temp = dest.with_suffix('.tmp')
    temp.write_text(json.dumps(result, ensure_ascii=False))
    temp.replace(dest)
    raise SystemExit(0 if result['status'] == 'success' else 1)
