#!/usr/bin/env python3
# ========================================================================
# 역할: configure(실행 폴더 새로 만들기·전체 빌드) 없이, runtime_manifest.json 기준으로 .runtime/<로봇> 의 링크만 다시 맞춘다.
#   - 목록에 있는 파일: 실행 폴더 쪽을 원본(AMR/...)을 가리키는 상대 링크로 (복사본이나 다른 곳을 가리키는 링크는 교체)
#   - 원본이 없어진 링크(예: 지운 ir_sensor.py, 예전 F411/B2 사본)는 삭제
#   - units·dds_*.xml·handoff/pc_env.bash(생성 파일)·빌드 결과(build/install/log)·data 는 건드리지 않는다.
# 실행: 서비스를 멈춘 뒤 python3 scripts/relink_runtime.py --robot M1 [--dry-run]
# ========================================================================
"""Re-point an existing linked runtime at the repository sources listed in the manifest."""
import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKIP_PARTS = {'build', 'install', 'log', 'data', '__pycache__'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', choices=['M1', 'M2'], required=True)
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    manifest = json.loads((ROOT/a.robot/'runtime_manifest.json').read_text())
    runtime = ROOT/'.runtime'/a.robot
    targets = {'workspace': runtime/'final_robot_ws', 'camera': runtime/manifest['camera_directory']}
    if not all(d.is_dir() for d in targets.values()):
        raise SystemExit(f'실행 폴더가 없습니다: {runtime} (처음이면 robot.py --robot {a.robot} configure)')
    changed = removed = 0
    wanted = set()
    for entry in manifest['files']:
        kind, rel = entry['target'].split('/', 1)
        if kind not in targets:
            continue
        source, dest = ROOT/entry['source'], targets[kind]/rel
        if dest.name.startswith('dds_') or dest == targets['workspace']/'handoff/pc_env.bash':
            continue   # 프로그램이 다시 만드는 파일 (materialize 와 같음)
        wanted.add(dest)
        link = os.path.relpath(source, dest.parent)
        if dest.is_symlink() and os.readlink(dest) == link:
            continue
        print(('[연결] ' if not (dest.exists() or dest.is_symlink()) else '[교체] ') + f'{dest} -> {link}')
        changed += 1
        if not a.dry_run:
            dest.parent.mkdir(parents=True, exist_ok=True)
            if dest.is_symlink() or dest.exists():
                dest.unlink()
            dest.symlink_to(link)
    # 원본이 사라진 링크 정리 (저장소 안을 가리키던 것만)
    for base in targets.values():
        for path in base.rglob('*'):
            if SKIP_PARTS & set(path.relative_to(base).parts) or not path.is_symlink() or path in wanted:
                continue
            target = (path.parent/os.readlink(path)).resolve()
            if not target.exists() and str(target).startswith(str(ROOT)):
                print(f'[삭제] 원본 없는 링크 {path} -> {os.readlink(path)}')
                removed += 1
                if not a.dry_run:
                    path.unlink()
    print(f'{"(시험) " if a.dry_run else ""}바꾼 링크 {changed}개, 지운 링크 {removed}개')


if __name__ == '__main__':
    main()
