#!/usr/bin/env python3
# ========================================================================
# 역할: colcon --symlink-install 이 상대 링크를 그대로 복사해 깨지는 경우, 설치 폴더의 링크를 올바른 위치로 다시 건다.
# 실행: build.sh 끝에서 자동 실행.
# ========================================================================
"""ament may copy relative source links verbatim; re-anchor them at install paths."""
import os
from pathlib import Path
import sys


# 설치 폴더를 돌며 깨진 링크를 원본으로 다시 연결.
def repair(workspace):
    workspace = Path(workspace).absolute()
    repaired = 0
    for package in (workspace/'src').iterdir():
        if not package.is_dir():continue
        by_link = {}
        for source in package.rglob('*'):
            if source.is_symlink():
                by_link.setdefault(os.readlink(source), []).append(source)
        installed = workspace/'install'/package.name
        if not installed.is_dir():continue
        for dest in installed.rglob('*'):
            if not dest.is_symlink():continue
            matches = by_link.get(os.readlink(dest), [])
            if not matches:continue
            targets = {p.resolve() for p in matches}
            if len(targets)!=1:raise RuntimeError('Ambiguous source link: '+str(dest))
            # Point to runtime source, preserving its profile-specific indirection.
            source = matches[0]
            target = os.path.relpath(source, dest.parent)
            dest.unlink()
            dest.symlink_to(target)
            repaired += 1
    broken = [str(p) for p in (workspace/'install').rglob('*') if p.is_symlink() and not p.exists()]
    if broken:raise RuntimeError('Broken installed links: '+', '.join(broken))
    print('Verified installed links; repaired:',repaired)


if __name__=='__main__':repair(sys.argv[1])
