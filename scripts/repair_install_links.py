#!/usr/bin/env python3
"""ament may copy relative source links verbatim; re-anchor them at install paths."""
import os
from pathlib import Path
import sys


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
