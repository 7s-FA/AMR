#!/usr/bin/env python3
# ========================================================================
# 역할: 원본 파일을 고친 뒤 runtime_manifest.json 의 SHA-256 해시를 새로 계산해 저장.
# 실행: 수동. python3 scripts/update_manifest.py (코드 수정 후 한 번).
# ========================================================================
"""Explicitly refresh source hashes after reviewing an onboard source change."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
for robot in ('M1','M2'):
    p=root/robot/'runtime_manifest.json';d=json.loads(p.read_text())
    for e in d['files']:e['sha256']=hashlib.sha256((root/e['source']).read_bytes()).hexdigest()
    p.write_text(json.dumps(d,indent=2)+'\n')
