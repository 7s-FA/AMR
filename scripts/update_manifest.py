#!/usr/bin/env python3
"""Explicitly refresh source hashes after reviewing an onboard source change."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
for robot in ('M1','M2'):
    p=root/robot/'runtime_manifest.json';d=json.loads(p.read_text())
    for e in d['files']:e['sha256']=hashlib.sha256((root/e['source']).read_bytes()).hexdigest()
    p.write_text(json.dumps(d,indent=2)+'\n')
