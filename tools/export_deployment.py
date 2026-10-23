#!/usr/bin/env python3
"""Reconstruct a reviewed robot source tree in a NEW directory; no robot I/O."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from public_examples import examples

ROOT=Path(__file__).resolve().parents[1]
def git(*args):
    return subprocess.check_output(['git','-C',str(ROOT),*args])
def write(path,data,mode='100644'):
    path.parent.mkdir(parents=True,exist_ok=True)
    try:data=examples(data.decode('utf-8')).encode('utf-8')
    except UnicodeDecodeError:pass
    path.write_bytes(data)
    if mode=='100755' or path.suffix in ('.sh','.bash'):path.chmod(0o755)
def export(robot,output):
    target=Path(output).absolute()
    if target.exists():raise ValueError('Output already exists; choose a new review directory')
    snapshot=ROOT/'deployments'/robot
    meta=json.loads((snapshot/'snapshot.json').read_text())
    base=meta['comparison_base']
    listing=git('ls-tree','-r','-z',base).split(b'\0')
    # Preflight the entire base before creating output. No symlinks or gitlinks.
    entries=[]
    for row in listing:
        if not row:continue
        head,name=row.split(b'\t',1);mode,kind,sha=head.decode().split();name=name.decode()
        if kind!='blob' or mode not in ('100644','100755'):raise ValueError('Unsupported base entry: '+name)
        rel=Path(name)
        if rel.is_absolute() or '..' in rel.parts:raise ValueError('Unsafe path')
        entries.append((name,mode,sha))
    target.mkdir(parents=True)
    for name,mode,sha in entries:write(target/name,git('cat-file','blob',sha),mode)
    for source in (snapshot/'source').rglob('*'):
        if source.is_file():write(target/source.relative_to(snapshot/'source'),source.read_bytes())
    # Put runtime-only changes into their manifest source locations in this
    # isolated export. Otherwise a later configure would silently lose them.
    for entry in meta['runtime_overrides']:
        source=snapshot/'runtime-overrides'/entry['runtime_path']
        write(target/entry['source'],source.read_bytes())
    for name in (robot,):
        path=target/name/'runtime_manifest.json';manifest=json.loads(path.read_text())
        for entry in manifest['files']:
            entry['sha256']=hashlib.sha256((target/entry['source']).read_bytes()).hexdigest()
        path.write_text(json.dumps(manifest,indent=2)+'\n')
    (target/'DEPLOYMENT_README.md').write_text(
        '# '+robot+' review export\n\n'
        'Source base: '+base+'\n\n'
        'Only use the '+robot+' profile in this export. The other profile is not a validated deployment.\n'
        'Runtime overrides have been folded into source files and manifest hashes refreshed.\n'
        'Configure creates runtime files only; build/install/start are separate actions.\n'
        'Set example 192.0.2.* addresses, SSH users and /home paths to the real installation.\n'
        'This is a review copy, not a deployment or hardware test. No service or robot command ran.\n'
        'Absent files in the inventory were not automatically deleted. Logs, runtime state and models are omitted.\n')
    return target
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('robot',choices=['M1','M2']);p.add_argument('--output',required=True)
    a=p.parse_args();print(export(a.robot,a.output))
