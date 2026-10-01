#!/usr/bin/env python3
"""Run on a robot. No SSH, host process scheduler or automatic motion."""
import argparse,json,os,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--runtime',required=True);p.add_argument('command',choices=['ready','parked','status','stop','mode','mat','asm','rest','park','action','install-services']);p.add_argument('args',nargs='*');a=p.parse_args()
r=Path(a.runtime).resolve();cfg=json.loads((r/'runtime.json').read_text());nav=Path(cfg['navigation'])
if a.command=='install-services':
    dest=Path.home()/'.config/systemd/user';dest.mkdir(parents=True,exist_ok=True)
    units=list(Path(cfg['units']).glob('*.service'))
    for unit in units:
        target=dest/unit.name
        if target.exists() and target.read_bytes()!=unit.read_bytes():raise SystemExit('Existing service differs; back up and review before replacement: '+str(target))
    for unit in units:(dest/unit.name).write_bytes(unit.read_bytes())
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    print('Units registered; no services started and no motion sent.')
elif a.command=='action':os.execv('/bin/bash',['bash',str(r/'start_action.sh'),*a.args])
else:
    args=[a.command,*a.args]
    if a.command in ('mat','asm','rest','park'):args=['submit',a.command,'--source','manual',*a.args]
    os.execv('/usr/bin/python3',['python3',str(nav/'operation.py'),*args])
