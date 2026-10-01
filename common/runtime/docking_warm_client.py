#!/usr/bin/env python3
"""Execute one explicit docking request; never retry an accepted request."""
import argparse,hashlib,json,os,socket,sys,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('robot');p.add_argument('--check',action='store_true');p.add_argument('--diagnose',action='store_true');p.add_argument('--log-dir');p.add_argument('--mode',choices=('normal','parking'),default='normal');p.add_argument('--profile',choices=('active','idle'));p.add_argument('--prepare',action='store_true');a=p.parse_args()
 path=Path(os.environ.get('XDG_RUNTIME_DIR','/run/user/'+str(os.getuid())))/(a.robot+'-docking-ready.sock')
 try:
  meta=json.loads(path.with_suffix('.json').read_text())
  if not path.exists() or not all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in meta['signature'].items()):return 2
  if a.mode not in meta.get('available_modes',[]):return 2
  if a.check:return 0
  with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
   conn.settimeout(5 if a.diagnose else 110);conn.connect(str(path));conn.sendall((json.dumps({'action':'profile' if a.profile else 'prepare' if a.prepare else 'status' if a.diagnose else 'start','profile':a.profile,'mode':a.mode,'issued':time.monotonic(),'log_dir':a.log_dir})+'\n').encode())
   if not (a.diagnose or a.profile or a.prepare):print('미리 준비된 영상 검출을 사용해 도킹을 시작합니다.',flush=True)
   data=conn.makefile('rb').readline(100000)
   if not data:raise RuntimeError('Docking standby disconnected; request will not be retried')
   result=json.loads(data);print(json.dumps(result.get('report',result),ensure_ascii=False),flush=True);return result['code']
 except KeyboardInterrupt:return 130
 except Exception as exc:print('Warm docking error: '+str(exc),file=sys.stderr);return 1
if __name__=='__main__':raise SystemExit(main())
