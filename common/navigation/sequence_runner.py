#!/usr/bin/env python3
"""Station mission: waypoint route -> direct terminal action, never skip failed steps."""
import argparse,json,os,re,signal,subprocess,time
from collections import deque
from pathlib import Path

def run_step(command):
    """Keep live journal output and return the underlying failure to the host."""
    tail=deque(maxlen=24)
    with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1) as child:
        try:
            for line in child.stdout:
                print(line,end='',flush=True)
                tail.append(line.rstrip()[-1000:])
            code=child.wait()
        except BaseException:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait()
            raise
    if code:
        errors=[line for line in tail if 'RuntimeError:' in line or '[ERROR]' in line or 'Nav2 error_code=' in line]
        reason='\n'.join(errors[-4:] or list(tail)[-8:])
        if re.search(r'\bNav2 error_code=703(?:,|\s|$)', reason):
            reason+='\n회전 충돌 검사 거부(Spin COLLISION_AHEAD): 실제 장애물 또는 장애물 지도·TF 상태를 확인하세요. 도킹은 시작하지 않았습니다.'
        raise RuntimeError(f'{Path(command[1]).name} 실패 (종료 코드 {code}):\n{reason or "상세 출력 없음"}')

class Sequence:
    def __init__(self,run,terminal):self.run,self.terminal=run,terminal
    def execute(self,destination):
        self.run(['run_selected_waypoints.sh',destination])
        self.terminal({'mat':'dock','asm':'dock','park':'park','rest':'rest'}[destination])

def collision_failure_snapshot(here, data, robot, task_id, error, run=subprocess.run):
    if not re.search(r'\bNav2 error_code=703(?:,|\s|$)', error):
        return {}
    folder=data/'nav2_failures';folder.mkdir(parents=True,exist_ok=True)
    snapshot=folder/((task_id or str(time.time_ns()))+'.json')
    log=snapshot.with_suffix('.log')
    command=['/bin/bash','-c',
             'source "$1/nav_env.bash"; exec python3 "$1/capture_nav2_failure.py" --robot "$2" --output "$3"',
             'capture',str(here),robot,str(snapshot)]
    try:
        with log.open('w') as stream:
            result=run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=22,check=False)
        if result.returncode or not snapshot.exists():
            return {'diagnostic_error':'상태 수집 실패; 원래 주행 오류 유지', 'diagnostic_log':str(log)}
        return {'diagnostic_file':str(snapshot),'diagnostic_log':str(log)}
    except Exception as exc:
        return {'diagnostic_error':str(exc),'diagnostic_log':str(log)}

def atomic_json(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2));temp.replace(path)

def main():
    p=argparse.ArgumentParser();p.add_argument('destination',choices=['mat','asm','park','rest']);p.add_argument('--robot',required=True);p.add_argument('--command-id');p.add_argument('--operation-mode',default='individual');p.add_argument('--request-source',default='manual');a=p.parse_args()
    here=Path(__file__).resolve().parent;root=here.parents[2];data=root/'data'/a.robot;state=data/'mission_state.json';terminal_file=data/'terminal_result.json'
    from operation import command_id, STATIONS
    if a.command_id: command_id(a.command_id)
    started=time.time();label={'mat':'재료창고','asm':'조립대','park':'지정 주차장','rest':'휴식공간'}[a.destination]
    def write(stage,status='running',**extra):
        record={'robot':a.robot,'destination':a.destination,'label':label,'stage':stage,'status':status,'started_unix':started,'updated_unix':time.time(),
                'command_id':a.command_id,'operation_mode':a.operation_mode,'request_source':a.request_source,'target_station':STATIONS[a.destination],**extra}
        if a.command_id: atomic_json(data/'commands'/(a.command_id+'.json'),record)
        atomic_json(state,record)
    def run(args):
        write(args[0]);print('단계: '+' '.join(args),flush=True)
        run_step(['/bin/bash',str(here/args[0]),*args[1:]])
    def terminal(action):
        if terminal_file.exists():terminal_file.unlink()
        if action=='rest':run(['rest.sh','start'])
        else:run(['manage.sh',action])
        write('terminal_'+action)
        deadline=time.monotonic()+115
        unit=a.robot+('-rest.service' if action=='rest' else '-docking.service')
        while time.monotonic()<deadline:
            if terminal_file.exists():
                result=json.loads(terminal_file.read_text())
                if result.get('status')!='success':raise RuntimeError('종단 이동 실패: '+str(result))
                # Wait until controller/GPIO is released before acknowledging arrival.
                if subprocess.run(['systemctl','--user','is-active','--quiet',unit]).returncode==0:
                    time.sleep(.1);continue
                return
            info=subprocess.run(['systemctl','--user','show',unit,'-p','ActiveState','-p','Result'],capture_output=True,text=True).stdout
            if 'ActiveState=failed' in info:raise RuntimeError(unit+' 실패')
            time.sleep(.15)
        raise RuntimeError('종단 이동 완료 시간 초과')
    try:
        Sequence(run,terminal).execute(a.destination)
        write('arrived','success',result={'success':True,'stopped':True,'dock_verified':True,'station_id':STATIONS[a.destination]},terminal_evidence=json.loads(terminal_file.read_text()));print(label+' 도착 완료',flush=True)
        atomic_json(data/'station_state.json',{'station':a.destination,'completed_unix':time.time()})
        return 0
    except BaseException as e:
        collision = bool(re.search(r'\bNav2 error_code=703(?:,|\s|$)', str(e)))
        if collision: write('failure_diagnostics',error=str(e))
        subprocess.run(['/bin/bash',str(here/'manage.sh'),'stop'],check=False)
        evidence=collision_failure_snapshot(here,data,a.robot,a.command_id,str(e)) if collision else {}
        write('interrupted' if isinstance(e,KeyboardInterrupt) else 'failed','cancelled' if isinstance(e,KeyboardInterrupt) else 'failed',error=str(e),**evidence)
        print('전체 이동 중단: '+str(e),flush=True);return 130 if isinstance(e,KeyboardInterrupt) else 1
if __name__=='__main__':raise SystemExit(main())
