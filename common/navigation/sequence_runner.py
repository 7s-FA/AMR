#!/usr/bin/env python3
"""Station mission: waypoint route -> direct terminal action, never skip failed steps."""
import argparse,json,os,re,signal,subprocess,time
import yaml
from station_routes import terminal_mode
from terminal_evidence import arrival_result
from collections import deque
from pathlib import Path
from mission_retry import RetryPolicy, RetryExhausted
from retry_ready import authorization

def run_step(command, env=None):
    """Keep live journal output and return the underlying failure to the host."""
    tail=deque(maxlen=24)
    with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,env=env) as child:
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
    if code in (130,143,-signal.SIGINT,-signal.SIGTERM):
        raise KeyboardInterrupt('주행 요청 취소: 자동 재시도하지 않습니다.')
    if code:
        errors=[line for line in tail if 'RuntimeError:' in line or '[ERROR]' in line or 'Nav2 error_code=' in line or 'ENCODER_DEPARTURE_FAILED' in line]
        reason='\n'.join(errors[-4:] or list(tail)[-8:])
        if re.search(r'\bNav2 error_code=703(?:,|\s|$)', reason):
            reason+='\n회전 충돌 검사 거부(Spin COLLISION_AHEAD): 실제 장애물 또는 장애물 지도·TF 상태를 확인하세요. 도킹은 시작하지 않았습니다.'
        raise RuntimeError(f'{Path(command[1]).name} 실패 (종료 코드 {code}):\n{reason or "상세 출력 없음"}')

class Sequence:
    def __init__(self,run,terminal,stations,retry=None):
        self.run,self.terminal,self.stations,self.retry=run,terminal,stations,retry
    def execute(self,destination,resume_terminal=False):
        mode=terminal_mode(self.stations,destination)
        steps=[('navigation',lambda:self.run(['run_selected_waypoints.sh',destination])),
               ('terminal_'+mode,lambda:self.terminal(mode))]
        if resume_terminal:steps=steps[1:]
        for name,step in steps:
            if self.retry is None:step()
            else:self.retry.execute(name,step)
        return mode

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

def interrupted_terminal(data, destination):
    flag=data/'departure_pending'
    if not flag.exists() or not flag.read_text().startswith('terminal_started_'):
        return False
    attempt=data/'terminal_attempt.json'
    try:
        # Preserve the old deployment's destination before mission_state changes.
        previous=json.loads((attempt if attempt.exists() else data/'mission_state.json').read_text())
        original=previous['destination']
        if original not in ('mat','asm','park','rest'):raise ValueError('unknown destination')
        if not attempt.exists():atomic_json(attempt,{'destination':original})
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise RuntimeError('중단된 도킹 목적지 확인 불가: 자동 출차 금지') from exc
    if original!=destination:
        raise RuntimeError(f'중단된 도킹({original}): 다른 목적지 자동 출차 금지. 위치 확인 필요')
    return True

def dock_retry_backup(here, data, env, run=run_step):
    """Reverse only, then re-enter terminal docking; failure must not retry motion."""
    from action_gate import departure_speed
    try:
        speed=departure_speed(data/'action_gate.json')
        run(['/bin/bash',str(here/'set_mode.sh'),'nav'])
        run(['/bin/bash',str(here/'run_waypoints.sh'),
             '--encoder-backup-only','--pre-backup-open-loop','--pre-backup-distance',str(speed*3.5),
             '--pre-backup-speed',str(speed),'--pre-turn-angle-deg','0'],env=env)
    except Exception as exc:
        raise RuntimeError(f'ENCODER_DEPARTURE_FAILED: 도킹 재시도 후진 실패: {exc}') from exc
    finally:
        try:
            run(['/bin/bash',str(here/'set_mode.sh'),'idle'])
        except Exception as exc:
            raise RuntimeError(f'ENCODER_DEPARTURE_FAILED: 후진 제어권 해제 실패: {exc}') from exc

def docking_retry_needs_backup(evidence):
    """Use only the failed attempt's start and fresh-at-failure encoder pair."""
    try:
        if evidence['status'] != 'failed':
            raise ValueError('not a failed docking attempt')
        sample=evidence['controller_report']['docking_encoder']
        if sample['valid'] is not True:
            raise ValueError('missing or stale encoder')
        start,current=sample['start'],sample['current']
        if (len(start)!=2 or len(current)!=2 or
                any(type(v) is not int or not -2**31<=v<2**31 for v in [*start,*current])):
            raise ValueError('invalid encoder pair')
        delta=[(c-s+2**31)%2**32-2**31 for s,c in zip(start,current)]
        forward_ticks=sum(delta)/2
    except (KeyError,TypeError,ValueError) as exc:
        raise RuntimeError(f'ENCODER_DEPARTURE_FAILED: 도킹 이동량 확인 불가: {exc}') from exc
    print(f'도킹 실패 이동량: {forward_ticks:g}카운트; 후진 기준 2963카운트',flush=True)
    return forward_ticks>=2963

def main():
    p=argparse.ArgumentParser();p.add_argument('destination',choices=['mat','asm','park','rest']);p.add_argument('--robot',required=True);p.add_argument('--command-id');p.add_argument('--operation-mode',default='individual');p.add_argument('--request-source',default='manual');a=p.parse_args()
    here=Path(__file__).absolute().parent;root=here.parents[2];data=root/'data'/a.robot;state=data/'mission_state.json';terminal_file=data/'terminal_result.json'
    from operation import command_id, STATIONS
    if a.command_id: command_id(a.command_id)
    started=time.time();label={'mat':'재료창고','asm':'조립대','park':'지정 주차장','rest':'휴식공간'}[a.destination]
    active_step=''
    retry_status={}
    def write(stage,status='running',**extra):
        record={'robot':a.robot,'destination':a.destination,'label':label,'stage':stage,'status':status,'started_unix':started,'updated_unix':time.time(),
                'command_id':a.command_id,'operation_mode':a.operation_mode,'request_source':a.request_source,'target_station':STATIONS[a.destination],**retry_status,**extra}
        if a.command_id: atomic_json(data/'commands'/(a.command_id+'.json'),record)
        atomic_json(state,record)
    def run(args):
        nonlocal active_step
        active_step=args[0]
        write(args[0]);print('단계: '+' '.join(args),flush=True)
        run_step(['/bin/bash',str(here/args[0]),*args[1:]],env=step_env)
    def terminal(action):
        atomic_json(data/'terminal_attempt.json',{'destination':a.destination})
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
                arrival_result(action,result)
                return
            info=subprocess.run(['systemctl','--user','show',unit,'-p','ActiveState','-p','Result'],capture_output=True,text=True).stdout
            if 'ActiveState=failed' in info:raise RuntimeError(unit+' 실패')
            time.sleep(.15)
        raise RuntimeError('종단 이동 완료 시간 초과')
    try:
        resume_terminal=interrupted_terminal(data,a.destination)
        config_path=here/'mission_retry.json'
        retries=json.loads(config_path.read_text()).get('retries',0) if config_path.exists() else 0
        if retries and a.robot not in ('burger1','burger2'):raise ValueError('Common retry enabled for an unexpected robot')
        step_env=dict(os.environ)
        if retries:
            progress=data/'retry_progress'/((a.command_id or str(time.time_ns()))+'.json')
            step_env.update(BURGER_COMMON_RETRY='1',BURGER_ROUTE_PROGRESS=str(progress))
        def stop():
            subprocess.run(['/bin/bash',str(here/'manage.sh'),'stop'],check=True,timeout=25)
        def recover(stage):
            command=['/bin/bash','-c',
                     'source "$1/nav_env.bash"; exec python3 "$1/retry_ready.py" --robot "$2" --source "$3" --command-id "$4" --stage "$5"',
                     'recovery',str(here),a.robot,a.request_source,a.command_id or '',stage]
            run_step(command)
            if stage in ('terminal_dock','terminal_park'):
                try:
                    evidence=json.loads(terminal_file.read_text())
                except (OSError,ValueError) as exc:
                    raise RuntimeError(f'ENCODER_DEPARTURE_FAILED: 도킹 실패 기록 없음: {exc}') from exc
                if docking_retry_needs_backup(evidence):
                    dock_retry_backup(here,data,step_env)
                else:
                    print('도킹 시작 대비 150mm 미만: 후진 없이 제자리 도킹 재시도',flush=True)
        def report(stage,attempt,status,error):
            retry_status.update(retry_step=stage,retry_attempt=attempt,retry_limit=retries)
            write('retry_'+status,error=error)
            print(f'공통 재시도: {stage} / {attempt}/{retries} / {status}: {error}',flush=True)
        policy=RetryPolicy(retries,stop,recover,report,
                           authorized=lambda:authorization(data,a.command_id,a.request_source),
                           deadline=time.monotonic()+550) if retries else None
        stations=yaml.safe_load((here/'station_routes.yaml').read_text())
        if resume_terminal:
            recover('terminal_resume')
            print('중단된 동일 목적지: 출차·경로 주행 없이 도킹 재시작',flush=True)
        mode=Sequence(run,terminal,stations,policy).execute(a.destination,resume_terminal)
        evidence=json.loads(terminal_file.read_text())
        proof=arrival_result(mode,evidence);proof['station_id']=STATIONS[a.destination]
        write('arrived','success',result=proof,terminal_evidence=evidence);print(label+' 도착 완료',flush=True)
        atomic_json(data/'station_state.json',{'station':a.destination,'completed_unix':time.time()})
        return 0
    except BaseException as e:
        collision = bool(re.search(r'\bNav2 error_code=703(?:,|\s|$)', str(e)))
        if collision: write('failure_diagnostics',error=str(e))
        subprocess.run(['/bin/bash',str(here/'manage.sh'),'stop'],check=False)
        evidence=collision_failure_snapshot(here,data,a.robot,a.command_id,str(e)) if collision else {}
        recoverable=(a.robot in ('burger1','burger2') and active_step=='run_selected_waypoints.sh'
                     and isinstance(e,RuntimeError) and 'NAVIGATION_RETRIES_EXHAUSTED:' in str(e))
        write('interrupted' if isinstance(e,KeyboardInterrupt) else 'failed','cancelled' if isinstance(e,KeyboardInterrupt) else 'failed',
              error=str(e),recoverable_navigation_failure=recoverable,
              common_retry_exhausted=isinstance(e,RetryExhausted),**evidence)
        print('전체 이동 중단: '+str(e),flush=True);return 130 if isinstance(e,KeyboardInterrupt) else 1
if __name__=='__main__':raise SystemExit(main())
