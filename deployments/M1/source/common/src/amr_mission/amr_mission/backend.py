# ========================================================================
# 역할: 액션 서버와 로봇 내부 임무 시스템(operation.py) 사이의 연결부.
#       정지 래치·속도 상한 파일(data/<로봇>/action_gate.json)을 쓰고 읽어 모터 출력을 통제한다.
# 실행: action_server.py 가 import 해서 사용 (단독 실행 안 함).
# 호출 관계: operation.Operation(임무 접수), action_gate.load_gate(래치 읽기). motion_owner.py 가 같은 래치 파일을 보고 속도를 제한한다.
# ========================================================================
"""Call the robot's existing admission point; no SSH or host dependency."""
import json,math,subprocess,sys,time
from pathlib import Path

ROUTES={'GO_TO_MAT':'mat','GO_TO_ASM':'asm','GO_TO_REST':'rest','GO_TO_PARK':'park'}
# RESET_PARKED: 사람이 로봇을 자기 주차 자리에 옮겨 놓았을 때 관제에서 보낸다 (주차 위치로 관제 준비, 2026-10-11 추가).
CONTROLS={'EMER_STOP','RESTART','RESET_PARKED'}
# 관제 명령 이름과 속도(0~100%)를 검사. 이동 명령은 속도 0을 허용하지 않는다.
def validate(command,speed):
    if command not in ROUTES and command not in CONTROLS:raise ValueError('UNKNOWN_COMMAND')
    if not math.isfinite(speed) or not 0<=speed<=100:raise ValueError('SPEED_OUT_OF_RANGE')
    if command in ROUTES and speed<=0:raise ValueError('ZERO_SPEED_CANNOT_COMPLETE_ROUTE')

# action_gate.json 래치와 Operation 을 묶어 액션 서버에 간단한 함수(submit/stop/restart/...)로 제공한다.
class Backend:
    # navigation 폴더 기준으로 Operation 준비, 속도 상한 읽기. 서버 재시작 시 이전 목표 권한만 지운다(정지 래치는 그대로).
    def __init__(self,nav):
        self.nav=Path(nav).absolute();sys.path.insert(0,str(self.nav))
        from operation import Operation, atomic_json
        from action_gate import load_gate
        self.atomic=atomic_json;self.load_gate=load_gate
        self.op=Operation(self.nav,run=self.run)
        self.path=self.op.data/'action_gate.json'
        self.service_checked_at=float('-inf')
        profile=self.nav/'action_profile.json'
        self.profile=json.loads(profile.read_text()) if profile.exists() else {'max_linear_mps':.066,'max_angular_rps':.924}
        if not (0<float(self.profile['max_linear_mps'])<=.08 and 0<float(self.profile['max_angular_rps'])<=1.):raise ValueError('Invalid action speed ceilings')
        # A restarted action process must never re-authorize a previous action.
        # 정지 래치(estop)는 관제의 EMER_STOP/RESTART 액션으로만 바뀐다 (2026-10-11). 여기서는 목표 권한만 지운다.
        with self.op.lock():
            previous=self.load_gate(self.path)
            if previous and previous.get('goal_id'):
                self.atomic(self.path,dict(previous,goal_id=None,reason='action_server_restarted'))
    # 외부 명령 실행 (기본 25초 제한).
    @staticmethod
    def run(*args,**kwargs):
        kwargs.setdefault('timeout',25)
        return subprocess.run(*args,**kwargs)
    # 목표 접수 전 검사: 정지 래치 없음, 공정(process) 모드, 로봇이 놀고 있음.
    def preflight(self):
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and g.get('estop',True):raise RuntimeError('ESTOP_LATCHED')
            if self.op.mode()!='process':raise RuntimeError('PROCESS_MODE_REQUIRED')
            if self.op.busy():raise RuntimeError('ROBOT_BUSY')
    # 래치 파일에 목표 ID·속도%·하트비트를 쓰고 Operation.submit 으로 임무 서비스를 시작한다.
    def submit(self,command,speed,goal_id):
        validate(command,speed)
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and g.get('estop',True):raise RuntimeError('ESTOP_LATCHED')
            if self.op.mode()!='process' or self.op.busy():raise RuntimeError('ROBOT_BUSY_OR_MODE_CHANGED')
            self.atomic(self.path,{'estop':False,'goal_id':goal_id,'speed_percent':speed,'heartbeat':time.monotonic(),'max_linear_mps':self.profile['max_linear_mps'],'max_angular_rps':self.profile['max_angular_rps']})
            try:return self.op.submit(ROUTES[command],source='host',task_id=goal_id)
            except Exception:
                self.atomic(self.path,{'estop':False,'goal_id':None,'reason':'submit_failed'});raise
    # 지금 이 목표가 래치 없이 유효한지 확인.
    def authorized(self,goal_id):
        with self.op.lock():
            return self._authorized(self.load_gate(self.path),goal_id)
    # 래치 내용(g)이 이 goal_id 의 유효한 실행 권한인지 판정.
    @staticmethod
    def _authorized(g,goal_id):
        return bool(g and not g.get('estop',True) and g.get('goal_id')==goal_id)
    # 하트비트 시각 갱신 (action_server 0.15초 타이머가 호출).
    def renew(self,goal_id):
        with self.op.lock():
            g=self.load_gate(self.path)
            if not self._authorized(g,goal_id):return False
            g['heartbeat']=time.monotonic();self.atomic(self.path,g);return True
    # 임무 결과 파일(commands/<goal_id>.json) 읽기. 서비스가 죽었는지는 1초에 한 번만 확인.
    def result(self,goal_id):
        # 결과 파일은 매번(0.2초마다) 읽는다. 이동 서비스가 기록 없이 죽었는지 보는 Operation.current()는
        # systemctl 프로세스를 새로 띄우므로 1초에 한 번만 한다 (예전: 0.2초마다 → 주행 중 초당 5회).
        with self.op.lock():
            now=time.monotonic()
            if now-self.service_checked_at>=1.:
                self.service_checked_at=now;self.op.current()
            return self.op.read('commands/'+goal_id+'.json')
    # 이동을 멈춘다. 정지 래치는 관제 EMER_STOP(reason='emergency_stop')일 때만 켠다.
    # 취소·실패·서버 종료일 때는 래치를 바꾸지 않고 목표 권한만 지운다 (모터 출력은 목표가 없으니 더 나가지 않음).
    def stop(self,reason='emergency_stop'):
        # Write the gate before potentially slow systemd stop calls.
        with self.op.lock():
            previous=self.load_gate(self.path)
            latched=bool(previous and previous.get('estop',False))
            self.atomic(self.path,{'estop':True if reason=='emergency_stop' else latched,
                                   'goal_id':None,'reason':reason})
        return self.op.stop()
    # RESTART: 이동 중이 아니면 래치 해제. 이전 임무를 다시 시작하지는 않는다.
    def restart(self):
        with self.op.lock():
            if self.op.busy():raise RuntimeError('ROBOT_BUSY')
            self.atomic(self.path,{'estop':False,'goal_id':None,'reason':'released_no_resume'})
        return True
    # RESET_PARKED: 관제 준비를 '주차 위치(--parked)'로 실행한다 (런처의 host_on.sh --recover --parked 와 같은 절차).
    # 본체·위치추정 시작 → 주차 좌표를 초기 위치로 → 도킹 중단 표시를 '주차 확인'으로 → 모터 토크 → 공정 모드.
    # 이동 명령은 보내지 않고, 정지 래치(estop)는 바꾸지 않는다. 이동 중이면 거부한다.
    def reset_parked(self,request_id):
        with self.op.lock():
            if self.op.busy():raise RuntimeError('ROBOT_BUSY')
        command=['/bin/bash','-c','source "$1/nav_env.bash"; exec /usr/bin/python3 "$1/host_prepare.py" "$2" --request-id "$3" --parked',
                 'reset_parked',str(self.nav),self.op.robot,'parked-'+request_id]
        r=subprocess.run(command,capture_output=True,text=True,timeout=330)
        if r.returncode:
            lines=(r.stdout+r.stderr).strip().splitlines()
            raise RuntimeError('PARKED_RESET_FAILED: '+(lines[-1] if lines else 'no output')[:300])
        return True
    # 임무가 끝나면 목표 ID를 비운다. 실패해도 정지 래치는 걸지 않는다 (관제가 새 명령을 바로 보낼 수 있음).
    def finish(self,goal_id,success):
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and not g.get('estop',True) and g.get('goal_id')==goal_id:
                self.atomic(self.path,{'estop':False,'goal_id':None,'reason':'complete' if success else 'mission_failed'})
