"""Call the robot's existing admission point; no SSH or host dependency."""
import json,math,subprocess,sys,time
from pathlib import Path

ROUTES={'GO_TO_MAT':'mat','GO_TO_ASM':'asm','GO_TO_REST':'rest','GO_TO_PARK':'park'}
CONTROLS={'EMER_STOP','RESTART'}
def validate(command,speed):
    if command not in ROUTES and command not in CONTROLS:raise ValueError('UNKNOWN_COMMAND')
    if not math.isfinite(speed) or not 0<=speed<=100:raise ValueError('SPEED_OUT_OF_RANGE')
    if command in ROUTES and speed<=0:raise ValueError('ZERO_SPEED_CANNOT_COMPLETE_ROUTE')

class Backend:
    def __init__(self,nav):
        self.nav=Path(nav).resolve();sys.path.insert(0,str(self.nav))
        from operation import Operation, atomic_json
        from action_gate import load_gate
        self.atomic=atomic_json;self.load_gate=load_gate
        self.op=Operation(self.nav,run=self.run)
        self.path=self.op.data/'action_gate.json'
        profile=self.nav/'action_profile.json'
        self.profile=json.loads(profile.read_text()) if profile.exists() else {'max_linear_mps':.066,'max_angular_rps':.924}
        if not (0<float(self.profile['max_linear_mps'])<=.08 and 0<float(self.profile['max_angular_rps'])<=1.):raise ValueError('Invalid action speed ceilings')
        # A restarted action process must never re-authorize a previous action.
        with self.op.lock():
            previous=self.load_gate(self.path)
            if previous and previous.get('goal_id'):
                self.atomic(self.path,dict(previous,estop=True,reason='action_server_restarted'))
    @staticmethod
    def run(*args,**kwargs):
        kwargs.setdefault('timeout',25)
        return subprocess.run(*args,**kwargs)
    def preflight(self):
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and g.get('estop',True):raise RuntimeError('ESTOP_LATCHED')
            if self.op.mode()!='process':raise RuntimeError('PROCESS_MODE_REQUIRED')
            if self.op.busy():raise RuntimeError('ROBOT_BUSY')
    def submit(self,command,speed,goal_id):
        validate(command,speed)
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and g.get('estop',True):raise RuntimeError('ESTOP_LATCHED')
            if self.op.mode()!='process' or self.op.busy():raise RuntimeError('ROBOT_BUSY_OR_MODE_CHANGED')
            self.atomic(self.path,{'estop':False,'goal_id':goal_id,'speed_percent':speed,'heartbeat':time.monotonic(),'max_linear_mps':self.profile['max_linear_mps'],'max_angular_rps':self.profile['max_angular_rps']})
            try:return self.op.submit(ROUTES[command],source='host',task_id=goal_id)
            except Exception:
                self.atomic(self.path,{'estop':True,'reason':'submit_failed'});raise
    def authorized(self,goal_id):
        with self.op.lock():
            return self._authorized(self.load_gate(self.path),goal_id)
    @staticmethod
    def _authorized(g,goal_id):
        return bool(g and not g.get('estop',True) and g.get('goal_id')==goal_id)
    def renew(self,goal_id):
        with self.op.lock():
            g=self.load_gate(self.path)
            if not self._authorized(g,goal_id):return False
            g['heartbeat']=time.monotonic();self.atomic(self.path,g);return True
    def result(self,goal_id):
        # A failed service or unfinished record is handled by Operation.current().
        with self.op.lock():
            self.op.current()
            return self.op.read('commands/'+goal_id+'.json')
    def stop(self,reason='emergency_stop'):
        # Write zero-output latch before potentially slow systemd stop calls.
        with self.op.lock():self.atomic(self.path,{'estop':True,'reason':reason})
        return self.op.stop()
    def restart(self):
        with self.op.lock():
            if self.op.busy():raise RuntimeError('ROBOT_BUSY')
            self.atomic(self.path,{'estop':False,'goal_id':None,'reason':'released_no_resume'})
        return True
    def finish(self,goal_id,success):
        with self.op.lock():
            g=self.load_gate(self.path)
            if g and not g.get('estop',True) and g.get('goal_id')==goal_id:
                self.atomic(self.path,{'estop':not success,'goal_id':None,'reason':'complete' if success else 'mission_failed'})
