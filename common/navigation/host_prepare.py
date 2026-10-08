#!/usr/bin/env python3
"""Persistent host readiness worker; reconnect by request ID, never queue movement."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import time
from startup_state import Feedback, Journal, lifecycle_evidence_valid, apply_lifecycle_reply


def socket_path(robot):
    return Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/(robot+'-host-ready.sock')


def close_readiness(stop, thread, executor, listener, node, shutdown):
    # Stop and join the executor before destroying anything that can wake it.
    stop.set()
    thread.join()
    executor.remove_node(node)
    listener.unregister()
    node.destroy_node()
    executor.shutdown()
    shutdown()


def wait_current_health(check, timeout=45., clock=time.monotonic, sleep=time.sleep):
    """Allow a newly started observer to receive evidence from healthy services."""
    deadline = clock() + timeout
    while True:
        if check():
            return True
        if clock() >= deadline:
            return False
        sleep(.05)


def request(robot, identity, parked=False):
    here = Path(__file__).absolute().parent
    subprocess.run(['systemctl','--user','start',robot+'-host-ready.service'],check=True)
    deadline=time.monotonic()+12
    while True:
        connection=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        try: connection.connect(str(socket_path(robot))); break
        except (FileNotFoundError,ConnectionRefusedError):
            connection.close()
            if time.monotonic()>=deadline:raise RuntimeError('Preparation worker socket not ready')
            time.sleep(.05)
    with connection:
        connection.settimeout(1.)
        connection.sendall((json.dumps(dict(request_id=identity,parked=parked))+'\n').encode())
        raw=b''
        end=time.monotonic()+300;last_progress=None
        while b'\n' not in raw and len(raw)<65536:
            try:chunk=connection.recv(4096)
            except socket.timeout:
                if time.monotonic()>=end:raise RuntimeError('Preparation deadline exceeded; query the same request ID')
                try:
                    records=json.loads((here.parents[2]/'data'/robot/'host_prepare_requests.json').read_text())
                    phase=records.get(identity,{}).get('stage','worker_wait')
                    if phase!=last_progress:print(robot+' 준비 단계: '+phase,flush=True);last_progress=phase
                except (OSError,ValueError):pass
                continue
            if not chunk:raise RuntimeError('Preparation connection lost; reconnect with the same request ID')
            raw+=chunk
        reply=json.loads(raw)
    print(json.dumps(reply,ensure_ascii=False),flush=True)
    if not reply.get('success'):raise RuntimeError(reply.get('message','Preparation failed'))


def serve(robot):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from lifecycle_msgs.srv import GetState
    from lifecycle_msgs.msg import TransitionEvent
    from std_srvs.srv import SetBool
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from turtlebot3_msgs.msg import SensorState
    from tf2_ros import Buffer,TransformListener
    from host_pkg.action import Burger
    from operation import Operation

    here=Path(__file__).absolute().parent;op=Operation(here)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    journal=Journal(op.data/'host_prepare_requests.json',boot)
    rclpy.init(args=['--ros-args','-r','/tf:=/'+robot+'/tf','-r','/tf_static:=/'+robot+'/tf_static'])
    node=rclpy.create_node('host_readiness_worker',namespace='/'+robot)
    executor=SingleThreadedExecutor();executor.add_node(node)
    buf=Buffer();listener=TransformListener(buf,node,spin_thread=False)
    feedback=Feedback();lock=threading.Lock();scan={};states={};clients={};pending={}
    action=ActionClient(node,Burger,'/M'+robot[-1]+'/data')
    power=node.create_client(SetBool,'motor_power')
    def age(stamp):return (node.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
    def odom(msg):
        with lock:feedback.odometry(age(msg.header.stamp),msg.twist.twist.linear.x,msg.twist.twist.angular.z)
    def sensor(msg):
        with lock:feedback.motor(msg.torque,age(msg.header.stamp))
    def laser(msg):
        with lock:scan.update(at=time.monotonic(),age=age(msg.header.stamp))
    latest_sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs=[node.create_subscription(Odometry,'odom',odom,latest_sensor),
          node.create_subscription(SensorState,'sensor_state',sensor,latest_sensor),
          node.create_subscription(LaserScan,'scan',laser,latest_sensor)]
    for name in ('amcl','bt_navigator'):clients[name]=node.create_client(GetState,name+'/get_state')
    due={name:0. for name in clients}
    transitions={}
    def on_transition(name):
        def receive(msg):
            now=time.monotonic()
            with lock:
                transitions[name]=now
                states[name]=(msg.goal_state.id,now)
        return receive
    for name in clients:
        subs.append(node.create_subscription(TransitionEvent,name+'/transition_event',on_transition(name),1))
    def poll_states():
        now=time.monotonic()
        for name,client in clients.items():
            if name in pending:
                future,sent=pending[name]
                if future.done():
                    try:
                        result=future.result()
                        with lock:apply_lifecycle_reply(states,transitions,name,result.current_state.id if result else None,sent,now)
                    except Exception:
                        with lock:apply_lifecycle_reply(states,transitions,name,None,sent,now)
                    del pending[name];due[name]=now+1.
                elif now-sent>12.:
                    client.remove_pending_request(future);future.cancel();del pending[name]
                    # Retain the last observation as bounded evidence. It cannot
                    # pass after expiry or without a live pending query.
                    due[name]=now+.5
            elif now>=due[name] and client.service_is_ready():
                pending[name]=(client.call_async(GetState.Request()),now)
    node.create_timer(.2,poll_states)
    spin_stop=threading.Event()
    def spin_readiness():
        while not spin_stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    thread=threading.Thread(target=spin_readiness,daemon=True);thread.start()

    def proof(navigation=True):
        now=time.monotonic()
        with lock:
            checks={'fresh_stopped_odom':feedback.stopped(),'motor_torque':feedback.torque(),
                    'scan':bool(scan) and now-scan['at']<1 and -.1<=scan['age']<=1}
            for name in ('amcl','bt_navigator') if navigation else ('amcl',):
                state=states.get(name);inflight=pending.get(name)
                checks[name]=lifecycle_evidence_valid(state,inflight[1] if inflight else None,now)
        try:
            t=buf.lookup_transform(robot+'/map',robot+'/base_footprint',rclpy.time.Time())
            checks['map_tf']= -.8<=age(t.header.stamp)<=.8
        except Exception:checks['map_tf']=False
        return checks

    def wait_proof(timeout=45):
        deadline=time.monotonic()+timeout
        while True:
            checks=proof()
            if all(checks.values()):return checks
            if time.monotonic()>=deadline:raise RuntimeError('Readiness feedback missing: '+', '.join(k for k,v in checks.items() if not v))
            time.sleep(.05)

    def ensure_motor():
        deadline=time.monotonic()+45
        while True:
            with lock:sample=feedback.sensor
            if sample and time.monotonic()-sample[0]<=.5 and -.1<=sample[2]<=.5:break
            if time.monotonic()>=deadline:raise RuntimeError('No fresh sensor_state; motor enable refused')
            time.sleep(.05)
        if sample[1]:return
        while not power.service_is_ready():
            if time.monotonic()>=deadline:raise RuntimeError('motor_power service discovery timeout')
            time.sleep(.05)
        command=SetBool.Request();command.data=True
        result=future_result(power.call_async(command),10,'motor power')
        if not result.success:raise RuntimeError('motor_power enable failed')
        deadline=time.monotonic()+8
        while True:
            with lock:ready=feedback.torque()
            if ready:return
            if time.monotonic()>=deadline:raise RuntimeError('Fresh torque=True not confirmed')
            time.sleep(.05)

    def units_ready():
        units=[robot+'-'+s+'.service' for s in ('base','localization','nav2','camera','docking-ready','rest-ready','nav-control')]
        result=subprocess.run(['systemctl','--user','is-active',*units],capture_output=True,text=True)
        return result.stdout.splitlines()==['active']*len(units)

    def future_result(future,timeout,label):
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>=deadline:raise RuntimeError(label+' response timeout; do not repeat without the request ID')
            time.sleep(.02)
        result=future.result()
        if result is None:raise RuntimeError(label+' empty response')
        return result

    def prepare(identity,parked):
        started=time.monotonic();old=journal.get(identity)
        if old and old.get('parked')!=parked:raise RuntimeError('Request ID reused with different parking option')
        if old and old.get('boot')!=boot:raise RuntimeError('Robot rebooted; use a new host session')
        if old and old.get('status')=='failed':return dict(old,duplicate=True)
        if old and old.get('boot')==boot and old.get('status')=='success':
            with op.lock():
                if op.busy():raise RuntimeError('Robot is moving; no preparation during a mission')
            if op.mode()=='process' and action.server_is_ready() and all(proof().values()) and units_ready():
                return dict(old,duplicate=True,seconds=round(time.monotonic()-started,3))
            raise RuntimeError('Saved preparation result is no longer current; start a new host session')
        if old and old.get('status')=='running':
            raise RuntimeError('Preparation was interrupted; do not blindly repeat RESTART. Start a new host session while stopped')
        journal.put(identity,status='running',stage='admission',parked=parked,success=False)
        timings={}
        def stage(name,fn):
            begin=time.monotonic();journal.put(identity,status='running',stage=name,parked=parked)
            print(robot+' prepare '+name,flush=True)
            result=fn();timings[name]=round(time.monotonic()-begin,3);return result
        try:
            from action_gate import load_gate
            gate = load_gate(op.data/'action_gate.json')
            with op.lock():
                reuse_candidate = (not parked and not op.busy() and op.mode() == 'process'
                                   and gate is not None and not gate.get('estop', True)
                                   and units_ready())
            # A new ROS observer initially has an empty TF/sensor/lifecycle cache.
            # Give existing services a short chance to supply current evidence;
            # an empty observer cache must not trigger RESTART of a healthy robot.
            healthy = reuse_candidate and wait_current_health(
                lambda: action.server_is_ready() and all(proof().values()))
            if healthy:
                with op.lock():
                    if op.busy() or op.mode() != 'process':
                        raise RuntimeError('Robot state changed during readiness confirmation')
                gate = load_gate(op.data/'action_gate.json')
                if gate is None or gate.get('estop', True):
                    raise RuntimeError('Action gate changed during readiness confirmation')
                return journal.put(identity,status='success',stage='ready',success=True,parked=parked,
                                   seconds=round(time.monotonic()-started,3),timings=timings,
                                   movement_sent=False,reused_running=True)
            def admit():
                with op.lock():op.set_mode('individual')
            stage('admission',admit)
            if parked:
                stage('parking_seed',op.parked)
            # No old boolean files qualify as readiness. A warm path requires
            # current lifecycle replies AND current sensor/TF evidence.
            warm=units_ready() and all(proof().values())
            if not warm:stage('services',op.ready)
            else:
                # Keep the original idle ownership and old-goal cancellation.
                stage('nav_handoff',lambda:subprocess.run(['/bin/bash',str(here/'set_mode.sh'),'prepare'],check=True,timeout=160))
            stage('motor',ensure_motor)
            stage('feedback',wait_proof)
            stage('action_server',lambda:subprocess.run(['systemctl','--user','start','m'+robot[-1]+'-action.service'],check=True))
            deadline=time.monotonic()+20
            while not action.server_is_ready():
                if time.monotonic()>=deadline:raise RuntimeError('Action discovery timeout: /M'+robot[-1]+'/data')
                time.sleep(.05)
            # Require samples received after Action discovery, not pre-start history.
            with lock: feedback.odom.clear()
            stage('stationary',wait_proof)
            with op.lock():
                if op.busy():raise RuntimeError('Robot became busy during preparation')
            # The Action backend takes the same operation lock. Do not hold it
            # while waiting for its ACK. Admission remains individual mode.
            goal=Burger.Goal();goal.command='RESTART';goal.cmd_val=0.
            journal.put(identity,status='running',stage='restart_sent',parked=parked)
            handle=stage('restart_accept',lambda:future_result(action.send_goal_async(goal),15,'RESTART acceptance'))
            if not handle.accepted:raise RuntimeError('RESTART rejected')
            result=stage('restart_result',lambda:future_result(handle.get_result_async(),30,'RESTART result')).result
            if not result.success:raise RuntimeError('RESTART failed: '+result.message)
            journal.put(identity,status='running',stage='restart_acknowledged',parked=parked)
            with op.lock():
                op.set_mode('process')
            return journal.put(identity,status='success',stage='ready',success=True,parked=parked,
                               seconds=round(time.monotonic()-started,3),timings=timings,movement_sent=False)
        except Exception as exc:
            journal.put(identity,status='failed',stage='failed',success=False,message=str(exc),
                        seconds=round(time.monotonic()-started,3),timings=timings,parked=parked)
            raise

    path=socket_path(robot);path.unlink(missing_ok=True)
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(path));os.chmod(path,0o600);server.listen(4);server.settimeout(120.)
    print('Host readiness worker: '+str(path),flush=True)
    try:
        while rclpy.ok():
            try: connection,_=server.accept()
            except socket.timeout: break  # Idle startup worker exits; retained services keep running.
            with connection:
                connection.settimeout(2.)
                try:
                    raw=b''
                    while b'\n' not in raw and len(raw)<4096:
                        part=connection.recv(4096)
                        if not part:raise RuntimeError('Empty request')
                        raw+=part
                    command=json.loads(raw);parked=command.get('parked',False)
                    if type(parked) is not bool:raise ValueError('Invalid parking flag')
                    reply=prepare(command['request_id'],parked)
                except Exception as exc:reply=dict(success=False,message=str(exc))
                try:connection.sendall((json.dumps(reply,ensure_ascii=False)+'\n').encode())
                except OSError:pass  # The durable result remains available by ID.
    finally:
        server.close();path.unlink(missing_ok=True)
        close_readiness(spin_stop,thread,executor,listener,node,rclpy.shutdown)


def main():
    p=argparse.ArgumentParser();p.add_argument('robot',choices=['burger1','burger2']);p.add_argument('--serve',action='store_true')
    p.add_argument('--request-id');p.add_argument('--parked',action='store_true');a=p.parse_args()
    if a.serve:serve(a.robot)
    elif a.request_id:request(a.robot,a.request_id,a.parked)
    else:p.error('--serve or --request-id required')
if __name__=='__main__':main()
