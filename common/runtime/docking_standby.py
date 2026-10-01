#!/usr/bin/env python3
"""Preload docking imports and vision only; execute on an explicit socket request."""
import argparse,hashlib,json,os,select,signal,socket,time,queue
from pathlib import Path

def signature(cfg_path, cfg):
    paths=[Path(cfg_path),Path(cfg['board_path']),Path(cfg['calibration_path'])]
    paths += [Path(cfg_path).parent / name for name in ('docking_node.py','docking_control.py','docking_vision_worker.py')]
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}

def validate_request(req, now):
    if req.get('action')!='start' or not isinstance(req.get('issued'),(int,float)):
        raise ValueError('Invalid docking start request')
    if not 0 <= now-req['issued'] <= 2:
        raise ValueError('Expired docking request; not executed')

def disconnected(conn):
    readable,_,_=select.select([conn],[],[],0)
    return bool(readable) and conn.recv(1,socket.MSG_PEEK)==b''

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--robot',required=True);p.add_argument('--calibration');a=p.parse_args()
    # Heavy imports and spawned detector are loaded BEFORE a movement request.
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from docking_vision.docking_config import load_docking_config
    from docking_vision_worker import LocalVision
    from docking_node import DockingNode,Settings
    from ir_sensor import GPIOInput
    cfg=load_docking_config(a.config,'normal',calibration_path=a.calibration);Settings(**cfg['control']);sig=signature(a.config,cfg)
    path=Path(os.environ.get('XDG_RUNTIME_DIR','/run/user/'+str(os.getuid())))/(a.robot+'-docking-ready.sock')
    manifest=path.with_suffix('.json');server=None;vision=None;node=None;gpio=None;probe=None
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    owner={'mode':None,'at':0.}
    try:
        probe=Node('docking_standby_probe',namespace='/'+a.robot)
        def mode(m):owner.update(mode=m.data,at=time.monotonic())
        probe.create_subscription(String,'motion_owner/status',mode,1)
        vision=LocalVision(cfg)
        vision.set_active(False)
        # No GPIO and no velocity publisher exist in standby.
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);path.unlink(missing_ok=True);server.bind(str(path));os.chmod(path,0o600);server.listen(1);server.setblocking(False)
        manifest.write_text(json.dumps({'signature':sig,'pid':os.getpid()}));print('Docking standby: imports and vision warming; NO motor/GPIO ownership',flush=True)
        while rclpy.ok():
            rclpy.spin_once(probe,timeout_sec=.01)
            try:conn,_=server.accept()
            except BlockingIOError:continue
            with conn:
                recorder=None;exit_reason='startup_failed'
                conn.settimeout(1.)
                try:
                    req=json.loads(conn.makefile('rb').readline(4096))
                    if req.get('action')=='status':
                        conn.sendall((json.dumps({'code':0,'health':vision.health(),'owner':owner['mode'],'motor_publishers':probe.count_publishers('/'+a.robot+'/cmd_vel'),'direct_publishers':probe.count_publishers(cfg['cmd_topic'])})+'\n').encode());continue
                    validate_request(req,time.monotonic())
                    if signature(a.config,cfg)!=sig:raise RuntimeError('Docking config changed; restart standby before docking')
                    # Only terminal_wrapper grants direct ownership; reject all other requests.
                    refresh=time.monotonic()+.15
                    while time.monotonic()<refresh:rclpy.spin_once(probe,timeout_sec=.01)
                    deadline=time.monotonic()+1.
                    while time.monotonic()<deadline and (owner['mode']!='direct' or time.monotonic()-owner['at']>.5):rclpy.spin_once(probe,timeout_sec=.02)
                    if owner['mode']!='direct' or time.monotonic()-owner['at']>.5:raise RuntimeError('Direct motor ownership is not confirmed')
                    if probe.count_publishers(cfg['cmd_topic']):raise RuntimeError('Docking input already has a publisher')
                    # High-rate camera is restored by start_docking.sh before this request.
                    # Discard standby frames; require three new, recent detections before GPIO/node creation.
                    vision.set_active(True)
                    activated_wall=time.time()
                    prepared=time.monotonic();fresh=0;last_sequence=-1;deadline=prepared+5.
                    while time.monotonic()<deadline and fresh<3:
                        try:observation=vision.channel.get(timeout=.1)
                        except queue.Empty:continue
                        stamp=observation.get('source_stamp',{})
                        age=time.time()-stamp.get('sec',0)-stamp.get('nanosec',0)/1e9
                        sequence=observation.get('sequence',-1)
                        if observation.get('status')=='ok' and age<=time.time()-activated_wall and 0<=age<=.25 and sequence>last_sequence:
                            last_sequence=sequence;fresh+=1
                        else:fresh=0
                    if fresh<3:raise RuntimeError('High-rate camera detections not ready; docking not started')
                    # Recheck ownership after camera readiness wait; no stale grant.
                    refresh=time.monotonic()+.15
                    while time.monotonic()<refresh:rclpy.spin_once(probe,timeout_sec=.01)
                    if owner['mode']!='direct' or time.monotonic()-owner['at']>.5:
                        raise RuntimeError('Direct motor ownership changed during camera readiness')
                    print('High-rate camera: three fresh detections confirmed',flush=True)
                    if req.get('log_dir'):
                        from docking_recorder import DockingRecorder
                        recorder=DockingRecorder(req['log_dir'],cfg,[a.config,cfg['calibration_path'],cfg['board_path'],Path(a.config).with_name('docking_node.py'),Path(a.config).with_name('docking_control.py'),Path(a.config).with_name('docking_vision_worker.py')])
                    gpio=GPIOInput(cfg['gpio_chip'],cfg['gpio_pin'])
                    args=(cfg,gpio,True,True,vision.channel)
                    node=DockingNode(*args,recorder=recorder) if recorder else DockingNode(*args);node.vision_health=vision.health
                    print('Warm docking start accepted',flush=True)
                    conn.setblocking(False)
                    while rclpy.ok():
                        rclpy.spin_once(node,timeout_sec=.01)
                        if disconnected(conn):
                            node.control.halt('docking_client_disconnected');break
                        if node.control.state in ('DOCKED','STOPPED','FAULT'):break
                    code=0 if node.control.state=='DOCKED' else 1
                    report={'state':node.control.state,'reason':node.control.reason,'warm_start':True}
                    exit_reason=node.control.state+': '+node.control.reason
                    conn.settimeout(1.);conn.sendall((json.dumps({'code':code,'report':report})+'\n').encode())
                except (Exception,KeyboardInterrupt) as exc:
                    exit_reason=type(exc).__name__+': '+str(exc)
                    print('Warm docking failed: '+str(exc),flush=True)
                    try:conn.settimeout(1.);conn.sendall((json.dumps({'code':1,'error':str(exc)})+'\n').encode())
                    except OSError:pass
                    if isinstance(exc,KeyboardInterrupt):raise
                finally:
                    vision.set_active(False)
                    if node is not None:
                        for _ in range(3):node.publish_velocity(0.,0.);time.sleep(.02)
                        node.destroy_node();node=None
                    if gpio is not None:gpio.close();gpio=None
                    if recorder is not None:recorder.close(exit_reason)
    except KeyboardInterrupt:pass
    finally:
        if vision is not None:vision.close()
        if probe is not None:probe.destroy_node()
        if server is not None:server.close()
        path.unlink(missing_ok=True);manifest.unlink(missing_ok=True)
        rclpy.try_shutdown()
if __name__=='__main__':main()
