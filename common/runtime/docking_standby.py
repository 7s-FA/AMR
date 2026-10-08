#!/usr/bin/env python3
"""Persistent, robot-scoped vision for normal docking and parking; no idle motor/GPIO ownership."""
import argparse, hashlib, json, os, select, signal, socket, time, queue
from pathlib import Path


def signature(cfg_path, configurations):
    paths = {Path(cfg_path)}
    for cfg in configurations.values():
        paths.update((Path(cfg['board_path']), Path(cfg['calibration_path'])))
    paths.update(Path(cfg_path).parent / name for name in
                 ('docking_node.py', 'docking_control.py', 'docking_vision_worker.py',
                  'communication_guard.py', 'camera_ipc.py', 'video_http.py'))
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def validate_request(req, now):
    if req.get('action') not in ('start', 'prepare', 'profile') or not isinstance(req.get('issued'), (int, float)):
        raise ValueError('Invalid docking request')
    if not 0 <= now-req['issued'] <= 2:
        raise ValueError('Expired docking request; not executed')


def disconnected(conn):
    readable, _, _ = select.select([conn], [], [], 0)
    return bool(readable) and conn.recv(1, socket.MSG_PEEK) == b''


def wait_fresh(vision, cfg, timeout=5.0):
    # A pre-switch detection can never satisfy readiness for the new board.
    vision.set_active(False)
    vision.set_mode(cfg['docking_mode'])
    activated_wall = time.time()
    vision.set_active(True)
    fresh, last_sequence = 0, -1
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline and fresh < 3:
        try: observation = vision.channel.get(timeout=.1)
        except queue.Empty: continue
        stamp = observation.get('source_stamp', {})
        captured = stamp.get('sec', 0)+stamp.get('nanosec', 0)/1e9
        age = time.time()-captured
        sequence = observation.get('sequence', -1)
        if (observation.get('status') == 'ok' and captured >= activated_wall and 0 <= age <= .25
                and sequence > last_sequence and observation.get('board_spec') == cfg['board_spec']
                and observation.get('vision_mode') == cfg['docking_mode']):
            last_sequence = sequence; fresh += 1
        else: fresh = 0
    if fresh < 3:
        raise RuntimeError('High-rate camera detections not ready; docking not started')
    return {'fresh_frames': fresh, 'last_sequence': last_sequence,
            'mode': cfg['docking_mode'], 'target_ids': cfg['target_ids']}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config', required=True)
    p.add_argument('--robot', choices=('burger1', 'burger2'), required=True)
    p.add_argument('--calibration');a=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from camera_ipc import profile as native_camera_profile
    from docking_vision.docking_config import load_docking_config
    from docking_vision_worker import LocalVision
    from docking_node import DockingNode, Settings
    from ir_sensor import GPIOInput
    from communication_guard import GraphGuard
    configurations = {mode: load_docking_config(a.config, mode, calibration_path=a.calibration)
                      for mode in ('normal', 'parking')}
    for cfg in configurations.values():
        Settings(**cfg['control'])
        if cfg['cmd_topic'] != '/'+a.robot+'/cmd_vel_direct':
            raise ValueError('Docking target belongs to another robot')
    sig = signature(a.config, configurations)
    path = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/'+str(os.getuid()))) / (a.robot+'-docking-ready.sock')
    manifest=path.with_suffix('.json');server=vision=node=gpio=probe=guard=None
    def interrupted(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted);signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    owner={'mode': None, 'at': 0.}
    try:
        probe=Node('docking_standby_probe', namespace='/'+a.robot)
        def mode(m): owner.update(mode=m.data, at=time.monotonic())
        probe.create_subscription(String, 'motion_owner/status', mode, 1)
        def profile(value):
            if value not in ('active', 'idle'): raise ValueError('Invalid camera profile')
            report=native_camera_profile(a.robot,value)
            expected=500000 if value=='idle' else 66667
            if report['frame_duration_us']!=expected:raise RuntimeError('Native camera profile rejected')
            return dict(report,restart=False)
        guard=GraphGuard(configurations['normal']['cmd_topic'], 'docking_controller', '/'+a.robot, 'docking')
        vision=LocalVision(configurations['normal'], modes=configurations);vision.set_active(False)
        server=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        path.unlink(missing_ok=True);server.bind(str(path));os.chmod(path, 0o600)
        server.listen(1);server.setblocking(False)
        manifest.write_text(json.dumps({'signature': sig, 'pid': os.getpid(),
                                       'available_modes': list(configurations)}))
        print('Docking standby: normal + parking vision warming; NO motor/GPIO ownership', flush=True)
        while rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=.01)
            try: conn, _=server.accept()
            except BlockingIOError: continue
            with conn:
                recorder=None;exit_reason='startup_failed';selected=None
                conn.settimeout(1.)
                try:
                    req=json.loads(conn.makefile('rb').readline(4096))
                    if req.get('action') == 'status':
                        reply={'code': 0, 'health': vision.health(), 'owner': owner['mode'],
                               'motor_publishers': probe.count_publishers('/'+a.robot+'/cmd_vel'),
                               'direct_publishers': probe.count_publishers(configurations['normal']['cmd_topic']),
                               'graph_guard_alive': guard.process.is_alive(), 'graph_monitor': guard.snapshot()}
                        conn.sendall((json.dumps(reply)+'\n').encode());continue
                    validate_request(req, time.monotonic())
                    if signature(a.config, configurations) != sig:
                        raise RuntimeError('Docking config changed; restart standby before docking')
                    if req['action'] == 'profile':
                        report=profile(req.get('profile'))
                        conn.sendall((json.dumps({'code': 0, 'report': report})+'\n').encode());continue
                    selected=req.get('mode', 'normal')
                    if selected not in configurations: raise ValueError('Mode was not preloaded')
                    cfg=configurations[selected]
                    if req['action'] == 'prepare':
                        report=wait_fresh(vision, cfg)
                        report['motor_owned']=False
                        conn.sendall((json.dumps({'code': 0, 'report': report})+'\n').encode());continue
                    # Only terminal_wrapper can grant direct ownership.
                    refresh=time.monotonic()+.15
                    while time.monotonic() < refresh: rclpy.spin_once(probe, timeout_sec=.01)
                    deadline=time.monotonic()+1.
                    while time.monotonic() < deadline and (owner['mode'] != 'direct' or time.monotonic()-owner['at'] > .5):
                        rclpy.spin_once(probe, timeout_sec=.02)
                    if owner['mode'] != 'direct' or time.monotonic()-owner['at'] > .5:
                        raise RuntimeError('Direct motor ownership is not confirmed')
                    if probe.count_publishers(cfg['cmd_topic']):
                        raise RuntimeError('Docking input already has a publisher')
                    prepared=wait_fresh(vision, cfg)
                    refresh=time.monotonic()+.15
                    while time.monotonic() < refresh: rclpy.spin_once(probe, timeout_sec=.01)
                    if owner['mode'] != 'direct' or time.monotonic()-owner['at'] > .5:
                        raise RuntimeError('Direct motor ownership changed during camera readiness')
                    print('Fresh mode-specific camera detections confirmed: '+json.dumps(prepared), flush=True)
                    if req.get('log_dir'):
                        from docking_recorder import DockingRecorder
                        recorder=DockingRecorder(req['log_dir'], cfg, [a.config, cfg['calibration_path'], cfg['board_path'],
                            Path(a.config).with_name('docking_node.py'), Path(a.config).with_name('docking_control.py'),
                            Path(a.config).with_name('docking_vision_worker.py'), Path(a.config).with_name('communication_guard.py')])
                    gpio=GPIOInput(cfg['gpio_chip'], cfg['gpio_pin'])
                    node=DockingNode(cfg, gpio, True, True, vision.channel, recorder=recorder, graph_guard=guard)
                    node.vision_health=vision.health
                    print('Warm '+selected+' docking start accepted', flush=True)
                    conn.setblocking(False)
                    while rclpy.ok():
                        rclpy.spin_once(node, timeout_sec=.01)
                        if disconnected(conn): node.control.halt('docking_client_disconnected');break
                        if node.control.state in ('DOCKED', 'STOPPED', 'FAULT'): break
                    code=0 if node.control.state == 'DOCKED' else 1
                    report={'state': node.control.state, 'reason': node.control.reason,
                            'docking_encoder': node.encoder_retry_report(),
                            'warm_start': True, 'docking_mode': selected, 'target_ids': cfg['target_ids']}
                    exit_reason=node.control.state+': '+node.control.reason
                    conn.settimeout(1.);conn.sendall((json.dumps({'code': code, 'report': report})+'\n').encode())
                except (Exception, KeyboardInterrupt) as exc:
                    exit_reason=type(exc).__name__+': '+str(exc)
                    print('Warm docking failed: '+str(exc), flush=True)
                    try: conn.settimeout(1.);conn.sendall((json.dumps({'code': 1, 'error': str(exc)})+'\n').encode())
                    except OSError: pass
                    if isinstance(exc, KeyboardInterrupt): raise
                finally:
                    vision.set_active(False)
                    if node is not None:
                        for _ in range(3): node.publish_velocity(0., 0.);time.sleep(.02)
                        node.destroy_node();node=None
                    if gpio is not None: gpio.close();gpio=None
                    if recorder is not None: recorder.close(exit_reason)
    except KeyboardInterrupt: pass
    finally:
        if guard is not None: guard.close()
        if vision is not None: vision.close()
        if probe is not None: probe.destroy_node()
        if server is not None: server.close()
        path.unlink(missing_ok=True);manifest.unlink(missing_ok=True)
        rclpy.try_shutdown()

if __name__ == '__main__': main()
