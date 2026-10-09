#!/usr/bin/env python3
# ========================================================================
# 역할: burger1 REST(대기장소) 대기 작업자. rest_forward 코드를 미리 import 하고 odom·통신 감시만 유지한다.
#       대기 중에는 GPIO·속도 발행자를 잡지 않고, 요청이 오면 그때 만든다.
# 실행: burger1-rest-ready.service → start_rest_ready.sh → rest_ready_worker.py (ensure_rest_ready.sh 가 켬)
# 호출 관계: run_rest.sh → waypoint_client.py → 소켓 → 여기 → rest_forward.main(prepared=...).
# ========================================================================
"""Burger1 REST standby: feedback/graph only, GPIO and command publisher per execution."""
import hashlib,importlib.util,json,os,signal,socket,threading,time
from pathlib import Path
from types import SimpleNamespace
from waypoint_worker import forwarded_output,monitor_disconnect


# 파일 SHA-256 해시.
def signature(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# rest 코드·설정이 바뀌었으면 예외 → 서비스가 재시작되며 새 설정을 읽는다.
def check_sources(state):
    if signature(state['entry'])!=state['sha256'] or signature(state['config'])!=state['config_sha256']:
        raise RuntimeError('REST code/config changed; restarting idle standby')

# 요청 검사 (2초 이내, run/probe, 이 로봇 설정 경로).
def validate(request, now, config):
    issued=request.get('issued')
    if isinstance(issued,bool) or not isinstance(issued,(int,float)) or not 0<=now-issued<=2:
        raise ValueError('Expired request; no queued REST movement')
    if request.get('op') not in ('run','probe'):raise ValueError('Unknown REST operation')
    if request['op']=='run' and request.get('argv')!=['--config',str(config)]:
        raise ValueError('REST worker accepts only its Burger1 configuration')


# 요청 1건 실행 (출력 전달, 연결 감시, 결과 JSON).
def execute(module, prepared, request, connection):
    finished=threading.Event();lock=threading.Lock()
    connection.sendall(b'{"type":"accepted"}\n')
    watcher=threading.Thread(target=monitor_disconnect,args=(connection,finished,
        lambda:os.kill(os.getpid(),signal.SIGUSR1)),daemon=True);watcher.start()
    try:
        with forwarded_output(connection,lock):
            try:return module.main(argv=request['argv'],prepared=prepared)
            except KeyboardInterrupt:
                print(json.dumps({'state':'FAULT','reason':'operator_or_client_interrupted','stopped':False}),flush=True)
                return 130
            except Exception as exc:
                print(json.dumps({'state':'FAULT','reason':str(exc),'stopped':False}),flush=True)
                return 1
    finally:finished.set();watcher.join(timeout=.2)


# 설정·코드 로드 → odom·모드 구독 → 소켓 대기 루프 (0.5초마다 코드 변경 확인).
def main():
    started=time.monotonic();here=Path(__file__).absolute().parent
    entry=here/'rest_forward.py';config=Path((os.environ['AMR_CAMERA'] + '/docking.yaml'))
    import yaml
    cfg=yaml.safe_load(config.read_text())
    if cfg['cmd_topic']!='/burger1/cmd_vel_direct' or cfg['odom_topic']!='/burger1/odom':
        raise ValueError('REST standby belongs to Burger1 direct control only')
    spec=importlib.util.spec_from_file_location('burger1_rest_program',entry)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.settings(cfg)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from communication_guard import GraphGuard
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('rest_forward',namespace='/burger1')
    prepared=SimpleNamespace(node=node,on_odom=None,guard=None)
    owner={'mode':None,'received':0.};odom={'received':0.,'stamp':0.}
    # odom 수신: 시각 기록, 실행 중이면 rest 코드로 전달.
    def receive(msg):
        odom.update(received=time.monotonic(),stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        if prepared.on_odom is not None:prepared.on_odom(msg)
    node.create_subscription(Odometry,cfg['odom_topic'],receive,qos_profile_sensor_data)
    node.create_subscription(String,'motion_owner/status',lambda msg:owner.update(
        mode=msg.data,received=time.monotonic()),qos_profile_sensor_data)
    # Graph reader is retained; no GPIO claim and NO velocity publisher in standby.
    prepared.guard=GraphGuard(cfg['cmd_topic'],node.get_name(),node.get_namespace(),'rest')
    runtime=Path(os.environ.get('XDG_RUNTIME_DIR','/run/user/'+str(os.getuid())))
    address=runtime/'burger1-rest-ready.sock';meta=address.with_suffix('.json')
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    stopping=False;executing=False
    # 종료 신호 처리.
    def stop(*_):
        nonlocal stopping
        stopping=True
        if executing:raise KeyboardInterrupt
    # 연결 끊김 신호: 실행 중이면 중단.
    def interrupt(*_):
        if executing:raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGUSR1,interrupt)
    state={'pid':os.getpid(),'boot':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
           'entry':str(entry),'sha256':signature(entry),'config':str(config),'config_sha256':signature(config),
           'startup_s':time.monotonic()-started}
    try:
        address.unlink(missing_ok=True);server.bind(str(address));os.chmod(address,0o600)
        server.listen(1);server.setblocking(False);meta.write_text(json.dumps(state))
        print('REST standby prepared: no GPIO or command publisher',flush=True)
        next_source_check=0.
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node,timeout_sec=.05)
            # Execution is synchronous below: reload only between requests.
            # Raising here runs cleanup and Restart=on-failure reloads settings.
            if time.monotonic() >= next_source_check:
                check_sources(state)
                next_source_check=time.monotonic()+.5
            try:connection,_=server.accept()
            except BlockingIOError:continue
            with connection:
                connection.settimeout(2)
                try:
                    request=json.loads(connection.makefile('rb').readline(16384))
                    validate(request,time.monotonic(),config)
                    check_sources(state)
                    if request['op']=='probe':
                        reply={'type':'result','code':0,'owner':owner,'odom':odom,
                               'command_publishers':len(node.get_publishers_info_by_topic(cfg['cmd_topic'])),
                               'gpio_claimed':False,'motion_sent':False}
                    else:
                        end=time.monotonic()+1
                        while (owner['mode']!='direct' or time.monotonic()-owner['received']>.8) and time.monotonic()<end:
                            rclpy.spin_once(node,timeout_sec=.02)
                        if owner['mode']!='direct' or time.monotonic()-owner['received']>.8:
                            raise RuntimeError('Fresh direct ownership required')
                        connection.settimeout(5);executing=True
                        code=execute(module,prepared,request,connection);executing=False
                        reply={'type':'result','code':int(code or 0)}
                    connection.sendall((json.dumps(reply)+'\n').encode())
                except BaseException as exc:
                    executing=False
                    print('REST request rejected/ended: '+str(exc),flush=True)
                    try:connection.sendall((json.dumps({'type':'result','code':1,'error':str(exc)})+'\n').encode())
                    except OSError:pass
    finally:
        prepared.on_odom=None;prepared.guard.close();server.close()
        meta.unlink(missing_ok=True);address.unlink(missing_ok=True)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
