#!/usr/bin/env python3
# ========================================================================
# 역할: 모터 명령(/<로봇>/cmd_vel)을 내보내는 유일한 프로그램. 지금 누가 모터를 쓸지(idle/nav/direct)를 정한다.
#       nav=Nav2 주행 명령(cmd_vel_nav_out), direct=도킹·rest 직접 명령(cmd_vel_direct). 정지 래치·속도 상한도 여기서 적용.
# 실행: <로봇>-motion-owner.service → launch_owner.sh → python3 motion_owner.py --robot burgerN
# 호출 관계: 모드 전환은 서비스 motion_owner/idle|nav|direct (motion_mode.py 가 호출).
#            상태는 motion_owner/status 로 2Hz 방송 (waypoint_worker, docking_standby, rest_ready_worker 가 확인).
# 부하: 주행 중 50Hz 출력, 대기(idle) 중에는 0 속도를 0.5초마다만 보낸다 (2026-10-09 최적화).
# ========================================================================
"""One motor publisher, exclusive command owner. No planning or speed tuning."""
import argparse,math,signal,time,json,threading
from communication_guard import GraphGuard
from pathlib import Path

# 모터 명령 주인(mode)과 마지막 명령을 들고 있는 작은 상태 기계.
class CommandOwner:
    # 처음엔 idle. timeout(0.35초) 동안 새 명령이 없으면 0 속도를 낸다.
    def __init__(self,timeout=.35):
        self.mode='idle';self.last=None;self.timeout=timeout
    # 주인 변경. 바꿀 때 이전 명령은 버린다.
    def switch(self,mode):
        if mode not in ('idle','nav','direct'):raise ValueError(mode)
        self.mode=mode;self.last=None
    # 현재 주인이 보낸 명령만 받아 저장한다 (다른 쪽 명령은 무시).
    def receive(self,source,v,w,now):
        if source!=self.mode or not all(math.isfinite(x) for x in (v,w,now)):return False
        self.last=(v,w,now);return True
    # 지금 내보낼 속도. idle 이거나 명령이 오래됐으면 0.
    def output(self,now):
        if self.mode=='idle' or self.last is None or now-self.last[2]>self.timeout:return 0.,0.
        return self.last[:2]

# 노드 생성, 서비스·구독·타이머 등록 후 종료 신호까지 돈다. 끝날 때 0 속도를 0.3초간 보내고 끝낸다.
def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',required=True);a=p.parse_args()
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from data_flow import EdgeHeartbeat
    from std_srvs.srv import Trigger
    rclpy.init(args=[]);node=rclpy.create_node('motion_owner',namespace='/'+a.robot)
    owner=CommandOwner();stopping=False
    # SIGINT/SIGTERM 을 받으면 루프를 끝내도록 표시.
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    output=node.create_publisher(TwistStamped,'cmd_vel',1)
    status=node.create_publisher(String,'motion_owner/status',1)
    communication_pub=node.create_publisher(String,'motion_owner/communication_status',1)
    from rclpy.qos import QoSProfile,DurabilityPolicy
    mission_pub=node.create_publisher(String,'mission/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    mission_file=Path(__file__).absolute().parents[3]/'data'/a.robot/'mission_state.json'
    from action_gate import gate_output
    gate_file=mission_file.with_name('action_gate.json')
    # 속도 명령 발행. 0이 아니면 action_gate.json(정지 래치·하트비트·속도%)로 제한을 적용한다.
    def send(v,w):
        # 0 속도는 정지 래치와 상관없이 0이므로 게이트 파일을 읽지 않는다 (대기 중 파일 읽기 절약).
        if v or w:v,w=gate_output(gate_file,v,w)
        m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id=a.robot+'/base_footprint';m.twist.linear.x=v;m.twist.angular.z=w;output.publish(m)
    guard=GraphGuard('/'+a.robot+'/cmd_vel', 'motion_owner', '/'+a.robot, 'motor', period=.5, max_age=1.5)
    mission_snapshot={'data': None};file_stop=threading.Event()
    # 별도 스레드: mission_state.json 을 0.5초마다 읽어 둔다 (ROS 콜백 안에서 파일을 읽지 않으려고).
    def read_mission():
        while not file_stop.is_set():
            try:mission_snapshot['data']=mission_file.read_text()
            except OSError:mission_snapshot['data']=None
            file_stop.wait(.5)
    file_worker=threading.Thread(target=read_mission,daemon=True);file_worker.start()
    # 통신 감시(GraphGuard) 결과: cmd_vel 발행자가 나 하나인지, 모터 구독자가 있는지.
    def communication_error():
        return guard.snapshot()['error']
    # 모드 전환 서비스 콜백 생성. 일단 idle·0 속도로 바꾼 뒤, 통신 정상이면 요청 모드로 전환.
    def change(mode):
        def cb(req,res):
            owner.switch('idle');send(0.,0.)
            error=communication_error()
            if mode!='idle' and error:res.success=False;res.message='모터 연결 확인 대기/오류: '+error;return res
            owner.switch(mode);status.publish(String(data=mode));res.success=True;res.message=mode;return res
        return cb
    for mode in ('idle','nav','direct'):node.create_service(Trigger,'motion_owner/'+mode,change(mode))
    # nav/direct 입력 구독 콜백 생성. 0.5초 넘은 명령은 버리고, 0 명령은 바로 내보낸다.
    def input_cb(source):
        def cb(m):
            ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            age=(node.get_clock().now().nanoseconds-ns)/1e9
            if age<-.1 or age>.5:return
            if owner.receive(source,m.twist.linear.x,m.twist.angular.z,time.monotonic()) and abs(m.twist.linear.x)+abs(m.twist.angular.z)<1e-9:send(0.,0.)
        return cb
    node.create_subscription(TwistStamped,'cmd_vel_nav_out',input_cb('nav'),1)
    node.create_subscription(TwistStamped,'cmd_vel_direct',input_cb('direct'),1)
    # 주행(nav/direct) 중에는 50Hz로 모터 명령을 내보낸다.
    # 대기(idle) 중에는 0 속도를 0.5초마다 한 번만 보낸다. 예전에는 대기 중에도 50Hz로 0을 보내
    # 이 프로그램과 turtlebot3_ros(시리얼 쓰기) 둘 다 CPU를 계속 썼다. idle 전환 순간에는 change()가 즉시 0을 보낸다.
    idle_sent={'at':float('-inf')}
    # 50Hz 타이머. 주행 중에는 매번, idle 에서는 0.5초마다 한 번 0 속도를 보낸다.
    def tick():
        now=time.monotonic()
        if owner.mode=='idle':
            if now-idle_sent['at']<.5:return
            idle_sent['at']=now
        send(*owner.output(now))
    node.create_timer(.02,tick)
    mission_gate = EdgeHeartbeat(1.)
    communication_gate = EdgeHeartbeat(1.)
    # 0.5초마다: 통신 상태 방송, 통신 오류면 강제 idle, 모드 방송, 임무 상태(mission/status) 변경 시 방송.
    def health():
        snapshot=guard.snapshot();error=snapshot['error']
        if communication_gate.due((owner.mode, error), time.monotonic()):
            communication_pub.publish(String(data=json.dumps({'mode':owner.mode, **snapshot})))
        if error and owner.mode!='idle':
            owner.switch('idle');send(0.,0.);node.get_logger().error('모터 통신 감시 오류: '+error)
        status.publish(String(data=owner.mode))
        cached=mission_snapshot['data']
        if cached is not None:
            try:
                record=json.loads(cached)
                key=tuple(record.get(k) for k in ('command_id','stage','status','error'))
            except (ValueError, TypeError):key=cached
            if mission_gate.due(key,time.monotonic()):mission_pub.publish(String(data=cached))
    node.create_timer(.5,health)
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        owner.switch('idle');end=time.monotonic()+.3
        while rclpy.ok() and time.monotonic()<end:send(0.,0.);rclpy.spin_once(node,timeout_sec=.02)
        file_stop.set();guard.close();file_worker.join(timeout=.5)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
