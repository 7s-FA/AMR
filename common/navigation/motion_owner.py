#!/usr/bin/env python3
"""One motor publisher, exclusive command owner. No planning or speed tuning."""
import argparse,math,signal,time,json,threading
from communication_guard import GraphGuard
from pathlib import Path

class CommandOwner:
    def __init__(self,timeout=.35):
        self.mode='idle';self.last=None;self.timeout=timeout
    def switch(self,mode):
        if mode not in ('idle','nav','direct'):raise ValueError(mode)
        self.mode=mode;self.last=None
    def receive(self,source,v,w,now):
        if source!=self.mode or not all(math.isfinite(x) for x in (v,w,now)):return False
        self.last=(v,w,now);return True
    def output(self,now):
        if self.mode=='idle' or self.last is None or now-self.last[2]>self.timeout:return 0.,0.
        return self.last[:2]

def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',required=True);a=p.parse_args()
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from data_flow import EdgeHeartbeat
    from std_srvs.srv import Trigger
    rclpy.init(args=[]);node=rclpy.create_node('motion_owner',namespace='/'+a.robot)
    owner=CommandOwner();stopping=False
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
    def send(v,w):
        v,w=gate_output(gate_file,v,w)
        m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id=a.robot+'/base_footprint';m.twist.linear.x=v;m.twist.angular.z=w;output.publish(m)
    guard=GraphGuard('/'+a.robot+'/cmd_vel', 'motion_owner', '/'+a.robot, 'motor', period=.5, max_age=1.5)
    mission_snapshot={'data': None};file_stop=threading.Event()
    def read_mission():
        while not file_stop.is_set():
            try:mission_snapshot['data']=mission_file.read_text()
            except OSError:mission_snapshot['data']=None
            file_stop.wait(.5)
    file_worker=threading.Thread(target=read_mission,daemon=True);file_worker.start()
    def communication_error():
        return guard.snapshot()['error']
    def change(mode):
        def cb(req,res):
            owner.switch('idle');send(0.,0.)
            error=communication_error()
            if mode!='idle' and error:res.success=False;res.message='모터 연결 확인 대기/오류: '+error;return res
            owner.switch(mode);status.publish(String(data=mode));res.success=True;res.message=mode;return res
        return cb
    for mode in ('idle','nav','direct'):node.create_service(Trigger,'motion_owner/'+mode,change(mode))
    def input_cb(source):
        def cb(m):
            ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            age=(node.get_clock().now().nanoseconds-ns)/1e9
            if age<-.1 or age>.5:return
            if owner.receive(source,m.twist.linear.x,m.twist.angular.z,time.monotonic()) and abs(m.twist.linear.x)+abs(m.twist.angular.z)<1e-9:send(0.,0.)
        return cb
    node.create_subscription(TwistStamped,'cmd_vel_nav_out',input_cb('nav'),1)
    node.create_subscription(TwistStamped,'cmd_vel_direct',input_cb('direct'),1)
    node.create_timer(.02,lambda:send(*owner.output(time.monotonic())))
    mission_gate = EdgeHeartbeat(1.)
    communication_gate = EdgeHeartbeat(1.)
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
