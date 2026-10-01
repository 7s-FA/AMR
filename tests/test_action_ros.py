"""Isolated action transport tests. Fake backend never issues motor commands."""
import os,threading,time
import pytest
if os.environ.get('ROS_DOMAIN_ID')!='232':pytest.skip('Use isolated ROS_DOMAIN_ID=232',allow_module_level=True)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from host_pkg.action import Burger
from amr_mission.action_server import MissionServer

class FakeBackend:
    def __init__(self):self.estop=False;self.busy=False;self.mode='process';self.submissions=[];self.done=False;self.renewed=0
    def preflight(self):
        if self.estop or self.busy or self.mode!='process':raise RuntimeError('blocked')
    def submit(self,command,speed,goal_id):self.preflight();self.busy=True;self.submissions.append((command,speed,goal_id))
    def renew(self,goal_id):self.renewed+=1;return not self.estop
    def result(self,goal_id):return {'status':'success','stage':'arrived','result':{'success':True,'stopped':True,'dock_verified':True}} if self.done else {'status':'running'}
    def stop(self,reason='stop'):self.estop=True;self.busy=False
    def restart(self):
        assert not self.busy
        self.estop=False
    def finish(self,goal_id,success):self.busy=False

def wait(f,timeout=5):
    end=time.monotonic()+timeout
    while not f.done() and time.monotonic()<end:time.sleep(.01)
    assert f.done(),'future timeout';return f.result()

@pytest.fixture
def rig():
    rclpy.init();backend=FakeBackend();server=MissionServer(backend);io=Node('amr_contract_test_client')
    client=ActionClient(io,Burger,'/m2/data');odom=io.create_publisher(Odometry,'/burger2/odom',10);pose=io.create_publisher(PoseWithCovarianceStamped,'/burger2/amcl_pose',10)
    def sensors():
        o=Odometry();o.header.stamp=io.get_clock().now().to_msg();o.pose.pose.orientation.w=1.;odom.publish(o)
        p=PoseWithCovarianceStamped();p.header.stamp=o.header.stamp;p.header.frame_id='burger2/map';p.pose.pose.position.x=.4;p.pose.pose.position.y=-.2;p.pose.pose.orientation.w=1.;pose.publish(p)
    io.create_timer(.03,sensors);ex=MultiThreadedExecutor(num_threads=6);ex.add_node(io);ex.add_node(server);t=threading.Thread(target=ex.spin);t.start();assert client.wait_for_server(timeout_sec=3);time.sleep(.5)
    try:yield backend,client,server
    finally:
        backend.done=True;time.sleep(.3);ex.shutdown(timeout_sec=4);t.join(timeout=4);client.destroy();server.close();io.destroy_node();rclpy.shutdown()

def goal(client,command,speed=100.,feedback=None):
    g=Burger.Goal();g.command=command;g.cmd_val=speed
    return wait(client.send_goal_async(g,feedback_callback=feedback))

def test_host_contract_success_only_after_terminal_completion(rig):
    backend,client,server=rig;seen=[];g=goal(client,'GO_TO_ASM',50.,lambda x:seen.append(x.feedback));assert g.accepted
    result=g.get_result_async();time.sleep(.5);assert not result.done();assert seen and seen[-1].robot_x==pytest.approx(.4)
    backend.done=True;r=wait(result);assert r.result.success and r.result.message=='IDLE';assert backend.submissions[0][:2]==('GO_TO_ASM',50.)

def test_duplicate_goal_rejected_and_stop_preempts_then_restart_only_releases(rig):
    backend,client,server=rig;g=goal(client,'GO_TO_PARK');assert g.accepted
    assert not goal(client,'GO_TO_ASM').accepted
    stop=goal(client,'EMER_STOP',0.);assert stop.accepted;assert wait(stop.get_result_async()).result.success
    assert not wait(g.get_result_async()).result.success
    assert not goal(client,'GO_TO_MAT').accepted
    release=goal(client,'RESTART',0.);assert release.accepted;assert wait(release.get_result_async()).result.success
    assert len(backend.submissions)==1 and not backend.estop

def test_cancellation_stops_and_latches(rig):
    backend,client,server=rig;g=goal(client,'GO_TO_REST');assert g.accepted;wait(g.cancel_goal_async());r=wait(g.get_result_async());assert r.status==5 and not r.result.success and backend.estop

def test_old_host_example_command_rejected(rig):
    backend,client,_=rig;assert not goal(client,'move_to_warehouse',1.5).accepted;assert not backend.submissions

def test_individual_mode_rejects_host_route(rig):
    backend,client,_=rig;backend.mode='individual';assert not goal(client,'GO_TO_MAT').accepted
