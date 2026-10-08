#!/usr/bin/env python3
"""Activate localization; once per boot seed the confirmed own parking pose."""
import argparse,time,json,math,subprocess,fcntl
from collections import deque
from startup_state import wait_response,wait_responses
from localization_ready import ensure_active
from pathlib import Path

def atomic(path,data):
 path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False));temp.replace(path)
def main():
 p=argparse.ArgumentParser();p.add_argument('robot');p.add_argument('--force-parked',action='store_true');a=p.parse_args()
 root=Path(__file__).absolute().parents[3];data=root/'data'/a.robot;boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
 pid=subprocess.check_output(['systemctl','--user','show',a.robot+'-localization.service','-p','MainPID','--value'],text=True).strip()
 ready=data/'localization_ready.json';seed=data/'localization_seed.json'
 def read(path):
  try:return json.loads(path.read_text())
  except (OSError,ValueError):return {}
 if not a.force_parked and read(ready).get('boot')==boot and read(ready).get('pid')==pid and pid!='0':return 0
 import rclpy,yaml
 from lifecycle_msgs.srv import GetState
 from nav2_msgs.srv import SetInitialPose,ManageLifecycleNodes
 from std_srvs.srv import Trigger
 from geometry_msgs.msg import PoseWithCovarianceStamped
 from tf2_ros import Buffer,TransformListener
 rclpy.init(args=['--ros-args','-r','/tf:=/'+a.robot+'/tf','-r','/tf_static:=/'+a.robot+'/tf_static']);n=rclpy.create_node('station_ready_check',namespace='/'+a.robot)
 ack=deque(maxlen=16)
 def on_pose(m):ack.append((time.monotonic(),m.pose.pose))
 n.create_subscription(PoseWithCovarianceStamped,'amcl_pose',on_pose,10)
 clients={name:n.create_client(GetState,name+'/get_state') for name in ('map_server','amcl')}
 manager=n.create_client(Trigger,'lifecycle_manager_localization/is_active')
 control=n.create_client(ManageLifecycleNodes,'lifecycle_manager_localization/manage_nodes')
 buf=Buffer();listener=TransformListener(buf,n,spin_thread=False)
 def spin(timeout):rclpy.spin_once(n,timeout_sec=timeout)
 def query(timeout):
  end=time.monotonic()+timeout;pending={}
  try:
   while len(pending)<len(clients) and time.monotonic()<end:
    for name,c in clients.items():
     if name not in pending and c.service_is_ready():pending[name]=c.call_async(GetState.Request())
    if len(pending)<len(clients):spin(.05)
   if len(pending)!=len(clients):raise RuntimeError('localization state service discovery incomplete')
   replies=wait_responses(pending,clients,spin,max(.01,end-time.monotonic()),'localization state query')
   if any(reply is None for reply in replies.values()):raise RuntimeError('localization state response empty')
   return {name:reply.current_state.id for name,reply in replies.items()}
  finally:
   for name,future in pending.items():
    if not future.done():clients[name].remove_pending_request(future);future.cancel()
 def manager_active(timeout):return wait_response(manager,Trigger.Request(),spin,timeout,'localization manager state').success
 def failed_bringup():
  unit=a.robot+'-localization.service'
  invocation=subprocess.check_output(['systemctl','--user','show',unit,'-p','InvocationID','--value'],text=True,timeout=3).strip()
  if not invocation:return False
  log=subprocess.check_output(['journalctl','--user','_SYSTEMD_INVOCATION_ID='+invocation,'-n','80','--no-pager','-o','cat'],text=True,timeout=3)
  aborted=log.rfind('Failed to bring up all requested nodes. Aborting bringup.')
  progressing=max(log.rfind('Starting managed nodes bringup...'),log.rfind('Resuming managed nodes...'),log.rfind('Managed nodes are active'))
  return aborted>=0 and aborted>progressing
 def recover(command,timeout):
  if command=='restart':
   # The uniform manager operations cannot resume a partially active stack.
   # Recover only the explicitly failed localization invocation while idle.
   from operation import Operation
   op=Operation(Path(__file__).absolute().parent)
   op.data.mkdir(parents=True,exist_ok=True)
   with (op.data/'operation.lock').open('a') as recovery_lock:
    # Never block behind a parent ready command which already holds this lock.
    try:fcntl.flock(recovery_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('localization restart refused: operation admission is locked')
    if op.busy(include_preparation=False) or op.mode()!='individual':raise RuntimeError('localization restart refused: robot not idle in preparation mode')
    if not failed_bringup():raise RuntimeError('localization restart refused: bringup no longer aborted')
    subprocess.run(['systemctl','--user','restart',a.robot+'-localization.service'],check=True,timeout=min(20.,timeout))
   return
  request=ManageLifecycleNodes.Request();request.command=request.RESUME if command=='resume' else request.STARTUP
  reply=wait_response(control,request,spin,timeout,'localization recovery')
  if not reply.success:raise RuntimeError('localization manager rejected '+command)
 def fresh_pose():
  try:
   t=buf.lookup_transform(a.robot+'/map',a.robot+'/base_footprint',rclpy.time.Time())
   stamp=t.header.stamp.sec*10**9+t.header.stamp.nanosec;age=(n.get_clock().now().nanoseconds-stamp)/1e9
   return t if -.8<=age<=1. else None
  except Exception:return None
 try:
  ensure_active(query,manager_active,failed_bringup,recover,report=lambda msg:print(a.robot+' '+msg,flush=True))
  if a.force_parked or read(seed).get('boot')!=boot:
   # Preserve a current location if the operator already localized this session.
   rclpy.spin_once(n,timeout_sec=.2)
   existing=fresh_pose()
   if not a.force_parked and existing is not None:
    t=existing;cfg=yaml.safe_load((root/'host_ws/src/waffle_navigation/config'/('nav2_'+a.robot+'_params.yaml')).read_text());home=cfg['amcl']['ros__parameters']['initial_pose']
    q=t.transform.rotation;yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z);angle=math.atan2(math.sin(yaw-float(home['yaw'])),math.cos(yaw-float(home['yaw'])))
    at_home=math.hypot(t.transform.translation.x-float(home['x']),t.transform.translation.y-float(home['y']))<.1 and abs(angle)<math.radians(20)
    atomic(seed,{'boot':boot,'source':'own_parking_existing' if at_home else 'existing_localization','updated_unix':time.time()})
    if at_home:
     data.mkdir(parents=True,exist_ok=True);(data/'departure_pending').write_text('confirmed existing own parking location\n')
   else:
    cfg=yaml.safe_load((root/'host_ws/src/waffle_navigation/config'/('nav2_'+a.robot+'_params.yaml')).read_text())
    pose=cfg['amcl']['ros__parameters']['initial_pose'];initial=n.create_client(SetInitialPose,'set_initial_pose')
    deadline=time.monotonic()+8
    while not initial.wait_for_service(timeout_sec=.2) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
    if not initial.service_is_ready():raise RuntimeError('AMCL 초기 위치 서비스 연결 없음')
    msg=PoseWithCovarianceStamped();msg.header.frame_id=a.robot+'/map'
    # A zero timestamp requests the latest transform, avoiding future-time warnings.
    msg.pose.pose.position.x=float(pose['x']);msg.pose.pose.position.y=float(pose['y']);msg.pose.pose.position.z=float(pose.get('z',0))
    msg.pose.pose.orientation.z=math.sin(float(pose['yaw'])/2);msg.pose.pose.orientation.w=math.cos(float(pose['yaw'])/2)
    msg.pose.covariance[0]=.25;msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.06853891945200942;sent=time.monotonic()
    # A service acknowledgment confirms receipt; an endpoint count alone does not.
    request=SetInitialPose.Request();request.pose=msg
    applied=initial.call_async(request)
    rclpy.spin_until_future_complete(n,applied,timeout_sec=5)
    if not applied.done() or applied.result() is None:
     raise RuntimeError('AMCL 초기 위치 적용 응답 없음: 자동 재전송하지 않습니다.')
    # Wait for AMCL acknowledgment through fresh map -> base TF.
    deadline=time.monotonic()+30;valid=False
    while time.monotonic()<deadline:
     rclpy.spin_once(n,timeout_sec=.1)
     if ack and ack[-1][0]>=sent and fresh_pose() is not None:
      p=ack[-1][1];yaw=math.atan2(2*p.orientation.w*p.orientation.z,1-2*p.orientation.z*p.orientation.z)
      angle=math.atan2(math.sin(yaw-float(pose['yaw'])),math.cos(yaw-float(pose['yaw'])))
      if math.hypot(p.position.x-float(pose['x']),p.position.y-float(pose['y']))<.35 and abs(angle)<math.radians(45):valid=True;break
    if not valid:raise RuntimeError('자동 초기 위치 적용 후 지도 TF 없음: 라이다/본체 연결 확인')
    atomic(seed,{'boot':boot,'source':'own_parking','pose':pose,'updated_unix':time.time()})
    data.mkdir(parents=True,exist_ok=True);(data/'departure_pending').write_text('confirmed own parking at startup\n')
    print(a.robot+' 지정 주차장 초기 위치 자동 적용: '+str(pose),flush=True)
  # A restarted AMCL process loses its pose even during the same OS boot.
  # Never treat lifecycle activation alone as a usable localization.
  deadline=time.monotonic()+8
  while fresh_pose() is None and time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.1)
  if fresh_pose() is None:
   raise RuntimeError('AMCL 재시작 후 초기 위치가 없습니다. 지정 주차장이라면 b'+a.robot[-1]+'_parked, 다른 위치라면 RViz의 2D Pose Estimate로 현재 위치를 지정하세요.')
  pid=subprocess.check_output(['systemctl','--user','show',a.robot+'-localization.service','-p','MainPID','--value'],text=True,timeout=3).strip()
  atomic(ready,{'boot':boot,'pid':pid,'updated_unix':time.time()})
  print(a.robot+' 위치 추정 활성화 확인',flush=True)
  return 0
 finally:
  listener.unregister()
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
