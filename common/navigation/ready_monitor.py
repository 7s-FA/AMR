#!/usr/bin/env python3
"""One read-only status line per second. Closing monitor never stops services."""
import argparse,json,math,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('robot');p.add_argument('--samples',type=int,default=0);a=p.parse_args()
 import rclpy
 from tf2_ros import Buffer,TransformListener
 from sensor_msgs.msg import LaserScan
 from nav_msgs.msg import Odometry
 from std_msgs.msg import String,Bool
 from rclpy.qos import qos_profile_sensor_data
 rclpy.init(args=['--ros-args','-r','/tf:=/'+a.robot+'/tf','-r','/tf_static:=/'+a.robot+'/tf_static'])
 n=rclpy.create_node('ready_status_monitor',namespace='/'+a.robot);buf=Buffer();listener=TransformListener(buf,n,spin_thread=False)
 last={};mode={'value':'대기'};dock={'value':'대기'};ir={'value':'?'}
 def stamp(key,m):last[key]=(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 n.create_subscription(LaserScan,'scan',lambda m:stamp('scan',m),qos_profile_sensor_data)
 n.create_subscription(Odometry,'odom',lambda m:stamp('odom',m),qos_profile_sensor_data)
 n.create_subscription(String,'motion_owner/status',lambda m:mode.update(value=m.data,at=time.monotonic()),10)
 def docking(m):
  try:d=json.loads(m.data);dock.update(value=d.get('state','?'),at=time.monotonic())
  except ValueError:pass
 n.create_subscription(String,'docking/status',docking,10)
 n.create_subscription(Bool,'ir/high',lambda m:ir.update(value='HIGH' if m.data else 'LOW',at=time.monotonic()),10)
 root=Path(__file__).resolve().parents[3];count=0;due=time.monotonic()
 print('1초 상태 표시 시작. Ctrl+C는 표시만 종료합니다. 로봇 정지: b'+a.robot[-1]+'_stop',flush=True)
 try:
  while rclpy.ok():
   rclpy.spin_once(n,timeout_sec=.02)
   if time.monotonic()<due:continue
   due=time.monotonic()+1.;now=n.get_clock().now().nanoseconds/1e9
   def age(key):return f'{now-last[key]:.2f}s' if key in last else '수신대기'
   pose='지도 위치 대기'
   try:
    t=buf.lookup_transform(a.robot+'/map',a.robot+'/base_footprint',rclpy.time.Time());q=t.transform.rotation
    yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    lag=now-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)
    pose=f'x={t.transform.translation.x:.3f} y={t.transform.translation.y:.3f} yaw={yaw:.1f}° TF={max(0,lag):.2f}s'
   except Exception:pass
   try:s=json.loads((root/'data'/a.robot/'mission_state.json').read_text());task=f"{s.get('destination','?')}/{s.get('stage','?')}/{s.get('status','?')}"
   except (OSError,ValueError):task='없음'
   def status(value):
    suffix='(지난 상태)' if 'at' in value and time.monotonic()-value['at']>3 else ''
    return value['value']+suffix
   print(f"[{time.strftime('%H:%M:%S')}] {a.robot} {pose} | scan={age('scan')} odom={age('odom')} | 제어={status(mode)} 도킹={status(dock)} IR={status(ir)} | 작업={task}",flush=True)
   count+=1
   if a.samples and count>=a.samples:break
 except KeyboardInterrupt:pass
 finally:
  listener.unregister();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
