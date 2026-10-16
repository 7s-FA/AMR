#!/usr/bin/env python3
# ========================================================================
# 역할: 준비 상태 모니터. 1초에 한 줄씩 odom·라이다·지도 위치·모드·도킹 상태를 보여 준다 (읽기 전용).
# 실행: watch_ready.sh(→ ready_watch.sh) 로 로봇 터미널에서 수동 실행. 창을 닫아도 서비스는 멈추지 않는다.
# ========================================================================
"""One read-only status line per second. Closing monitor never stops services."""
import argparse,json,math,time
from pathlib import Path

# 구독을 만들고 1초마다 상태 한 줄 출력 (--samples N 이면 N줄 후 종료).
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
 # 토픽별 마지막 메시지 시각 기록.
 def stamp(key,m):last[key]=(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 n.create_subscription(LaserScan,'scan',lambda m:stamp('scan',m),qos_profile_sensor_data)
 n.create_subscription(Odometry,'odom',lambda m:stamp('odom',m),qos_profile_sensor_data)
 n.create_subscription(String,'motion_owner/status',lambda m:mode.update(value=m.data,at=time.monotonic()),10)
 # 도킹 상태 토픽(JSON)에서 state 값 기록.
 def docking(m):
  try:d=json.loads(m.data);dock.update(value=d.get('state','?'),at=time.monotonic())
  except ValueError:pass
 n.create_subscription(String,'docking/status',docking,10)
 n.create_subscription(Bool,'ir/high',lambda m:ir.update(value='HIGH' if m.data else 'LOW',at=time.monotonic()),10)
 root=Path(__file__).absolute().parents[3];count=0;due=time.monotonic()
 print('1초 상태 표시 시작. Ctrl+C는 표시만 종료합니다. 로봇 정지: b'+a.robot[-1]+'_stop',flush=True)
 try:
  while rclpy.ok():
   rclpy.spin_once(n,timeout_sec=.02)
   if time.monotonic()<due:continue
   due=time.monotonic()+1.;now=n.get_clock().now().nanoseconds/1e9
   # 토픽 마지막 수신 후 경과 시간 문자열.
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
   # 상태 값 표시 (3초 넘게 안 바뀌면 '지난 상태' 표시).
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
