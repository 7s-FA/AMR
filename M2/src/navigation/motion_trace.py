#!/usr/bin/env python3
"""Read-only 1 Hz trace; no goals, velocity publishers or camera processing."""
import argparse,json,time
import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
p=argparse.ArgumentParser();p.add_argument('--robot',required=True);p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();node=rclpy.create_node('motion_trace',namespace='/'+a.robot);latest={};subs=[]
def receive(key,msg):latest[key]=(time.monotonic(),message_to_ordereddict(msg))
for topic in ['cmd_vel_nav','cmd_vel_smoothed','cmd_vel_nav_out','cmd_vel']:
 subs.append(node.create_subscription(TwistStamped,'/'+a.robot+'/'+topic,lambda m,k=topic:receive(k,m),qos_profile_sensor_data))
subs.append(node.create_subscription(CollisionMonitorState,'/'+a.robot+'/collision_monitor_state',lambda m:receive('collision',m),qos_profile_sensor_data))
with open(a.output,'a',buffering=1) as stream:
 try:
  due=0
  while rclpy.ok():
   rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
   if now>=due:
    stream.write(json.dumps({'unix':time.time(),**{k:{'age_s':round(now-t,3),'message':v} for k,(t,v) in latest.items()}})+'\n');due=now+1
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.try_shutdown()
