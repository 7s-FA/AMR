#!/usr/bin/env python3
"""Read-only post-failure evidence. Never publishes goals or velocity commands."""
import argparse,json,time,rclpy
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--robot",choices=("burger1","burger2"),required=True);p.add_argument("--output",required=True);args=p.parse_args()
robot=args.robot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
rclpy.init(args=['--ros-args','-r','/tf:=/'+robot+'/tf','-r','/tf_static:=/'+robot+'/tf_static']);n=rclpy.create_node('collision_snapshot',namespace='/'+robot);data={};subs=[]
for key,cls,topic,qos in [('scan',LaserScan,'/'+robot+'/scan',qos_profile_sensor_data),('footprint',PolygonStamped,'/'+robot+'/local_costmap/published_footprint',qos_profile_sensor_data)]:
 subs.append(n.create_subscription(cls,topic,lambda msg,k=key:data.__setitem__(k,message_to_ordereddict(msg)),qos))
b=Buffer();listener=TransformListener(b,n)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
c=n.create_client(GetCostmap,'/'+robot+'/local_costmap/get_costmap')
if c.wait_for_service(timeout_sec=3):
 f=c.call_async(GetCostmap.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if f.done() and f.result():data['costmap']=message_to_ordereddict(f.result().map)
for frame in (robot+'/map',robot+'/odom'):
 for child in (robot+'/base_footprint',robot+'/base_scan'):
  try:data[frame+'->'+child]=message_to_ordereddict(b.lookup_transform(frame,child,rclpy.time.Time()))
  except Exception as exc:data[frame+'->'+child]={'error':str(exc)}
for node,params in [('local_costmap/local_costmap',['robot_radius','footprint','footprint_padding','global_frame','robot_base_frame']),('behavior_server',['simulate_ahead_time','local_costmap_topic','local_footprint_topic','transform_tolerance','local_frame','global_frame','robot_base_frame'])]:
 c=n.create_client(GetParameters,'/'+robot+'/'+node+'/get_parameters')
 if c.wait_for_service(timeout_sec=2):
  f=c.call_async(GetParameters.Request(names=params));rclpy.spin_until_future_complete(n,f,timeout_sec=3)
  if f.done() and f.result():data[node]=dict(zip(params,[message_to_ordereddict(v) for v in f.result().values]))
data['captured_unix']=time.time();data['robot']=robot;data['capture_phase']='after_failure_stop';Path(args.output).write_text(json.dumps(data));listener.unregister();n.destroy_node();rclpy.shutdown()
