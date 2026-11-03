"""Read-only scan adapter; never moves a robot or masks fork returns."""
import math
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from waffle.vision_backoff import rear_clear


class RearObservation:
    def __init__(self,node):
        self.node=node
        self.scan=None
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,node)
        self.subscription=node.create_subscription(LaserScan,'/scan',self.receive,qos_profile_sensor_data)

    def receive(self,msg):
        try:
            if not (0 < msg.range_min < msg.range_max and 0 < msg.angle_increment < .1 and 30 <= len(msg.ranges) <= 10000):
                raise ValueError('invalid scan')
            t=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            q=t.rotation
            if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,t.translation.x,t.translation.y,msg.angle_min)):
                raise ValueError('nonfinite transform')
            if abs(q.x)>.001 or abs(q.y)>.001 or abs(q.z*q.z+q.w*q.w-1)>.001:
                raise ValueError('nonplanar transform')
            yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            rays=[]
            for i,r in enumerate(msg.ranges):
                a=msg.angle_min+i*msg.angle_increment+yaw
                valid=math.isfinite(r) and msg.range_min <= r <= msg.range_max
                rays.append((a,t.translation.x+r*math.cos(a) if valid else None,t.translation.y+r*math.sin(a) if valid else None))
            self.scan={'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'rays':rays}
        except Exception:
            self.scan=None

    def check(self):
        return rear_clear(self.scan,self.node.get_clock().now().nanoseconds/1e9)
