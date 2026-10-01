"""M1/M2 action server. RESTART releases a stop latch; it never resumes a goal."""
import json,math,threading,time
import rclpy
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from host_pkg.action import Burger
from .backend import Backend,ROUTES,validate

class MissionServer(Node):
    def __init__(self,backend=None,robot_id='M2'):
        super().__init__('amr_mission_server', namespace=robot_id)
        self.declare_parameter('robot_id',robot_id);self.declare_parameter('runtime_robot',{'M1':'burger1','M2':'burger2'}[robot_id])
        self.declare_parameter('navigation_dir','');self.declare_parameter('action_name','')
        self.declare_parameter('mission_timeout_s',600.)
        self.robot=self.get_parameter('robot_id').value;runtime=self.get_parameter('runtime_robot').value
        if (self.robot,runtime) not in [('M1','burger1'),('M2','burger2')]:raise ValueError('Robot/profile mismatch')
        self.backend=backend or Backend(self.get_parameter('navigation_dir').value)
        name=self.get_parameter('action_name').value or '/'+self.robot+'/data'
        if self.get_namespace()!='/'+self.robot or not name.startswith('/'+self.robot+'/'):
            raise ValueError('Action namespace must match the uppercase robot ID')
        self.group=ReentrantCallbackGroup();self.lock=threading.RLock()
        self.route_reserved=False;self.control_reserved=False;self.active_id=None
        self.pose=None;self.odom_time=None;self.stationary_since=None;self.direction='FORWARD'
        self.diagnostics=self.create_publisher(String,'/'+self.robot+'/mission/diagnostics',10)
        self.create_subscription(PoseWithCovarianceStamped,'/'+runtime+'/amcl_pose',self.on_pose,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(Odometry,'/'+runtime+'/odom',self.on_odom,qos_profile_sensor_data,callback_group=self.group)
        self.action=ActionServer(self,Burger,name,execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=self.cancel,callback_group=self.group)
        self.create_timer(.15,self.heartbeat,callback_group=self.group)
        self.get_logger().info('Robot action ready: '+name+' (no movement sent)')
    def report(self,stage,reason='',goal_id=None):
        self.diagnostics.publish(String(data=json.dumps({'robot':self.robot,'stage':stage,'reason':reason,'goal_id':goal_id})))
        if reason:self.get_logger().info(stage+': '+reason)
    def stamp_age(self,stamp):
        return (self.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
    def on_pose(self,msg):
        p=msg.pose.pose.position
        if msg.header.frame_id not in ('map',self.get_parameter('runtime_robot').value+'/map'):return
        if not -.1<=self.stamp_age(msg.header.stamp)<=1. or not all(math.isfinite(v) for v in (p.x,p.y)):return
        self.pose=(p.x,p.y,time.monotonic())
    def on_odom(self,msg):
        v,w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if not -.1<=self.stamp_age(msg.header.stamp)<=.3 or not all(math.isfinite(x) for x in (v,w)):return
        now=time.monotonic()
        if self.odom_time is None or now-self.odom_time>.3:self.stationary_since=None
        self.odom_time=now
        if abs(v)<=.01 and abs(w)<=.04:
            if self.stationary_since is None:self.stationary_since=now
        else:self.stationary_since=None
        if abs(v)>.001:self.direction='FORWARD' if v>0 else 'BACKWARD'
    def stationary(self):
        now=time.monotonic()
        return (self.odom_time is not None and now-self.odom_time<=.3
                and self.stationary_since is not None and now-self.stationary_since>=.3)
    def goal(self,request):
        try:validate(request.command,request.cmd_val)
        except ValueError as e:self.report('rejected',str(e));return GoalResponse.REJECT
        with self.lock:
            if request.command in ROUTES:
                if self.route_reserved or self.control_reserved:return GoalResponse.REJECT
                try:self.backend.preflight()
                except Exception as e:self.report('rejected',str(e));return GoalResponse.REJECT
                self.route_reserved=True
            else:
                if self.control_reserved:return GoalResponse.REJECT
                if request.command=='RESTART' and self.route_reserved:return GoalResponse.REJECT
                self.control_reserved=True
        return GoalResponse.ACCEPT
    def cancel(self,handle):
        return CancelResponse.ACCEPT if handle.request.command in ROUTES else CancelResponse.REJECT
    def heartbeat(self):
        with self.lock:goal_id=self.active_id
        if goal_id:
            try:self.backend.renew(goal_id)
            except Exception as e:self.report('heartbeat_failed',str(e),goal_id)
    def feedback(self,handle):
        pose=self.pose
        if pose is None or time.monotonic()-pose[2]>1.:return
        f=Burger.Feedback();f.robot_x=float(pose[0]);f.robot_y=float(pose[1]);f.robot_theta=self.direction;f.message='IDLE';handle.publish_feedback(f)
    def execute(self,handle):
        command=handle.request.command;result=Burger.Result();result.success=False;result.message='ERROR'
        goal_id=bytes(handle.goal_id.uuid).hex();route=command in ROUTES;successful=False
        try:
            if command=='EMER_STOP':
                self.backend.stop()
                end=time.monotonic()+3.
                while rclpy.ok() and time.monotonic()<end and not self.stationary():time.sleep(.05)
                if not self.stationary():raise RuntimeError('STOP_LATCHED_BUT_STATIONARY_NOT_CONFIRMED')
                successful=True
            elif command=='RESTART':
                if not self.stationary():raise RuntimeError('FRESH_STATIONARY_ODOMETRY_REQUIRED')
                self.backend.restart();successful=True
            else:
                with self.lock:self.active_id=goal_id
                self.backend.submit(command,handle.request.cmd_val,goal_id)
                self.report('accepted',command,goal_id)
                end=time.monotonic()+self.get_parameter('mission_timeout_s').value
                while rclpy.ok() and time.monotonic()<end:
                    if handle.is_cancel_requested:
                        self.backend.stop('action_cancelled');handle.canceled();return result
                    if not self.backend.authorized(goal_id):raise RuntimeError('STOP_LATCHED_OR_ACTION_OWNERSHIP_LOST')
                    record=self.backend.result(goal_id)
                    if record and record.get('status') in ('success','failed','cancelled'):
                        proof=record.get('result',{})
                        successful=(record['status']=='success' and record.get('stage')=='arrived'
                                    and proof.get('success') is True and proof.get('stopped') is True
                                    and proof.get('terminal_verified') is True
                                    and proof.get('terminal_mode') in ('dock','park','rest')
                                    and (proof.get('terminal_mode')=='rest' or proof.get('dock_verified') is True))
                        if not successful:raise RuntimeError(record.get('error','MISSION_FAILED_OR_UNVERIFIED'))
                        break
                    self.feedback(handle);time.sleep(.2)
                if not successful:raise RuntimeError('MISSION_TIMEOUT_OR_SHUTDOWN')
            result.success=successful;result.message='IDLE' if successful else 'ERROR'
            handle.succeed() if successful else handle.abort()
            self.report('complete',command,goal_id)
            return result
        except Exception as e:
            self.report('failed',str(e),goal_id)
            if route:
                try:self.backend.stop('action_failed')
                except Exception as stop_error:self.report('stop_failed',str(stop_error),goal_id)
            if handle.is_active:handle.abort()
            return result
        finally:
            if route:
                try:self.backend.finish(goal_id,successful)
                except Exception as cleanup_error:self.report('cleanup_failed',str(cleanup_error),goal_id)
                finally:
                    with self.lock:self.active_id=None;self.route_reserved=False
            else:
                with self.lock:self.control_reserved=False
    def close(self):
        if self.active_id:
            try:self.backend.stop('action_server_shutdown')
            except Exception as e:self.get_logger().error(str(e))
        self.action.destroy();self.destroy_node()

def main(args=None):
    rclpy.init(args=args);node=MissionServer();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.close();executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
