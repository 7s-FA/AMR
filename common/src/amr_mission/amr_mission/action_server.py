# ========================================================================
# 역할: 관제(host_backend)가 보내는 ROS 액션 /M1/data, /M2/data 를 받는 로봇 쪽 액션 서버.
#       GO_TO_MAT/ASM/REST/PARK 는 이동 임무로, EMER_STOP/RESTART 는 정지 래치 제어로 처리한다.
# 실행: m1-action.service / m2-action.service → .runtime/Mx/start_action.sh → ros2 run amr_mission action_server
# 호출 관계: 실제 일은 backend.py(Backend)가 하고, Backend 는 operation.py 로 이동 임무를 시작한다.
# 주고받는 값: 목표(command, cmd_val=속도%), 피드백(robot_x, robot_y, robot_theta=전진/후진, message), 결과(success, message=IDLE/ERROR).
# ========================================================================
"""M1/M2 action server. RESTART releases a stop latch; it never resumes a goal."""
import json,math,threading,time
import rclpy
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from host_pkg.action import Burger
from .backend import Backend,ROUTES,validate

# 액션 서버 노드. 명령 접수 → 임무 시작 → 결과 파일을 보며 완료/실패를 관제에 돌려준다.
class MissionServer(Node):
    # 파라미터 확인(M1↔burger1, M2↔burger2), odom/위치 수신 노드 준비, 액션 서버와 0.15초 하트비트 타이머 생성.
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
        self.last_odom_stamp_ns=-1;self.last_pose_log_at=0.
        self.pose=None;self.odom_time=None;self.stationary_since=None;self.direction='FORWARD'
        latest_sensor_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.diagnostics=self.create_publisher(String,'/'+self.robot+'/mission/diagnostics',10)
        # odom(30Hz)·amcl_pose 수신은 별도 노드를 싱글스레드 실행기로 따로 돌린다.
        # ROS 2 Jazzy의 파이썬 MultiThreadedExecutor는 자주 오는 구독이 하나만 있어도 거의 쉬지 않고 돌아
        # 대기 중에도 CPU를 크게 쓴다(PC 측정: 같은 구독 기준 멀티 약 75% vs 싱글 약 3%).
        # 액션 처리(명령 접수·완료 대기)는 그대로 MultiThreadedExecutor에서 돈다.
        self.sensors=Node('amr_mission_sensors',namespace=robot_id)
        self.sensors.create_subscription(PoseWithCovarianceStamped,'/'+runtime+'/amcl_pose',self.on_pose,latest_sensor_qos)
        self.sensors.create_subscription(Odometry,'/'+runtime+'/odom',self.on_odom,latest_sensor_qos)
        self.sensor_executor=SingleThreadedExecutor();self.sensor_executor.add_node(self.sensors)
        self.sensor_thread=threading.Thread(target=self.spin_sensors,daemon=True);self.sensor_thread.start()
        self.action=ActionServer(self,Burger,name,execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=self.cancel,callback_group=self.group)
        self.create_timer(.15,self.heartbeat,callback_group=self.group)
        self.get_logger().info('Robot action ready: '+name+' (no movement sent)')
    # 센서 수신 전용 스레드 본체. 싱글스레드 실행기로 odom·amcl_pose 콜백만 돌린다.
    def spin_sensors(self):
        try:self.sensor_executor.spin()
        except Exception:pass  # rclpy 종료(shutdown) 시 나는 예외. 이 스레드는 수신 전용이라 무시한다.
    # 진단 토픽(/Mx/mission/diagnostics)에 단계·사유를 JSON으로 내보내고 로그도 남긴다.
    def report(self,stage,reason='',goal_id=None):
        self.diagnostics.publish(String(data=json.dumps({'robot':self.robot,'stage':stage,'reason':reason,'goal_id':goal_id})))
        if reason:self.get_logger().info(stage+': '+reason)
    # 메시지 시각이 지금보다 몇 초 전인지 계산 (오래된 센서 값 거르기용).
    def stamp_age(self,stamp):
        return (self.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
    # AMCL 위치를 받아 피드백용 (x, y)를 저장. 다른 로봇 좌표계·오래된 값은 버린다.
    def on_pose(self,msg):
        p=msg.pose.pose.position
        if msg.header.frame_id not in ('map',self.get_parameter('runtime_robot').value+'/map'):return
        if not -.1<=self.stamp_age(msg.header.stamp)<=1. or not all(math.isfinite(v) for v in (p.x,p.y)):return
        self.pose=(p.x,p.y,time.monotonic())
    # odom 속도로 '정지 상태' 시작 시각과 전진/후진 방향을 기록한다.
    def on_odom(self,msg):
        v,w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if not -.1<=self.stamp_age(msg.header.stamp)<=.3 or not all(math.isfinite(x) for x in (v,w)):return
        stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp_ns<=self.last_odom_stamp_ns:return
        self.last_odom_stamp_ns=stamp_ns
        now=time.monotonic()
        if self.odom_time is None or now-self.odom_time>.3:self.stationary_since=None
        self.odom_time=now
        if abs(v)<=.01 and abs(w)<=.04:
            if self.stationary_since is None:self.stationary_since=now
        else:self.stationary_since=None
        if abs(v)>.001:self.direction='FORWARD' if v>0 else 'BACKWARD'
    # 최근 odom 기준 0.3초 이상 멈춰 있으면 True (정지·해제 판정에 사용).
    def stationary(self):
        now=time.monotonic()
        return (self.odom_time is not None and now-self.odom_time<=.3
                and self.stationary_since is not None and now-self.stationary_since>=.3)
    # 새 목표 접수 여부 결정. 잘못된 명령/속도, 이미 이동 중, 정지 래치 중이면 거부한다.
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
    # 관제의 취소 요청은 이동 임무만 받는다 (정지/해제 명령은 취소 불가).
    def cancel(self,handle):
        return CancelResponse.ACCEPT if handle.request.command in ROUTES else CancelResponse.REJECT
    # 이동 중이면 0.15초마다 action_gate.json 하트비트를 갱신한다. 끊기면 모터 출력이 0이 된다.
    def heartbeat(self):
        with self.lock:goal_id=self.active_id
        if goal_id:
            try:self.backend.renew(goal_id)
            except Exception as e:self.report('heartbeat_failed',str(e),goal_id)
    # 현재 위치·방향을 관제로 피드백 전송 (위치가 1초 넘게 오래되면 보내지 않음).
    def feedback(self,handle):
        pose=self.pose
        now=time.monotonic()
        if now-self.last_pose_log_at>=5.:
            self.get_logger().debug(f'pose: {pose}')
            self.last_pose_log_at=now
        if pose is None or time.monotonic()-pose[2]>1.:return
        f=Burger.Feedback();f.robot_x=float(pose[0]);f.robot_y=float(pose[1]);f.robot_theta=self.direction;f.message='IDLE';handle.publish_feedback(f)
    # 목표 실행 본체. EMER_STOP=래치 후 정지 확인, RESTART=정지 확인 후 래치 해제, 이동=임무 시작 후 결과 파일 대기.
    def execute(self,handle):
        command=handle.request.command;result=Burger.Result();result.success=False;result.message='ERROR'
        goal_id=bytes(handle.goal_id.uuid).hex();route=command in ROUTES;successful=False
        recoverable_navigation_failure=False
        try:
            if command=='EMER_STOP':
                self.backend.stop()
                end=time.monotonic()+3.
                while rclpy.ok() and time.monotonic()<end and not self.stationary():time.sleep(.05)
                if not self.stationary():raise RuntimeError('STOP_LATCHED_BUT_STATIONARY_NOT_CONFIRMED')
                successful=True
            elif command=='RESTART':
                # EMER_STOP처럼 최대 3초 동안 정지를 확인한다. 순간적인 odom 지연만으로 해제가 거부되지 않게 한다.
                end=time.monotonic()+3.
                while rclpy.ok() and time.monotonic()<end and not self.stationary():time.sleep(.05)
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
                        if not successful:
                            recoverable_navigation_failure=(self.robot in ('M1','M2')
                                and record.get('status')=='failed'
                                and record.get('recoverable_navigation_failure') is True)
                            raise RuntimeError(record.get('error','MISSION_FAILED_OR_UNVERIFIED'))
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
                try:
                    self.backend.stop('action_failed')
                    # Retry exhaustion is still ERROR to the host. Allow a NEW command only
                    # after stationary feedback; operator stop/cancel can never be cleared.
                    if recoverable_navigation_failure and not handle.is_cancel_requested:
                        end=time.monotonic()+5.
                        while rclpy.ok() and time.monotonic()<end and not self.stationary():
                            time.sleep(.05)
                        if (rclpy.ok() and self.stationary() and not handle.is_cancel_requested
                                and self.backend.release_failed_navigation(goal_id)):
                            self.report('ready_after_navigation_failure','재시도 3회 소진; 정지 확인 후 새 명령 대기',goal_id)
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
    # 종료 처리. 이동 중이었다면 정지 래치를 건 뒤 노드를 정리한다.
    def close(self):
        if self.active_id:
            try:self.backend.stop('action_server_shutdown')
            except Exception as e:self.get_logger().error(str(e))
        self.sensor_executor.shutdown();self.sensors.destroy_node()
        self.action.destroy();self.destroy_node()

# 프로그램 시작점. 액션 처리는 4스레드 실행기로 돈다 (센서 수신은 별도 스레드).
def main(args=None):
    rclpy.init(args=args);node=MissionServer();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.close();executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
