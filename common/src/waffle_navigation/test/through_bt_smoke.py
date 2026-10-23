"""Isolated real Nav2 BT + fake planner/controller; no cmd_vel publisher."""
import os
assert os.environ.get('ROS_DOMAIN_ID') == '149'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
import time, threading, subprocess, tempfile, pathlib, json, sys
CANCEL = "--cancel" in sys.argv
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, ActionClient, CancelResponse
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateThroughPoses
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformBroadcaster

TREE=str(pathlib.Path(__file__).resolve().parents[1]/'behavior_trees/navigate_through_forward.xml')
rclpy.init()
node=Node('through_smoke',namespace='through_test')
exec=MultiThreadedExecutor(num_threads=5);exec.add_node(node)
group=ReentrantCallbackGroup()
state={'started':None,'x':0.,'follow_calls':0,'cancel_calls':0,'plans':[]}
tf=TransformBroadcaster(node)
odompub=node.create_publisher(Odometry,'odom',10)
def tick():
    t=state['started']
    # Purely synthetic coordinates, in a separate ROS domain.
    state['x']=0. if t is None else min(.8,max(0.,time.monotonic()-t)*1.1)
    tr=TransformStamped();tr.header.stamp=node.get_clock().now().to_msg()
    tr.header.frame_id='map';tr.child_frame_id='base_link'
    tr.transform.translation.x=state['x'];tr.transform.rotation.w=1.
    tf.sendTransform(tr)
    od=Odometry();od.header=tr.header;od.child_frame_id='base_link';od.pose.pose.position.x=state['x'];od.pose.pose.orientation.w=1.
    odompub.publish(od)
node.create_timer(.01,tick,callback_group=group)
def planner(handle):
    state['plans'].append(len(handle.request.goals))
    result=ComputePathThroughPoses.Result();result.path.header.frame_id='map'
    result.path.header.stamp=node.get_clock().now().to_msg()
    start=PoseStamped();start.header=result.path.header;start.pose.position.x=state['x'];start.pose.orientation.w=1.
    result.path.poses=[start]+list(handle.request.goals)
    handle.succeed();return result

def cancel(handle):
    state['cancel_calls']+=1
    return CancelResponse.ACCEPT

def follow(handle):
    state['follow_calls']+=1
    assert handle.request.controller_id=='FollowPositionForward'
    assert handle.request.goal_checker_id=='position_goal_checker'
    if state['started'] is None:state['started']=time.monotonic()
    while time.monotonic()-state['started']<.82:
        if handle.is_cancel_requested:
            handle.canceled();return FollowPath.Result()
        time.sleep(.01)
    handle.succeed();return FollowPath.Result()
servers=[ActionServer(node,ComputePathThroughPoses,'compute_path_through_poses',execute_callback=planner,callback_group=group),
ActionServer(node,FollowPath,'follow_path',execute_callback=follow,cancel_callback=cancel,callback_group=group)]
thread=threading.Thread(target=exec.spin,daemon=True);thread.start()
params={'use_sim_time':False,'global_frame':'map','robot_base_frame':'base_link','odom_topic':'odom',
        'bt_loop_duration':10,'default_server_timeout':500,'wait_for_service_timeout':2000,
        'navigators':['navigate_through_poses'],'navigate_through_poses.plugin':'nav2_bt_navigator::NavigateThroughPosesNavigator',
        'default_nav_through_poses_bt_xml':TREE,'enable_groot_monitoring':False,'bond_heartbeat_period':0.0}
import yaml
parampath='/tmp/through_bt_smoke.yaml'
pathlib.Path(parampath).write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
log=open('/tmp/through_bt_smoke.log','w')
process=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator','--ros-args','-r','__ns:=/through_test','--params-file',parampath],stdout=log,stderr=log)
def wait(future,seconds=10):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:time.sleep(.01)
    assert future.done(),'future timeout'
    return future.result()
try:
    client=node.create_client(ChangeState,'bt_navigator/change_state',callback_group=group)
    assert client.wait_for_service(timeout_sec=10)
    for transition in [1,3]:
        req=ChangeState.Request();req.transition.id=transition
        assert wait(client.call_async(req)).success,('lifecycle transition',transition)
    action=ActionClient(node,NavigateThroughPoses,'navigate_through_poses',callback_group=group)
    assert action.wait_for_server(timeout_sec=5)
    time.sleep(.3)
    goal=NavigateThroughPoses.Goal();goal.behavior_tree=TREE
    for x in [.4,.8]:
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg();p.pose.position.x=x;p.pose.orientation.w=1.
        goal.poses.append(p)
    remaining=[]
    handle=wait(action.send_goal_async(goal,feedback_callback=lambda msg:remaining.append(msg.feedback.number_of_poses_remaining)))
    assert handle.accepted
    if CANCEL:
        time.sleep(.15)
        assert wait(handle.cancel_goal_async()).goals_canceling
    result=wait(handle.get_result_async())
    if CANCEL:
        assert result.status == 5 and state['cancel_calls'] == 1, (result.status,state)
        print(json.dumps({'result':'PASS', 'scenario':'cancel', 'controller_cancelled':True}))
        sys.exit(0)
    assert result.status==4,(result.status,result.result)
    assert 2 in remaining and 1 in remaining,remaining
    assert state['follow_calls']==1 and state['cancel_calls']==0,state
    print(json.dumps({'result':'PASS','via_feedback':sorted(set(remaining)),'follow_path_goals':state['follow_calls'],'cancellations':state['cancel_calls'],'plans':state['plans']}))
finally:
    process.terminate()
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.kill();process.wait()
    exec.shutdown(timeout_sec=2);node.destroy_node();rclpy.shutdown();log.close()
