import math,subprocess
from pathlib import Path
import pytest,yaml
import importlib.util
from test_directional_waypoints import route,ROOT,FakeNavigator

@pytest.mark.parametrize('mismatch',[None,'local','global'])
def test_behavior_collision_checks_use_costmap_coordinates(mismatch):
 spec=importlib.util.spec_from_file_location('frame_validation',ROOT/'config/arrival_tuning.py')
 helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
 params=yaml.safe_load((ROOT/'config/nav2_burger2_params.yaml').read_text())
 if mismatch:
  params['behavior_server']['ros__parameters'][f'{mismatch}_frame']='burger2/odom'
  with pytest.raises(ValueError,match='충돌검사 좌표계 불일치'):
   helper.validate_collision_frames(params)
 else:
  helper.validate_collision_frames(params)

@pytest.mark.parametrize('mode,pose,expected',[
 ('forward',(0,0,0),0),('reverse',(0,0,math.pi),0),
 ('forward',(0,0,math.pi),math.pi),('reverse',(0,0,0),math.pi),
 ('reverse',(0,0,-math.pi),0)])
def test_travel_direction_does_not_use_final_parking_yaw(mode,pose,expected):
 d,a=route.travel_heading_error(dict(x=1,y=0,yaw=90,mode=mode),pose)
 assert d==1 and abs(a)==pytest.approx(expected)

def test_pre_alignment_is_bounded_and_waits_for_stop(monkeypatch):
 n=FakeNavigator();n.xy_tolerance=.02;n.map_poses=[(0,0,math.pi),(0,0,0)]
 monkeypatch.setattr(route,'wait_for_task',lambda *args:True)
 assert route.WaypointNavigator.align_for_travel(n,dict(x=1,y=0,mode='forward'),route.time.monotonic()+40,1)
 assert len(n.spin_requests)==1 and 'stopped' in n.events
 n=FakeNavigator();n.xy_tolerance=.02;n.map_poses=[(0,0,math.pi)]*3
 assert not route.WaypointNavigator.align_for_travel(n,dict(x=1,y=0,mode='forward'),route.time.monotonic()+40,1)
 assert len(n.spin_requests)==2

def test_actual_dwb_turn_iterator_preserves_zero_and_both_directions(tmp_path):
 # Compile against the installed Nav2 iterator, not a reimplementation.
 src=tmp_path/'samples.cpp';exe=tmp_path/'samples'
 src.write_text('''#include <dwb_plugins/one_d_velocity_iterator.hpp>
#include <cassert>
#include <cstdlib>
int main(int argc,char**argv){
 double limit=std::atof(argv[1]);
 dwb_plugins::OneDVelocityIterator w(0,-limit,limit,std::atof(argv[2]),std::atof(argv[3]),std::atof(argv[4]),std::atoi(argv[5]));
 bool zero=false,positive=false,negative=false;
 for(;!w.isFinished();++w){double v=w.getVelocity();assert(std::abs(v)<=limit+1e-8);zero|=std::abs(v)<1e-8;positive|=v>0;negative|=v<0;}
 assert(zero && positive && negative);
}''')
 subprocess.run(['c++','-I/opt/ros/jazzy/include',str(src),'-o',str(exe)],check=True)
 p=yaml.safe_load((ROOT/'config/nav2_burger2_params.yaml').read_text())
 c=p['controller_server']['ros__parameters']['FollowPositionForward']
 subprocess.run([str(exe)]+[str(c[k]) for k in ['max_vel_theta','acc_lim_theta','decel_lim_theta','sim_time','vtheta_samples']],check=True)
 assert 'BaseObstacle' in c['critics']
 assert p['local_costmap']['local_costmap']['ros__parameters']['resolution']==.01
 assert p['collision_monitor']['ros__parameters']['scan']['enabled']

@pytest.mark.parametrize('kind',['delayed','missing','wrong_namespace','wrong_type'])
def test_velocity_graph_discovery_is_bounded_and_checks_identity(monkeypatch,kind):
 from types import SimpleNamespace
 clock=[0.0]
 monkeypatch.setattr(route.time,'monotonic',lambda:clock[0])
 monkeypatch.setattr(route.rclpy,'ok',lambda:True)
 monkeypatch.setattr(route.rclpy,'spin_once',lambda n,timeout_sec:clock.__setitem__(0,clock[0]+timeout_sec))
 def endpoints(topic):
  assert topic.startswith('/burger2/')
  if kind=='missing' or clock[0]<.2:return []
  name='velocity_smoother' if topic.endswith('cmd_vel_nav') else 'collision_monitor'
  return [SimpleNamespace(node_name=name,node_namespace='/burger1' if kind=='wrong_namespace' else '/burger2',topic_type='geometry_msgs/msg/Twist' if kind=='wrong_type' else 'geometry_msgs/msg/TwistStamped')]
 n=SimpleNamespace(get_namespace=lambda:'/burger2',get_subscriptions_info_by_topic=endpoints)
 if kind=='delayed':
  route.WaypointNavigator._wait_for_velocity_connections(n,timeout=.5)
  assert .2<=clock[0]<.5
 else:
  with pytest.raises(RuntimeError,match='시간 초과'):
   route.WaypointNavigator._wait_for_velocity_connections(n,timeout=.5)
  assert clock[0]==pytest.approx(.5)
