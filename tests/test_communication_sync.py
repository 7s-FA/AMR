"""No hardware access: latest robot communication changes plus AMR protections."""
import importlib.util
import json
from pathlib import Path
import queue
import sys
import time
from types import SimpleNamespace
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'common/runtime'),str(ROOT/'common/navigation')]
from communication_guard import read_snapshot,GraphGuard
from docking_standby import validate_request,wait_fresh
from motion_owner import CommandOwner
from action_gate import gate_output

@pytest.mark.parametrize('state,now,error',[
 ([2,10,0,3],10.1,None),([2,10,2,3],10.1,'cmd_vel_has_other_publisher_or_graph_not_ready'),
 ([2,10,0,3],11.1,'graph_monitor_stale'),([0,0,1,0],10,'graph_monitor_stale'),
 ([3,10,0,3],10.1,'graph_snapshot_unavailable')])
def test_guard_snapshots_are_bounded_and_fail_closed(state,now,error):
 assert read_snapshot(state,now)['error']==error


def test_in_progress_snapshot_reuses_only_fresh_committed_result():
 g=GraphGuard.__new__(GraphGuard);g.state=[2,10,0,1];g.max_age=1.;g.cached=None;g.cached_at=None
 assert g.snapshot(10.1)['error'] is None
 g.state[0]=3
 assert g.snapshot(10.2)['error'] is None
 assert g.snapshot(11.2)['error']=='graph_monitor_stale'


def test_cross_robot_guard_rejected_without_starting_process():
 with pytest.raises(ValueError):GraphGuard('/burger1/cmd_vel','motion_owner','/burger2','motor')


def test_owner_timeout_and_action_gate_both_remain_active(tmp_path):
 owner=CommandOwner();owner.switch('nav');owner.receive('nav',.06,.1,10.)
 assert owner.output(10.1)==(.06,.1)
 assert owner.output(10.5)==(0.,0.)
 p=tmp_path/'gate.json';p.write_text(json.dumps({'estop':True}))
 assert gate_output(p,.06,.1,10.1)==(0.,0.)


@pytest.mark.parametrize('issued',[7.,11.])
def test_expired_or_future_warm_request_never_starts(issued):
 with pytest.raises(ValueError):validate_request({'action':'start','issued':issued},10.)


def test_warm_switch_requires_three_fresh_target_board_frames():
 cfg={'docking_mode':'parking','board_spec':{'markers':[8]},'target_ids':[8]}
 class Channel:
  def __init__(self):self.calls=0
  def get(self,timeout):
   self.calls+=1;stamp=time.time()
   return {'source_stamp':{'sec':int(stamp),'nanosec':int(stamp%1*1e9)},'sequence':self.calls,
           'status':'ok','board_spec':cfg['board_spec'],'vision_mode':'normal' if self.calls==1 else 'parking'}
 class Vision:
  channel=Channel()
  def set_active(self,value):pass
  def set_mode(self,value):assert value=='parking'
 vision=Vision();result=wait_fresh(vision,cfg)
 assert result['fresh_frames']==3 and vision.channel.calls==4
