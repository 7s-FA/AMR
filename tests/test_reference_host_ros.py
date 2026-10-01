"""Use unmodified user-provided host clients with motor-free Action backends."""
import importlib.util
import os
from pathlib import Path
import sys
import time
import pytest
from test_action_ros import rig,wait
HOST=Path(os.environ.get('AMR_REFERENCE_HOST_SOURCE','/home/jh/Desktop/final_251001/final_project_ws/host/src/host_pkg/src'))
if not HOST.exists():pytest.skip('Set AMR_REFERENCE_HOST_SOURCE to reference host src',allow_module_level=True)
sys.path.insert(0,str(HOST))

def until(condition,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if condition():return
  time.sleep(.01)
 raise AssertionError('reference host callback timeout')

@pytest.mark.parametrize('successful',[True,False])
def test_unmodified_host_client_exchanges_goal_feedback_result(rig,successful):
 backend,_,server=rig;number=server.robot[-1]
 spec=importlib.util.spec_from_file_location('reference_burger'+number,HOST/('burger'+number+'_node.py'))
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 client=getattr(module,'Burger'+number+'ActionClient')();server.executor.add_node(client)
 memory=module.shared_memory;prefix='burger'+number
 setattr(memory,prefix+'_end_flag',False)
 try:
  assert client._client.wait_for_server(timeout_sec=3.)
  client.send_goal('GO_TO_ASM',50.)
  until(lambda:len(backend.submissions)==1)
  until(lambda:getattr(memory,prefix+'_x_axis')==pytest.approx(.4))
  backend.verified=successful;backend.done=True
  until(lambda:hasattr(client,'_get_result_future') and client._get_result_future.done())
  until(lambda:getattr(memory,prefix+'_end_message')==('IDLE' if successful else 'ERROR'))
  assert getattr(memory,prefix+'_success')==successful
  assert client._get_result_future.result().result.success==successful
  # Record, do not repair, the reference host's known M2 completion flag defect.
  assert getattr(memory,prefix+'_end_flag')==(number=='1')
 finally:
  server.executor.remove_node(client);client._client.destroy();client.destroy_node()
