"""Opt-in pre-alignment clearance; disabled stations retain their old policy."""
import importlib.util
import math
from pathlib import Path
import sys
import pytest

spec=importlib.util.spec_from_file_location('clearance_control',Path(__file__).resolve().parents[1]/'common/runtime/docking_control.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)

def feed(c,t,z=.215,side=.02,yaw=0.,valid=True):
    c.set_ir(False,t);c.set_odom(0.,0.,0.,0.,0.,t)
    c.set_observation({'pose_valid':valid,'tvec_m':[-side,0.,z],'normal_yaw_rad':-math.radians(yaw)},t)

def create():
    return m.DockingControl(m.Settings(alignment_min_distance_m=.20,staging_distance_m=.22,
                                      align_speed_mps=.025,final_speed_mps=.025))

@pytest.mark.parametrize('side',[-.02,.02])
@pytest.mark.parametrize('z',[.215,.205,.16])
def test_lateral_offset_cannot_consume_more_clearance(side,z):
    c=create();feed(c,0,z,side);assert c.start(0)[0]
    assert c.tick(0)==(0.,0.)
    assert c.state=='FAULT' and c.reason=='alignment_clearance_exhausted'

def test_rotation_can_finish_then_hold_before_final():
    c=create();feed(c,0,side=.215*math.tan(math.radians(3)),yaw=3);assert c.start(0)[0]
    v,w=c.tick(0);assert v==0 and w>0
    for i in range(1,60):feed(c,i*.01,side=0);assert c.tick(i*.01)==(0.,0.)
    assert c.state=='ALIGN'
    for i in range(60,90):feed(c,i*.01,side=0);c.tick(i*.01)
    assert c.state=='FINAL_APPROACH'

def test_default_disables_guard():
    assert m.Settings().alignment_min_distance_m==0

@pytest.mark.parametrize('value',[-.01,float('nan'),.12,.3])
def test_invalid_clearance_rejected(value):
    with pytest.raises(ValueError):m.Settings(alignment_min_distance_m=value,staging_distance_m=.22)


def test_visual_entry_requires_turn_before_translation():
    c=m.DockingControl(m.Settings(visual_entry_alignment=True,alignment_min_distance_m=.20,
                                  staging_distance_m=.30,align_speed_mps=.025,final_speed_mps=.025))
    feed(c,0,z=.55,side=.09);assert c.start(0)[0]
    v,w=c.tick(0);assert v==0 and w>0 and c.reason=='visual_entry_heading_alignment'
    feed(c,.1,z=.55,side=0.);assert c.tick(.1)[0]>0


def test_visual_entry_disabled_by_default_and_boolean_only():
    assert m.Settings().visual_entry_alignment is False
    with pytest.raises(ValueError):m.Settings(visual_entry_alignment=1)
