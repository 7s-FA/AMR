import math
import pytest
from test_position_then_yaw import navigator, POINTS
from test_directional_waypoints import ROOT, route

@pytest.mark.parametrize('angle, expected', [(180,1),(90,1),(30,0),(45,0)])
def test_large_heading_alignment_before_forward_goal(monkeypatch, angle, expected):
    nav=navigator(monkeypatch);nav.align_large_heading_before_navigation=True
    nav.current_map_pose=lambda timeout=2: (0.,0.,0.)
    monkeypatch.setattr(route,'travel_heading_error',lambda *a:(.19,math.radians(angle)))
    calls=[]
    nav.align_for_travel=lambda *a:calls.append('align') or True
    monkeypatch.setattr(route,'finish_waypoint',lambda *a,**kw:True)
    assert route.run_waypoints(nav,POINTS[:1],60,ROOT/'behavior_trees')==0
    assert len(calls)==expected
    assert len(nav.goals)==1


def test_failed_alignment_never_sends_translation_goal(monkeypatch):
    nav=navigator(monkeypatch);nav.align_large_heading_before_navigation=True
    nav.current_map_pose=lambda timeout=2:(0.,0.,0.)
    monkeypatch.setattr(route,'travel_heading_error',lambda *a:(.19,math.pi))
    nav.align_for_travel=lambda *a:False
    assert route.run_waypoints(nav,POINTS[:1],60,ROOT/'behavior_trees')==1
    assert not nav.goals

@pytest.mark.parametrize('angle, expected', [(90,0),(100,0),(101,1),(180,1)])
def test_m1_configured_prealignment_threshold(monkeypatch, angle, expected):
    nav=navigator(monkeypatch);nav.align_large_heading_before_navigation=True
    nav.prealign_heading_deg=100.;nav.current_map_pose=lambda timeout=2:(0.,0.,0.)
    monkeypatch.setattr(route,'travel_heading_error',lambda *a:(.19,math.radians(angle)))
    calls=[];nav.align_for_travel=lambda *a:calls.append('align') or True
    monkeypatch.setattr(route,'finish_waypoint',lambda *a,**kw:True)
    assert route.run_waypoints(nav,POINTS[:1],60,ROOT/'behavior_trees')==0
    assert len(calls)==expected and len(nav.goals)==1
