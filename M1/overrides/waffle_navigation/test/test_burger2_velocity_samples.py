"""Check the actual installed DWB sampler, rather than duplicating its math."""
import pytest
import subprocess
import yaml
from test_directional_waypoints import ROOT


@pytest.mark.parametrize('robot', ['burger1','burger2'])
def test_forward_sampler_preserves_rotation_but_filters_stalled_translation(tmp_path, robot):
    p = yaml.safe_load((ROOT/f'config/nav2_{robot}_params.yaml').read_text())
    c = p['controller_server']['ros__parameters']['FollowPositionForward']
    src, binary = tmp_path/'samples.cpp', tmp_path/'samples'
    src.write_text('''#include <dwb_plugins/one_d_velocity_iterator.hpp>
#include "waffle_navigation/position_approach.hpp"
#include <iostream>
#include <cstdlib>
int main(int argc,char**argv){
 dwb_plugins::OneDVelocityIterator it(.05,std::atof(argv[1]),std::atof(argv[2]),
  .25,-.35,.6,std::atoi(argv[3]));
 waffle_navigation::PositionApproach policy; policy.current={0,0,0};policy.goal={1,0,0};
 for(;!it.isFinished();++it) if(policy.allowed(it.getVelocity(),0)) std::cout<<it.getVelocity()<<"\\n";
}''')
    subprocess.run(['c++', '-std=c++17', '-I/opt/ros/jazzy/include', '-I'+str(ROOT/'include'), str(src), '-o', str(binary)], check=True)
    output = subprocess.check_output([str(binary), str(c['min_vel_x']),
                                       str(c['max_vel_x']), str(c['vx_samples'])], text=True)
    samples = [float(v) for v in output.split()]
    assert len(samples) >= 3
    assert min(samples) == 0 and max(samples) == pytest.approx(c['max_vel_x'])
    assert all(v == 0 or v >= .025 for v in samples)
    assert .025 <= min(v for v in samples if v > 0) <= .03
    assert 'waffle_navigation::PositionApproach' in c['critics']
    expected_speed, expected_turn = (.066, .462)
    assert c['max_speed_xy'] == expected_speed and c['max_vel_theta'] == expected_turn
    assert 'RotateToGoal' not in c['critics'] and 'BaseObstacle' in c['critics']
    assert p['collision_monitor']['ros__parameters']['scan']['enabled']
