"""Pure geometry regression tests; no ROS node or robot command is created."""
from pathlib import Path
import subprocess


def test_continuous_path_heading(tmp_path):
    root = Path(__file__).resolve().parents[1]
    source = tmp_path / 'path_heading.cpp'
    source.write_text(r'''
#include "waffle_navigation/position_approach.hpp"
#include <cassert>
#include <limits>
using namespace waffle_navigation;
int main() {
  const std::vector<Pose> sparse{{0,0,0},{.4,0,0},{.4,.4,0}};
  PositionApproach disabled;
  disabled.prepare({.3,0,0},{.4,.4,0}); disabled.prepare_path(sparse);
  assert(!disabled.have_path_target && disabled.score({.3,0,0})==0);
  PositionApproach p; p.path_heading_weight=.003; p.path_lookahead=.16;
  p.prepare({.3,0,0},{.4,.4,0}); p.prepare_path(sparse);
  assert(p.have_path_target && std::abs(p.path_target.x-.4)<1e-10);
  assert(std::abs(p.path_target.y-.06)<1e-10);
  // Both forward arcs and a stop remain legal; the soft cost prefers the
  // correcting heading without weakening any obstacle/oscillation critic.
  assert(p.allowed(0,0) && p.allowed(.03,.3));
  assert(p.score({.32,.002,.3})<p.score({.32,0,0}));
  assert(p.score({.32,.002,.3})<p.score({.32,-.002,-.3}));
  auto target=p.path_target;
  p.prepare_path({{0,0,2},{.1,0,1},{.3,0,-1},{.4,0,3},{.4,0,3},{.4,.2,0},{.4,.4,1}});
  assert(distance(target,p.path_target)<1e-10); // spacing/yaw independent
  p.prepare({.3,0,0},{.4,-.4,0});
  p.prepare_path({{0,0,0},{.4,0,0},{.4,-.4,0}});
  assert(p.score({.32,-.002,-.3})<p.score({.32,.002,.3}));
  p.prepare_path({}); assert(!p.have_path_target);
  p.prepare_path({{0,0,0},{0,0,0}}); assert(!p.have_path_target);
  p.prepare_path({{0,0,0},{std::numeric_limits<double>::quiet_NaN(),0,0}});
  assert(!p.have_path_target);
  // Replans replace the carrot rather than keeping the previous turn.
  p.prepare({.3,0,0},{1,0,0});p.prepare_path({{0,0,0},{1,0,0}});
  assert(std::abs(p.path_target.x-.46)<1e-10 && p.path_target.y==0);
  assert(p.score({.33,0,0})<p.score({.33,.005,.3}));
  // Final 15 cm, braking guard and drive floor are unchanged.
  p.prepare({.9,0,0},{1,0,0});p.prepare_path({{0,0,0},{1,0,0}});
  disabled.prepare(p.current,p.goal);
  assert(p.score({.93,0,.1})==disabled.score({.93,0,.1}));
  assert(!p.allowed(.01,0) && !p.allowed(-.03,0));
  p.heading_guard_enabled=true;p.prepare({.3,0,1},{1,0,0},.05,0);
  assert(p.heading_hold && p.allowed(0,0) && !p.allowed(.03,-.2));
}
''')
    binary = tmp_path / 'path_heading'
    subprocess.run(['g++', '-std=c++17', '-I', str(root/'include'),
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
