#include <cassert>
#include <cmath>
#include "waffle_navigation/precision_spin_speed.hpp"
int main() {
  using waffle_navigation::precision_spin_cap;
  constexpr double pi = 3.14159265358979323846;
  using namespace waffle_navigation;
  assert(encoder_turn_target(pi)==4965);
  assert(encoder_turn_target(-pi)==4965);
  assert(encoder_tick_delta(INT32_MIN,INT32_MAX)==1);
  assert(encoder_tick_delta(INT32_MAX,INT32_MIN)==-1);
  assert(!encoder_turn_invalid(4965,4965,4965));
  assert(!encoder_turn_invalid(4865,5065,4965));
  assert(encoder_turn_invalid(4965,5066,4965));
  assert(encoder_turn_invalid(-101,0,4965));
  assert(encoder_turn_invalid(201,0,4965));
  const double maximum = 0.3592512;
  assert(precision_spin_cap(pi, maximum) == maximum);
  assert(precision_spin_cap(0, maximum) == 0.06);
  assert(precision_spin_cap(0.05, maximum) < 0.08);
  assert(precision_spin_cap(-0.05, maximum) == precision_spin_cap(0.05, maximum));
  assert(precision_spin_cap(0, 0.04) == 0.04);
  double previous = 0.06;
  for (int i=0; i<=1800; ++i) {
    double speed = precision_spin_cap(i*pi/1800, maximum);
    assert(speed >= previous && speed <= maximum);
    previous = speed;
  }
}
