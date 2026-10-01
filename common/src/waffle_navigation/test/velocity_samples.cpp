// Check the installed DWB implementation, including floating-point endpoint sampling.
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <dwb_plugins/one_d_velocity_iterator.hpp>

int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  const double low = std::atof(argv[1]);
  const double high = std::atof(argv[2]);
  const int samples = std::atoi(argv[3]);
  dwb_plugins::OneDVelocityIterator iterator(0.0, low, high, 3.0, -2.5, 1.5, samples);
  bool has_zero = false;
  bool has_motion = false;
  for (; !iterator.isFinished(); ++iterator) {
    const double velocity = iterator.getVelocity();
    has_zero = has_zero || velocity == 0.0;
    has_motion = has_motion || std::abs(velocity) > 0.001;
    if (velocity < low - 1e-12 || velocity > high + 1e-12) {return 1;}
  }
  if (!has_zero || !has_motion) {
    std::cerr << "DWB must sample motion AND exact zero for RotateToGoal\n";
    return 1;
  }
  return 0;
}
