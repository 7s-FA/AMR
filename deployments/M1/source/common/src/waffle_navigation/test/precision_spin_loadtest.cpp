#include "pluginlib/class_loader.hpp"
#include "nav2_core/behavior.hpp"
int main() {
  pluginlib::ClassLoader<nav2_core::Behavior> loader("nav2_core", "nav2_core::Behavior");
  auto plugin = loader.createSharedInstance("waffle_navigation::PrecisionSpin");
  auto departure = loader.createSharedInstance("waffle_navigation::DepartureSpin");
  return plugin && departure ? 0 : 1;
}
