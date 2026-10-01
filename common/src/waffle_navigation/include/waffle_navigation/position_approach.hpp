#pragma once
#include <cmath>
#include "waffle_navigation/precision_pose.hpp"

namespace waffle_navigation {
// Position-only approach. Saved waypoint yaw is deliberately never used.
class PositionApproach {
public:
  Pose current{}, goal{};
  static constexpr double drive_floor = .025, radius = .15;
  double bearing(Pose pose) const {
    return angle(std::atan2(goal.y-pose.y,goal.x-pose.x)-pose.yaw);
  }
  bool allowed(double v, double w) const {
    if (!std::isfinite(v) || !std::isfinite(w) || v < -1e-9) return false;
    // Keep zero translation for steering, but reject the observed wheel-stall
    // interval. Do not impose a positive min_vel_x that removes rotation.
    if (v > 1e-9 && v < drive_floor-1e-9) return false;
    if (distance(current,goal)>radius) return true;
    const double error = bearing(current);
    if (std::abs(w)>.21+1e-9) return false;
    if (v>1e-9 && std::abs(error)>35.0*3.141592653589793/180.0) return false;
    if (v<=1e-9) {
      // Turn only toward the coordinate; avoid zero-translation tiny commands.
      if (std::abs(w)<.07-1e-9 || w*error<=0) return false;
    }
    return true;
  }
  bool pose_allowed(Pose predicted) const {
    return distance(current,goal)>radius ||
      distance(predicted,goal)<=distance(current,goal)+.002;
  }
  double score(Pose end) const {
    if (distance(current,goal)>radius) return 0;
    // A continuous XY cost avoids cell plateaus. The target is the coordinate,
    // not its final parking orientation. Suppress bearing singularity inside
    // the unchanged 2cm arrival tolerance.
    double heading = distance(end,goal)<=.02 ? 0 : std::abs(bearing(end));
    return distance(end,goal)+.035*heading;
  }
};
}
