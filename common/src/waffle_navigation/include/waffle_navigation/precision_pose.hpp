#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace waffle_navigation {
struct Pose { double x, y, yaw; };
inline double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double distance(Pose a, Pose b) { return std::hypot(a.x-b.x, a.y-b.y); }

// Pure geometry policy shared by the DWB critic and offline trajectory tests.
// All speeds are positive-forward. Invalid candidates remain subject to Nav2's
// normal failure handling; this policy never commands the motor topic.
class PrecisionPolicy {
public:
  static constexpr double radius = .05, xy = .02, lock_radius = .012;
  static constexpr double yaw_tolerance = 3.0 * 3.14159265358979323846 / 180.0;
  Pose current{}, goal{};
  bool initialized = false, near = false, locked = false;
  double previous_turn = 0;

  void prepare(Pose pose, Pose target) {
    if (!initialized || distance(goal, target) > 1e-5 ||
        std::abs(angle(goal.yaw-target.yaw)) > 1e-5) {
      near = locked = false;
      previous_turn = 0;
      initialized = true;
    }
    current = pose; goal = target;
    near = near || distance(pose, target) <= radius;
    locked = locked || distance(pose, target) <= lock_radius;
  }
  double speed_limit() const {
    if (locked) return 0;
    return std::clamp(.006 + .014 * (distance(current,goal)-lock_radius) /
      (radius-lock_radius), .006, .02);
  }
  bool command_allowed(double v, double w) const {
    if (!std::isfinite(v) || !std::isfinite(w) || v < -1e-9) return false;
    if (!near) return true;
    if (locked && distance(current,goal) > xy) return false;
    if (v > speed_limit() + 1e-9 || std::abs(w) > .12 + 1e-9) return false;
    if (locked) {
      double error = angle(goal.yaw-current.yaw);
      // A 3-degree deadband and bounded proportional rate prevent repeated
      // left/right correction of already acceptable heading noise.
      if (std::abs(error) <= yaw_tolerance) return std::abs(w) < 1e-9;
      if (w * error < -1e-9 || std::abs(w) > std::max(.035, .8*std::abs(error)) + 1e-9)
        return false;
    }
    return true;
  }
  bool pose_allowed(Pose predicted) const {
    if (!near) return true;
    // Evaluate every collision-checked trajectory sample, not just its endpoint.
    return distance(predicted,goal) <= (locked ? xy : distance(current,goal)+.002);
  }
  double score(Pose end, double w) const {
    if (!near) return 0;
    const double yaw_error = std::abs(angle(goal.yaw-end.yaw));
    const double remaining = distance(end,goal);
    const double bearing_error = remaining <= lock_radius || locked ? 0.0 :
      std::abs(angle(std::atan2(goal.y-end.y,goal.x-end.x)-end.yaw));
    // Continuous metre-based XY cost avoids a one-costmap-cell plateau. Heading
    // is optimized alongside XY; once locked, only zero-translation is legal.
    // Bearing prevents an orientation-only local minimum beside the goal:
    // a differential-drive robot cannot correct lateral offset while its
    // wheels already face the final yaw. XY approach therefore has priority.
    return remaining + .01*bearing_error + (locked ? .008 : .002)*yaw_error +
      .001*std::abs(w-previous_turn);
  }
};
}  // namespace waffle_navigation
