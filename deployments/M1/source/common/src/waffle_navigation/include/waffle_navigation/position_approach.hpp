// ========================================================================
// 역할: 위치 전용 접근 계산(순수 기하). PositionApproachCritic 과 시험이 같이 쓴다.
// ========================================================================
#pragma once
#include <cmath>
#include <algorithm>
#include <vector>
#include "waffle_navigation/precision_pose.hpp"

namespace waffle_navigation {
// Position-only approach. Saved waypoint yaw is deliberately never used.
class PositionApproach {
public:
  Pose current{}, goal{};
  static constexpr double drive_floor = .025, radius = .15;
  // Disabled defaults preserve other robot profiles. Burger1 enables these
  // through critic parameters, without changing Action or docking control.
  bool heading_guard_enabled{false}, heading_hold{false}, smooth_speed_enabled{false};
  bool continuous_steering_enabled{false};
  double heading_start{5.0*3.141592653589793/180.0};
  double heading_stop{12.0*3.141592653589793/180.0};
  double cruise_speed{.0684288}, approach_speed{.03}, arrival_radius{.03}, slowing_distance{.15};
  double near_rotational_limit{.21};
  double path_heading_weight{0}, path_lookahead{.12};
  Pose path_target{};
  bool have_path_target{false};
  double measured_v{0}, measured_w{0};
  bool have_goal{false};
  void prepare(Pose pose, Pose target, double velocity=0, double rotation=0) {
    if (!have_goal || distance(goal,target) > 1e-5) heading_hold=false;
    current=pose;goal=target;have_goal=true;measured_v=velocity;measured_w=rotation;
    if (!heading_guard_enabled || distance(current,goal)<=arrival_radius) {
      heading_hold=false;
    } else {
      const double error=std::abs(bearing(current));
      if (error>heading_stop) heading_hold=true;
      else if (error<=heading_start && std::abs(measured_v)<=.008 &&
               std::abs(measured_w)<=.03) heading_hold=false;
    }
  }
  double speed_cap() const {
    if (!smooth_speed_enabled) return 1e9;
    const double fraction=std::clamp((distance(current,goal)-arrival_radius)/
                                    (slowing_distance-arrival_radius),0.0,1.0);
    const double blend=fraction*fraction*(3.0-2.0*fraction);
    double cap=approach_speed+(cruise_speed-approach_speed)*blend;
    // Slow a curved approach without a stop/realign latch. Retain a sampled
    // wheel-driving command until the target is substantially behind us.
    if (continuous_steering_enabled)
      cap=std::max(approach_speed,cap*std::max(0.0,std::cos(bearing(current))));
    return cap;
  }
  double bearing(Pose pose) const {
    return angle(std::atan2(goal.y-pose.y,goal.x-pose.x)-pose.yaw);
  }
  // Interpolate by arc length, so sparse planner points do not create a
  // heading dead zone at a corner. This only adds a continuous soft cost.
  void prepare_path(const std::vector<Pose> & path) {
    have_path_target=false;
    if(path_heading_weight<=0 || path.size()<2) return;
    double nearest=1e100, fraction=0; size_t segment=0;
    for(size_t i=1;i<path.size();++i) {
      const auto a=path[i-1], b=path[i];
      if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(b.x)||!std::isfinite(b.y))return;
      const double dx=b.x-a.x,dy=b.y-a.y, length2=dx*dx+dy*dy;
      if(length2<1e-12)continue;
      const double t=std::clamp(((current.x-a.x)*dx+(current.y-a.y)*dy)/length2,0.,1.);
      const double d=std::hypot(current.x-a.x-t*dx,current.y-a.y-t*dy);
      if(d<nearest){nearest=d;segment=i;fraction=t;}
    }
    if(!segment)return;
    double remaining=path_lookahead;
    for(size_t i=segment;i<path.size();++i) {
      const auto a=path[i-1],b=path[i];const double start=i==segment?fraction:0.;
      const double length=distance(a,b), available=length*(1-start);
      if(length>1e-9 && remaining<=available) {
        const double t=start+remaining/length;
        path_target={a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0};
        have_path_target=true;return;
      }
      remaining-=available;
    }
    path_target=path.back();have_path_target=true;
  }
  bool allowed(double v, double w) const {
    if (!std::isfinite(v) || !std::isfinite(w) || v < -1e-9) return false;
    // Keep zero translation for steering, but reject the observed wheel-stall
    // interval. Do not impose a positive min_vel_x that removes rotation.
    if (v > 1e-9 && v < drive_floor-1e-9) return false;
    if (heading_hold && (std::abs(measured_v)>.008 ||
                         std::abs(bearing(current))<=heading_start))
      return std::abs(v)<=1e-9 && std::abs(w)<=1e-9;
    if (v>speed_cap()+1e-9 || (heading_hold && v>1e-9)) return false;
    if (heading_hold && v<=1e-9 &&
        (std::abs(w)<.07-1e-9 || w*bearing(current)<=0)) return false;
    if (distance(current,goal)>radius) return true;
    const double error = bearing(current);
    if (std::abs(w)>near_rotational_limit+1e-9) return false;
    const double steering_limit=(continuous_steering_enabled ? 80.0 : 35.0);
    if (v>1e-9 && std::abs(error)>steering_limit*3.141592653589793/180.0) return false;
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
    if (distance(current,goal)>radius) {
      if(heading_hold)return .035*std::abs(bearing(end));
      if(have_path_target && distance(end,path_target)>.02)
        return path_heading_weight*std::abs(angle(
          std::atan2(path_target.y-end.y,path_target.x-end.x)-end.yaw));
      return 0;
    }
    // A continuous XY cost avoids cell plateaus. The target is the coordinate,
    // not its final parking orientation. Suppress bearing singularity inside
    // the unchanged 2cm arrival tolerance.
    double heading = distance(end,goal)<=(smooth_speed_enabled ? arrival_radius : .02)
                     ? 0 : std::abs(bearing(end));
    return distance(end,goal)+.035*heading;
  }
};
}
