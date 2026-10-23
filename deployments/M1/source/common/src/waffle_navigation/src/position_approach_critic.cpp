// ========================================================================
// 역할: Nav2 DWB 평가 플러그인 PositionApproachCritic (우리 코드). 웨이포인트에 '위치만' 맞춰 접근 (각도는 나중에 회전으로).
//       정지 또는 실제로 바퀴가 도는 최소 전진 속도만 허용. 계산은 position_approach.hpp.
// ========================================================================
#include "waffle_navigation/position_approach.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <stdexcept>
namespace waffle_navigation {
class PositionApproachCritic : public dwb_core::TrajectoryCritic {
  PositionApproach policy_;
public:
  void onInit() override {
    auto node=node_.lock();
    if (!node) throw std::runtime_error("PositionApproach node unavailable");
    const auto prefix=dwb_plugin_name_+"."+name_+".";
    auto boolean=[&](const std::string & key,bool fallback) {
      if (!node->has_parameter(prefix+key)) node->declare_parameter(prefix+key,fallback);
      return node->get_parameter(prefix+key).as_bool();
    };
    auto number=[&](const std::string & key,double fallback) {
      if (!node->has_parameter(prefix+key)) node->declare_parameter(prefix+key,fallback);
      return node->get_parameter(prefix+key).as_double();
    };
    policy_.heading_guard_enabled=boolean("heading_guard_enabled",false);
    policy_.smooth_speed_enabled=boolean("smooth_speed_enabled",false);
    policy_.continuous_steering_enabled=boolean("continuous_steering_enabled",false);
    const double start=number("heading_start_deg",5.0),stop=number("heading_stop_deg",12.0);
    policy_.cruise_speed=number("cruise_speed",.0684288);
    policy_.approach_speed=number("approach_speed",.03);
    policy_.arrival_radius=number("arrival_radius",.03);
    policy_.slowing_distance=number("slowing_distance",.15);
    policy_.near_rotational_limit=number("near_rotational_limit",.21);
    policy_.path_heading_weight=number("path_heading_weight",0.0);
    policy_.path_lookahead=number("path_lookahead",.12);
    if (!std::isfinite(start) || !std::isfinite(stop) || start<=0 || stop<=start || stop>=90 ||
        !std::isfinite(policy_.cruise_speed) || policy_.cruise_speed<policy_.drive_floor ||
        !std::isfinite(policy_.approach_speed) || policy_.approach_speed<policy_.drive_floor ||
        policy_.approach_speed>policy_.cruise_speed ||
        !std::isfinite(policy_.arrival_radius) || policy_.arrival_radius<=0 ||
        !std::isfinite(policy_.slowing_distance) || policy_.slowing_distance<=policy_.arrival_radius ||
        !std::isfinite(policy_.near_rotational_limit) || policy_.near_rotational_limit<.07 ||
        !std::isfinite(policy_.path_heading_weight) || policy_.path_heading_weight<0 ||
        !std::isfinite(policy_.path_lookahead) || policy_.path_lookahead<=.02)
      throw std::runtime_error("Invalid PositionApproach heading/speed parameters");
    policy_.heading_start=start*3.141592653589793/180.0;
    policy_.heading_stop=stop*3.141592653589793/180.0;
  }
  // Keep the heading latch across replans; prepare resets it for a new endpoint.
  void reset() override {}
  bool prepare(const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity, const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & path) override {
    policy_.prepare({pose.x,pose.y,pose.theta},{goal.x,goal.y,0},velocity.x,velocity.theta);
    std::vector<Pose> points;
    if(policy_.path_heading_weight>0) {
      points.reserve(path.poses.size());
      for(const auto & p:path.poses)points.push_back({p.x,p.y,p.theta});
    }
    policy_.prepare_path(points);
    return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override {
    if(traj.poses.empty() || !policy_.allowed(traj.velocity.x,traj.velocity.theta))
      throw dwb_core::IllegalTrajectoryException(name_,"Position approach speed/bearing guard");
    for(const auto & p:traj.poses)
      if(!policy_.pose_allowed({p.x,p.y,p.theta}))
        throw dwb_core::IllegalTrajectoryException(name_,"Position approach receding trajectory");
    const auto & p=traj.poses.back();
    return policy_.score({p.x,p.y,p.theta});
  }
};
}
PLUGINLIB_EXPORT_CLASS(waffle_navigation::PositionApproachCritic,dwb_core::TrajectoryCritic)
