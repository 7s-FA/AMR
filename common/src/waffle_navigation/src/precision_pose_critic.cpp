// ========================================================================
// 역할: Nav2 DWB 컨트롤러 평가 플러그인 PrecisionPoseCritic (우리 코드). 목표 5cm 앞까지 전진하며 자세를 함께 평가하고
//       정밀 정지를 유지한다. 계산은 precision_pose.hpp. Nav2 controller_server 가 불러 쓴다 (precision_plugins.xml).
// ========================================================================
#include "waffle_navigation/precision_pose.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace waffle_navigation {
class PrecisionPoseCritic : public dwb_core::TrajectoryCritic {
  PrecisionPolicy policy_;
public:
  // Replanning calls reset even for the same endpoint. Preserve the slow/stop
  // latch across those replans; prepare resets it when the target changes.
  void reset() override {}
  bool prepare(const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D &) override {
    bool was_near = policy_.near, was_locked = policy_.locked;
    policy_.prepare({pose.x,pose.y,pose.theta}, {goal.x,goal.y,goal.theta});
    if (auto node = node_.lock()) {
      if (!was_near && policy_.near)
        RCLCPP_INFO(node->get_logger(), "PrecisionPose: within 5cm, slow joint XY/yaw approach");
      if (!was_locked && policy_.locked)
        RCLCPP_INFO(node->get_logger(), "PrecisionPose: XY <= 1.2cm, translation locked; yaw-only fine correction");
    }
    return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override {
    if (traj.poses.empty() || !policy_.command_allowed(traj.velocity.x,traj.velocity.theta))
      throw dwb_core::IllegalTrajectoryException(name_, "Precision speed/position guard");
    for (const auto & p : traj.poses) {
      if (!policy_.pose_allowed({p.x,p.y,p.theta}))
        throw dwb_core::IllegalTrajectoryException(name_, "Precision predicted position exit");
    }
    const auto & end = traj.poses.back();
    return policy_.score({end.x,end.y,end.theta},traj.velocity.theta);
  }
  void debrief(const nav_2d_msgs::msg::Twist2D & cmd) override {
    policy_.previous_turn = cmd.theta;
  }
};
}
PLUGINLIB_EXPORT_CLASS(waffle_navigation::PrecisionPoseCritic, dwb_core::TrajectoryCritic)
