#include "waffle_navigation/position_approach.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace waffle_navigation {
class PositionApproachCritic : public dwb_core::TrajectoryCritic {
  PositionApproach policy_;
public:
  bool prepare(const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D &, const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D &) override {
    policy_.current={pose.x,pose.y,pose.theta};
    policy_.goal={goal.x,goal.y,0};
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
