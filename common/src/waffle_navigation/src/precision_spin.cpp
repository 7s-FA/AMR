// Precision speed envelope around Nav2's Spin. Nav2 retains TF, timeout,
// cancellation, collision checking, command publication and stop handling.
#include "nav2_behaviors/plugins/spin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "waffle_navigation/precision_spin_speed.hpp"

namespace waffle_navigation {
class PrecisionSpin : public nav2_behaviors::Spin {
public:
  void onConfigure() override {
    nav2_behaviors::Spin::onConfigure();
    maximum_ = max_rotational_vel_;
    minimum_ = min_rotational_vel_;
  }
  nav2_behaviors::ResultStatus onCycleUpdate() override {
    const double remaining = std::max(0.0, std::abs(cmd_yaw_) - std::abs(relative_yaw_));
    max_rotational_vel_ = precision_spin_cap(remaining, maximum_);
    min_rotational_vel_ = std::min(minimum_, std::min(0.06, max_rotational_vel_));
    return nav2_behaviors::Spin::onCycleUpdate();
  }
protected:
  double maximum_{0.0};
  double minimum_{0.0};
};
// Separate departure action: only the one-shot exit turn uses this ceiling.
// The ordinary and precise waypoint turns retain their original limits.
class DepartureSpin : public PrecisionSpin {
public:
  void onConfigure() override {
    PrecisionSpin::onConfigure();
    auto node=node_.lock();
    if (!node) throw std::runtime_error("DepartureSpin node unavailable");
    const std::string key="departure_spin.max_rotational_vel";
    if (!node->has_parameter(key)) node->declare_parameter(key,maximum_);
    maximum_=node->get_parameter(key).as_double();
    if (!std::isfinite(maximum_) || maximum_<=0 || maximum_>0.924)
      throw std::runtime_error("Invalid departure rotation limit");
    max_rotational_vel_=maximum_;
  }
  nav2_behaviors::ResultStatus onRun(
    const std::shared_ptr<const SpinActionGoal> command) override {
    auto result=nav2_behaviors::Spin::onRun(command);
    if (result.status!=nav2_behaviors::Status::SUCCEEDED) return result;
    started_=clock_->now();
    duration_=4.4*std::abs(command->target_yaw)/M_PI;
    direction_=command->target_yaw>=0 ? 1 : -1;
    RCLCPP_INFO(logger_, "Timed departure turn: %.3f rad/s for %.3f s (not measured yaw)",
      direction_*maximum_,duration_);
    return result;
  }
  nav2_behaviors::ResultStatus onCycleUpdate() override {
    using nav2_behaviors::Status;
    const auto now=clock_->now();
    if (now>=end_time_) return fail("Timed departure turn timeout");
    const double elapsed=(now-started_).seconds();
    if (elapsed>=duration_) {
      stopRobot();
      return {Status::SUCCEEDED};
    }
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(pose,*tf_,local_frame_,robot_base_frame_,transform_tolerance_))
      return fail("No pose for departure collision check");
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x=pose.pose.position.x; pose2d.y=pose.pose.position.y;
    pose2d.theta=tf2::getYaw(pose.pose.orientation);
    // Actual TF yaw is feedback only; elapsed time controls completion.
    double yaw_delta=pose2d.theta-prev_yaw_;
    if (yaw_delta>M_PI) yaw_delta-=2*M_PI;
    if (yaw_delta<-M_PI) yaw_delta+=2*M_PI;
    relative_yaw_+=yaw_delta; prev_yaw_=pose2d.theta;
    feedback_->angular_distance_traveled=relative_yaw_;
    action_server_->publish_feedback(feedback_);
    const double remaining=std::max(0.0,duration_-elapsed)*maximum_;
    auto velocity=std::make_unique<geometry_msgs::msg::TwistStamped>();
    velocity->header.stamp=now; velocity->header.frame_id=robot_base_frame_;
    velocity->twist.angular.z=direction_*maximum_;
    if (!isCollisionFree(remaining,velocity->twist,pose2d)) {
      stopRobot(); return {Status::FAILED,SpinActionResult::COLLISION_AHEAD};
    }
    vel_pub_->publish(std::move(velocity));
    return {Status::RUNNING};
  }
private:
  nav2_behaviors::ResultStatus fail(const char *reason) {
    stopRobot(); RCLCPP_ERROR(logger_,"ENCODER_DEPARTURE_FAILED: %s",reason);
    return {nav2_behaviors::Status::FAILED,SpinActionResult::UNKNOWN};
  }
  rclcpp::Time started_{0,0,RCL_ROS_TIME};
  double duration_{0.0};
  int direction_{1};
};
}
PLUGINLIB_EXPORT_CLASS(waffle_navigation::PrecisionSpin, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(waffle_navigation::DepartureSpin, nav2_core::Behavior)
