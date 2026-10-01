#include "waffle_navigation/precision_pose.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
#include <iostream>
#include <stdexcept>
using namespace waffle_navigation;
void check(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }

int main() {
  PrecisionPolicy p;
  p.prepare({-.051,0,0},{0,0,0});
  check(p.command_allowed(.05,.35), "Far approach unexpectedly constrained");
  p.prepare({-.05,0,0},{0,0,0});
  check(!p.command_allowed(.025,0) && p.command_allowed(.02,.105), "5cm speed bound");
  p.prepare({-.03,0,0},{0,0,0});
  check(!p.command_allowed(.015,0) && p.command_allowed(.01,0), "Distance-based slowing");
  p.prepare({-.011,0,0},{0,0,1});
  check(p.locked && !p.command_allowed(.005,0), "Final translation latch");
  check(!p.command_allowed(0,-.035) && p.command_allowed(0,.105), "Yaw correction direction");
  p.prepare({-.021,0,0},{0,0,1});
  check(!p.command_allowed(0,.105), "Final XY departure must fail");
  p.prepare({-.01,0,.98},{0,0,1});
  check(p.command_allowed(0,0) && !p.command_allowed(0,.035), "Yaw deadband");
  p.prepare({.2,0,0},{.3,0,0});
  check(!p.near && !p.locked, "Next waypoint must reset latch");

  // Load the actual installed plugin, including its ament registration and ABI.
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("waffle_navigation::PrecisionPoseCritic");
  geometry_msgs::msg::Pose2D pose, goal;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  pose.x=-.01; goal.theta=1;
  critic->prepare(pose, velocity, goal, path);
  critic->reset(); // ordinary replan must not release the translation latch
  pose.x=-.015;
  critic->prepare(pose, velocity, goal, path);
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x=.005;
  trajectory.poses.push_back(pose);
  bool rejected=false;
  try { critic->scoreTrajectory(trajectory); }
  catch(const dwb_core::IllegalTrajectoryException &) { rejected=true; }
  check(rejected,"Replanning released stop latch");
  trajectory.velocity.x=0; trajectory.velocity.theta=.105;
  check(std::isfinite(critic->scoreTrajectory(trajectory)),"Valid rotation rejected");

  // Ideal unicycle rollout: this verifies policy convergence and bounds only,
  // not AMCL noise, wheel dead zones, costmap critics, or hardware performance.
  for(double target_yaw : {0., 1.57079632679, -1.57079632679, 3.14159265359}) {
    for(double lateral : {0., .015, -.015}) {
      PrecisionPolicy policy;
      Pose current{-.045,lateral,0}, target{0,0,target_yaw};
      bool done=false;
      for(int tick=0; tick<1200; ++tick) {
        policy.prepare(current,target);
        if(distance(current,target)<=.02 && std::abs(angle(target.yaw-current.yaw))<=policy.yaw_tolerance) {
          done=true; break;
        }
        double best=1e99, bv=0,bw=0;
        for(int iv=0;iv<=4;++iv) for(int iw=-3;iw<=3;++iw) {
          double v=.005*iv,w=.035*iw;
          if(!policy.command_allowed(v,w)) continue;
          Pose end=current; bool valid=true;
          for(int step=0;step<6;++step) {
            end.x += v*std::cos(end.yaw)*.1; end.y += v*std::sin(end.yaw)*.1;
            end.yaw=angle(end.yaw+w*.1);
            valid=valid && policy.pose_allowed(end);
          }
          if(!valid) continue;
          double score=policy.score(end,w);
          if(score<best) {best=score;bv=v;bw=w;}
        }
        check(best<1e98,"No valid precision candidate");
        current.x+=bv*std::cos(current.yaw)*.1;
        current.y+=bv*std::sin(current.yaw)*.1;
        current.yaw=angle(current.yaw+bw*.1); policy.previous_turn=bw;
      }
      if(!done) std::cerr<<"failed rollout yaw="<<target_yaw<<" lateral="<<lateral<<" at "<<current.x<<","<<current.y<<","<<current.yaw<<"\n";
      check(done,"Ideal rollout failed to converge");
    }
  }
  std::cout<<"precision bounds, plugin load, replan latch, 12 ideal rollouts: PASS\n";
}
