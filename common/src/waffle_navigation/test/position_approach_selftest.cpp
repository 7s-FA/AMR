#include "waffle_navigation/position_approach.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_loader.hpp"
#include <iostream>
#include <stdexcept>
using namespace waffle_navigation;
void check(bool ok,const char * why) {if(!ok) throw std::runtime_error(why);}
int main() {
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core","dwb_core::TrajectoryCritic");
  auto critic=loader.createSharedInstance("waffle_navigation::PositionApproachCritic");
  geometry_msgs::msg::Pose2D p,g;
  p.x=.401613; p.y=.359739; p.theta=82.9856*3.141592653589793/180;
  g.x=.43906513; g.y=.360654172; g.theta=1.584076;
  nav_2d_msgs::msg::Twist2D vel; nav_2d_msgs::msg::Path2D path;
  critic->prepare(p,vel,g,path);
  dwb_msgs::msg::Trajectory2D t; t.poses.push_back(p);
  for(double v:{.005,.01,.015,.02,.025}) {
    t.velocity.x=v;t.velocity.theta=0;bool rejected=false;
    try{critic->scoreTrajectory(t);}catch(const dwb_core::IllegalTrajectoryException&){rejected=true;}
    check(rejected,"Recorded sideways miss must not continue straight");
  }
  t.velocity.x=0;t.velocity.theta=-.21;
  double first=critic->scoreTrajectory(t);
  g.theta=-1.5;critic->prepare(p,vel,g,path);
  check(first==critic->scoreTrajectory(t),"Saved yaw affected position approach");
  t.velocity.theta=.21;bool rejected=false;
  try{critic->scoreTrajectory(t);}catch(const dwb_core::IllegalTrajectoryException&){rejected=true;}
  check(rejected,"Wrong-direction steering accepted");

  // Ideal rollout of the recorded failure plus varied lateral/heading starts.
  // No wheel, localization, obstacle or other-critic model: not a hardware test.
  int count=0;
  for(double heading:{0.,1.448372,-1.448372,3.141592653589793})
    for(double lateral:{-.03,0.,.03}) {
      Pose current{-.08,lateral,heading},target{0,0,0};
      if(count==0){current={p.x,p.y,p.theta};target={g.x,g.y,0};}
      bool done=false;
      for(int tick=0;tick<1000;++tick) {
        if(distance(current,target)<=.02){done=true;break;}
        PositionApproach policy;policy.current=current;policy.goal=target;
        double best=1e99,bv=0,bw=0;
        for(int iv=0;iv<=10;++iv) for(int iw=-10;iw<=10;++iw) {
          double v=.005*iv,w=.035*iw;
          if(!policy.allowed(v,w))continue;
          Pose end=current;bool valid=true;
          for(int step=0;step<6;++step){end.x+=v*std::cos(end.yaw)*.1;end.y+=v*std::sin(end.yaw)*.1;end.yaw=angle(end.yaw+w*.1);valid=valid&&policy.pose_allowed(end);}
          double score=policy.score(end);
          if(valid&&score<best){best=score;bv=v;bw=w;}
        }
        check(best<1e98,"No valid steering candidate");
        check(bv==0 || bv>=.025-1e-9,"Stalled speed chosen");
        current.x+=bv*std::cos(current.yaw)*.1;current.y+=bv*std::sin(current.yaw)*.1;current.yaw=angle(current.yaw+bw*.1);
      }
      check(done,"Position rollout did not arrive");++count;
    }
  std::cout<<"PositionApproach: plugin, recorded miss, yaw independence, speed floor, 12 ideal rollouts PASS\n";
}
