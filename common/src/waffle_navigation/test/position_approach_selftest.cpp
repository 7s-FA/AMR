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
  PositionApproach smooth;smooth.heading_guard_enabled=true;smooth.smooth_speed_enabled=true;
  const Pose destination{0,0,0};
  smooth.prepare({-.2,0,20*3.141592653589793/180},destination,.05,0);
  check(smooth.heading_hold,"Heading error must latch stop");
  check(smooth.allowed(0,0),"Braking command must remain available");
  check(!smooth.allowed(0,-.2),"Must not pivot while measured translation persists");
  check(!smooth.allowed(.03,-.1),"Must not drive an arc during heading recovery");
  smooth.prepare({-.2,0,10*3.141592653589793/180},destination,0,0);
  check(smooth.heading_hold && smooth.allowed(0,-.2),"Latch must persist until 5 degrees");
  smooth.prepare({-.2,0,4*3.141592653589793/180},destination,0,-.2);
  check(smooth.heading_hold && smooth.allowed(0,0),"Must stop residual rotation before release");
  check(!smooth.allowed(.03,0),"Rotation still moving must inhibit translation");
  smooth.prepare({-.2,0,4*3.141592653589793/180},destination,0,0);
  check(!smooth.heading_hold && smooth.allowed(.03,0),"Aligned and stationary must release");
  smooth.prepare({-.2,0,11*3.141592653589793/180},destination,0,0);
  check(!smooth.heading_hold,"5/12 degree hysteresis must prevent toggling");
  double previous=0;
  for (int i=0;i<=200;++i) {
    smooth.prepare({-i*.001,0,0},destination);
    const double cap=smooth.speed_cap();
    check(cap>=previous-1e-12 && cap<=smooth.cruise_speed+1e-12,"Speed cap must taper monotonically");
    check(cap>=.03-1e-12,"Slow cap must retain effective DWB velocity samples");
    previous=cap;
  }
  smooth.prepare({-.035,0,0},destination);
  check(!smooth.allowed(.0684288,0) && smooth.allowed(.02737152,0),"Near goal must slow without grid stall");
  std::cout<<"Burger1 heading hysteresis, stop-before-pivot/release, smooth approach and speed grid PASS\n";
  PositionApproach flowing;flowing.smooth_speed_enabled=true;
  flowing.continuous_steering_enabled=true;flowing.near_rotational_limit=.252;
  flowing.prepare({-.12,0,60*3.141592653589793/180},destination,.03,-.1);
  check(!flowing.heading_hold && flowing.allowed(.02737152,-.2),
        "Continuous 60-degree approach must allow a correcting arc without a stop latch");
  check(flowing.speed_cap()<flowing.cruise_speed,"Curved approach must reduce translation speed");
  flowing.prepare({-.12,0,90*3.141592653589793/180},destination);
  check(!flowing.allowed(.02737152,-.2),"Near sideways goal must retain pivot option");
  flowing.prepare({-.3,0,20*3.141592653589793/180},destination,.04,-.1);
  check(!flowing.heading_hold && flowing.allowed(.04,-.1),"Small errors must steer while translating");
  std::cout<<"Continuous steering, corner arc, angle-dependent taper and behind-goal guard PASS\n";
}
