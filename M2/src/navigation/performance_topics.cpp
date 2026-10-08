// Read-only serialized-topic observer; no publishers, services or motor commands.
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <deque>
#include <fstream>
#include <map>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <cstdio>
using json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static double now_s(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static int64_t cpu_ns(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
static json quantiles(const std::deque<double>& q){
 if(q.empty())return json::object();std::vector<double> v(q.begin(),q.end());std::sort(v.begin(),v.end());
 return {{"p50",v[v.size()/2]},{"p95",v[size_t(.95*(v.size()-1))]},{"max",v.back()}};
}
struct Stats{uint64_t count=0;double last=0,max_gap=0,max_age=-1e12;size_t bytes=0;int64_t cpu=0;std::deque<double> ages,gaps;};
static void bounded(std::deque<double>& q,double v){q.push_back(v);if(q.size()>600)q.pop_front();}
int main(int argc,char**argv){
 if(argc!=3)return 2;std::string robot=argv[1],out=argv[2];if(robot!="burger2"&&robot!="burger2")return 2;
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("data_topic_observer","/"+robot);
 std::map<std::string,Stats> stats;json dock=json::object(),ir=nullptr;double dock_at=0;
 std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
 std::map<std::string,std::string> types={{"odom","nav_msgs/msg/Odometry"},{"scan","sensor_msgs/msg/LaserScan"},{"sensor_state","turtlebot3_msgs/msg/SensorState"},{"imu","sensor_msgs/msg/Imu"},{"joint_states","sensor_msgs/msg/JointState"},{"battery_state","sensor_msgs/msg/BatteryState"},{"magnetic_field","sensor_msgs/msg/MagneticField"},{"docking/status","std_msgs/msg/String"},{"ir/high","std_msgs/msg/Bool"}};
 for(const auto& [name,type]:types){
  auto qos=rclcpp::QoS(rclcpp::KeepLast(1));if(name!="docking/status"&&name!="ir/high")qos.best_effort();
  subscriptions.push_back(node->create_generic_subscription(name,type,qos,[&,name](std::shared_ptr<rclcpp::SerializedMessage> msg){
   const auto start=cpu_ns();double now=now_s();auto& s=stats[name];const auto& raw=msg->get_rcl_serialized_message();
   if(s.last){double gap=(now-s.last)*1000;bounded(s.gaps,gap);s.max_gap=std::max(s.max_gap,gap);}s.last=now;++s.count;s.bytes=raw.buffer_length;
   if(raw.buffer_length>=12&&raw.buffer[1]==1&&name!="docking/status"&&name!="ir/high"){
    int32_t sec;uint32_t nano;std::memcpy(&sec,raw.buffer+4,4);std::memcpy(&nano,raw.buffer+8,4);
    double age=(node->now().nanoseconds()-int64_t(sec)*1000000000-int64_t(nano))/1e6;
    if(std::isfinite(age)){bounded(s.ages,age);s.max_age=std::max(s.max_age,age);}
   }
   if(name=="ir/high"&&raw.buffer_length>=5)ir=raw.buffer[4]!=0;
   if(name=="docking/status"&&raw.buffer_length>=8&&raw.buffer[1]==1){
    uint32_t len;std::memcpy(&len,raw.buffer+4,4);
    if(len>0&&len<=raw.buffer_length-8){auto value=json::parse(std::string(reinterpret_cast<char*>(raw.buffer+8),len-1),nullptr,false);if(!value.is_discarded()){dock=value;dock_at=now;}}
   }
   s.cpu+=cpu_ns()-start;
  }));
 }
 double began=now_s(),previous=began;std::map<std::string,uint64_t> previous_counts;
 auto save=[&](){double now=now_s();json topics=json::object();
  for(const auto& [name,s]:stats){std::string label=name=="docking/status"?"dock_status":name=="ir/high"?"ir_high":name;
   topics[label]={{"count",s.count},{"hz",(s.count-previous_counts[name])/std::max(.001,now-previous)},{"last_age_ms",(now-s.last)*1000},{"source_age_ms",quantiles(s.ages)},{"gap_ms",quantiles(s.gaps)},{"max_gap_ms",s.max_gap},{"sample_bytes",s.bytes},{"observer_callback_cpu_ms_total",s.cpu/1e6}};
   if(s.max_age>-1e11)topics[label]["max_source_age_ms"]=s.max_age;previous_counts[name]=s.count;
  }
  json value={{"elapsed_s",now-began},{"monotonic",now},{"topics",topics},{"dock",dock},{"dock_age_s",dock_at?now-dock_at:1e9},{"ir",ir}};
  std::ofstream file(out+".tmp");file<<value.dump();file.close();std::rename((out+".tmp").c_str(),out.c_str());previous=now;
 };
 auto timer=node->create_wall_timer(std::chrono::seconds(1),save);
 rclcpp::spin(node);save();rclcpp::shutdown();return 0;
}
