#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cassert>
#include <iostream>
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
constexpr int JOINT_NUM=2;
constexpr double RPM_TO_MS=0.229*0.0034557519189487725, TICK_TO_RAD=0.001533981;
struct Filter {
std::array<int32_t,2> last_position_{}, recovery_position_{};
std::array<int64_t,2> accumulated_ticks_{};
int64_t accepted_stamp_ns_=0,recovery_stamp_ns_=0;
bool encoder_initialized_=false;
unsigned recovery_count_=0;
bool sample(int64_t stamp_in, std::array<int32_t,2> position, std::array<int32_t,2> velocity={0,0}) {
  constexpr double max_wheel_rad_s = 20.0;
  constexpr double max_gap_s = 0.25;
  constexpr double tick_margin = 8.0;
  const int64_t stamp = stamp_in;

  if (encoder_initialized_ && stamp <= accepted_stamp_ns_) {return false;}
  const double dt = encoder_initialized_ ? (stamp - accepted_stamp_ns_) * 1e-9 : 0.0;
  const bool recovering = encoder_initialized_ && dt > max_gap_s;
  std::array<int64_t, JOINT_NUM> delta{};
  bool valid = true;
  for (size_t i = 0; i < JOINT_NUM; ++i) {
    // Signed modular delta handles an int32 encoder wrap without overflow.
    int64_t d = static_cast<int64_t>(position[i]) - last_position_[i];
    if (d > INT32_MAX) {d -= (INT64_C(1) << 32);}
    if (d < INT32_MIN) {d += (INT64_C(1) << 32);}
    delta[i] = d;
    if (std::abs(static_cast<double>(velocity[i]) * RPM_TO_MS) > max_wheel_rad_s * 0.033) {
      valid = false;
    }
    if (encoder_initialized_ && std::abs(static_cast<double>(d)) >
      max_wheel_rad_s * std::min(dt, 1.0) / TICK_TO_RAD + tick_margin) {valid = false;}
  }
  if (!valid) {
    recovery_count_ = 0;
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
      "Rejected implausible encoder sample; retaining last valid baseline");
    return false;
  }
  if (recovering) {
    bool consecutive = recovery_count_ > 0 && stamp > recovery_stamp_ns_ &&
      (stamp - recovery_stamp_ns_) * 1e-9 <= max_gap_s;
    if (consecutive) {
      const double step_dt = (stamp - recovery_stamp_ns_) * 1e-9;
      for (size_t i = 0; i < JOINT_NUM; ++i) {
        int64_t step = static_cast<int64_t>(position[i]) - recovery_position_[i];
        if (step > INT32_MAX) {step -= (INT64_C(1) << 32);}
        if (step < INT32_MIN) {step += (INT64_C(1) << 32);}
        if (std::abs(static_cast<double>(step)) >
          max_wheel_rad_s * step_dt / TICK_TO_RAD + tick_margin) {consecutive = false;}
      }
    }
    recovery_count_ = consecutive ? recovery_count_ + 1 : 1;
    recovery_position_ = position;
    recovery_stamp_ns_ = stamp;
    if (recovery_count_ < 5) {return false;}
    // Preserve bounded measured travel across the gap, never rebase to an
    // arbitrary corrupt count and never publish a fabricated zero position.
    RCLCPP_WARN(nh_->get_logger(), "Encoder stream recovered after five plausible samples");
  }
  recovery_count_ = 0;
  if (encoder_initialized_) {
    for (size_t i = 0; i < JOINT_NUM; ++i) {accumulated_ticks_[i] += delta[i];}
  }
  last_position_ = position;
  accepted_stamp_ns_ = stamp;
  encoder_initialized_ = true;
return true;}};
int main(){
Filter f; assert(f.sample(1000000000,{0,0}));
assert(f.sample(1050000000,{-10,-10}));
assert(!f.sample(1100000000,{1200000000,-10}));
assert(f.sample(1150000000,{-20,-20}));
for(int i=0;i<4;i++) assert(!f.sample(1600000000LL+i*50000000LL,{-30-i,-30-i}));
assert(f.sample(1800000000LL,{-34,-34}));
assert(f.accumulated_ticks_[0]==-34);
assert(!f.sample(2200000000LL,{-34,-34},{1000000,0}));
for(int i=0;i<4;i++) assert(!f.sample(2250000000LL+i*50000000LL,{-34,-34}));
assert(f.sample(2450000000LL,{-34,-34}));
assert(f.accumulated_ticks_[0]==-34);
Filter w;assert(w.sample(1000000000,{INT32_MAX-2,0}));
assert(w.sample(1050000000,{INT32_MIN+2,0}));assert(w.accumulated_ticks_[0]==5);
std::cout<<"PASS: spike rejection, gap recovery, velocity rejection, position continuity, rollover\n";
}
