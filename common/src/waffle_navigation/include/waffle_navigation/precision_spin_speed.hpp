// ========================================================================
// 역할: 회전 속도 곡선·엔코더 계산 (PrecisionSpin/DepartureSpin 용).
// ========================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace waffle_navigation {
constexpr double encoder_metres_per_tick = 0.033 * 0.001533981;
inline int64_t encoder_tick_delta(int32_t current, int32_t start) {
  int64_t value=int64_t(current)-int64_t(start);
  if (value>INT32_MAX) value-=4294967296LL;
  if (value<INT32_MIN) value+=4294967296LL;
  return value;
}
inline int64_t encoder_turn_target(double radians) {
  return std::llround(std::abs(radians)*0.160/(2*encoder_metres_per_tick));
}
inline bool encoder_turn_invalid(int64_t left, int64_t right, int64_t target) {
  return std::min(left,right)<-100 || std::abs(left-right)>200 ||
         std::max(left,right)>target+100;
}
// Keep the existing large-angle cap, then smoothly reduce the cap inside 15 deg.
// The floor avoids asking the drive to rotate below an effective motor speed.
inline double precision_spin_cap(double remaining, double maximum) {
  constexpr double zone = 15.0 * 3.14159265358979323846 / 180.0;
  constexpr double floor = 0.06;
  const double cap = std::max(0.0, maximum);
  const double low = std::min(floor, cap);
  const double fraction = std::clamp(std::abs(remaining) / zone, 0.0, 1.0);
  return low + (cap - low) * fraction * fraction;
}
}
