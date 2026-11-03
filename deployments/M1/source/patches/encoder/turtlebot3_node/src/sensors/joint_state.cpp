// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include <array>
#include <algorithm>
#include <cmath>

#include <memory>
#include <string>
#include <utility>

#include "turtlebot3_node/sensors/joint_state.hpp"

using robotis::turtlebot3::sensors::JointState;

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);
  nh_->get_parameter_or<std::string>(
    "namespace",
    name_space_,
    std::string(""));

  if (name_space_ != "") {
    frame_id_ = name_space_ + "/" + frame_id_;
    wheel_right_joint_ = name_space_ + "/" + wheel_right_joint_;
    wheel_left_joint_ = name_space_ + "/" + wheel_left_joint_;
  }
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void JointState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  std::array<int32_t, JOINT_NUM> position =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_left.addr,
      extern_control_table.present_position_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_position_right.addr,
      extern_control_table.present_position_right.length)};

  std::array<int32_t, JOINT_NUM> velocity =
  {dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_left.addr,
      extern_control_table.present_velocity_left.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      extern_control_table.present_velocity_right.addr,
      extern_control_table.present_velocity_right.length)};

  // Reject complete samples before updating the accepted encoder baseline.
  // 20 rad/s is about 0.66 m/s at the Burger's 33 mm wheel radius.
  // After a gap, require five consecutive plausible samples before resuming.
  constexpr double max_wheel_rad_s = 20.0;
  constexpr double max_gap_s = 0.25;
  constexpr double tick_margin = 8.0;
  const int64_t stamp = now.nanoseconds();

  if (encoder_initialized_ && stamp <= accepted_stamp_ns_) {return;}
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
    return;
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
    if (recovery_count_ < 5) {return;}
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
  msg->header.frame_id = this->frame_id_;
  msg->header.stamp = now;
  msg->name = {wheel_left_joint_, wheel_right_joint_};
  for (size_t i = 0; i < JOINT_NUM; ++i) {
    msg->position.push_back(TICK_TO_RAD * accumulated_ticks_[i]);
    msg->velocity.push_back(RPM_TO_MS * velocity[i]);
  }
  pub_->publish(std::move(msg));
}
