// Copyright 2022 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "off_highway_radar/sender.hpp"

namespace off_highway_radar
{

Sender::Sender()
: off_highway_common::Sender()
{
  using off_highway_common::get_param_or_throw;

  allowed_age_ = get_param_or_throw<double>(private_nh_, "allowed_age");

  ego_velocity_id_ = get_param_or_throw<FrameId>(private_nh_, "ego_velocity_id");
  yaw_rate_id_ = get_param_or_throw<FrameId>(private_nh_, "yaw_rate_id");

  fillMessageDefinitions();

  input_sub_ = nh_.subscribe("velocity", 10, &Sender::callback_input, this);
}

void Sender::callback_input(const RadarInput & msg)
{
  if (abs((ros::Time::now() - msg.header.stamp).toSec()) < allowed_age_) {
    auto & v = messages_[ego_velocity_id_].signals["v"];
    if (!v.set(msg.twist.linear.x, "v")) {
      ROS_ERROR("Received message out of range, will not be send!");
      return;
    }

    auto & yaw_rate = messages_[yaw_rate_id_].signals["psidt"];
    // Convert from SI [rad/s] into sensor [deg/s]
    if (!yaw_rate.set(kRadToDegree * msg.twist.angular.z, "psidt")) {
      ROS_ERROR("Received message out of range, will not be send!");
      return;
    }

    send_can();
  } else {
    ROS_ERROR("Received message too old, will not be send!");
  }
}

void Sender::fillMessageDefinitions()
{
  Message ego_velocity;
  ego_velocity.name = "Ego_Velocity";
  // Start bit, length, big endian, signed, factor, offset, min, max
  ego_velocity.message_counter = {52, 4, false, false, 1, 0};
  ego_velocity.signals["v"] = {0, 16, false, false, 0.0025, -81.92, -81.92, 81.9175};
  messages_[ego_velocity_id_] = ego_velocity;

  Message yaw_rate;
  yaw_rate.name = "Yaw_Rate";
  // If radar SW version is older than mentioned SW version in readme use definition:
  // yaw_rate.message_counter = {52, 4, false, false, 1, 0};
  yaw_rate.message_counter = {48, 4, false, false, 1, 0};
  yaw_rate.signals["psidt"] = {0, 16, false, false, 0.005, -163.84, -163.84, 163.83};
  messages_[yaw_rate_id_] = yaw_rate;
}

}  // namespace off_highway_radar
