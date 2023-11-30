// Copyright 2022 Robert Bosch GmbH and its subsidiaries
// Copyright 2023 digital workbench GmbH
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

#include "off_highway_radar/sender.hpp"

namespace off_highway_radar
{
Sender::Sender(const std::string & node_name)
: off_highway_can::Sender(node_name)
{
  declare_and_get_parameters();

  fillMessageDefinitions();

  input_sub_ = create_subscription<RadarInput>(
    "velocity",
    10,
    std::bind(&Sender::callback_input, this, std::placeholders::_1)
  );
}

void Sender::callback_input(const RadarInput::SharedPtr msg)
{
  if (std::abs((now() - msg->header.stamp).seconds()) < allowed_age_) {
    auto & v = messages_[ego_velocity_id_].signals["v"];
    if (!v.set(msg->twist.linear.x, "v")) {
      RCLCPP_ERROR(get_logger(), "Received message out of range, will not be sent!");
      return;
    }

    auto & yaw_rate = messages_[yaw_rate_id_].signals["psidt"];
    // Convert from SI [rad/s] into sensor [deg/s]
    if (!yaw_rate.set(kRadToDegree * msg->twist.angular.z, "psidt")) {
      RCLCPP_ERROR(get_logger(), "Received message out of range, will not be sent!");
      return;
    }

    send_can();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Received message too old, will not be sent!");
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

void Sender::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "Allowed age of USS input message to not filter it out";
  declare_parameter<double>("allowed_age", 0.2, param_desc);
  allowed_age_ = get_parameter("allowed_age").as_double();

  param_desc.description = "CAN frame id of ego velocity message";
  declare_parameter<int>("ego_velocity_id", 0x50);
  ego_velocity_id_ = get_parameter("ego_velocity_id").as_int();

  param_desc.description = "CAN frame id of yaw rate message";
  declare_parameter<int>("yaw_rate_id", 0x174);
  yaw_rate_id_ = this->get_parameter("yaw_rate_id").as_int();
}
}  // namespace off_highway_radar
