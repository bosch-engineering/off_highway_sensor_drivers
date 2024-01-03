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

#include "off_highway_uss/sender.hpp"

namespace off_highway_uss
{
Sender::Sender(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: off_highway_can::Sender(node_name, options)
{
  declare_and_get_parameters();

  fillMessageDefinitions();

  input_sub_ = create_subscription<UssInput>(
    "temperature",
    10,
    std::bind(&Sender::callback_input, this, std::placeholders::_1)
  );
}

void Sender::callback_input(const UssInput::SharedPtr msg)
{
  if (std::abs((now() - msg->header.stamp).seconds()) < allowed_age_) {
    auto & outside_temperature = messages_[outside_temperature_id_].signals["OutsideTemperature"];
    if (!outside_temperature.set(msg->temperature, "OutsideTemperature")) {
      RCLCPP_ERROR(get_logger(), "Received message out of range, will not be sent!");
      return;
    }

    send_can();
  } else {
    RCLCPP_ERROR(get_logger(), "Received message too old, will not be sent!");
  }
}

void Sender::fillMessageDefinitions()
{
  using Message = off_highway_can::Message;

  Message vehicle_data_1;
  vehicle_data_1.name = "VehicleData1";
  // Start bit, length, big endian, signed, factor, offset, min, max
  vehicle_data_1.message_counter = {8, 2, false, false, 1, 0};
  vehicle_data_1.signals["OutsideTemperature"] = {0, 7, false, false, 1, -40, -40, 87};
  messages_[outside_temperature_id_] = vehicle_data_1;
}

void Sender::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "Allowed age of USS input message to not filter it out";
  declare_parameter<double>("allowed_age", 0.2, param_desc);
  allowed_age_ = get_parameter("allowed_age").as_double();

  param_desc.description = "CAN frame id of outside temperature message";
  declare_parameter<int>("outside_temperature_id", 0xB500000);
  outside_temperature_id_ = get_parameter("outside_temperature_id").as_int();
}
}  // namespace off_highway_uss
