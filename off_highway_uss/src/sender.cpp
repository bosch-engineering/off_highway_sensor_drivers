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

#include "off_highway_uss/sender.hpp"

#include "off_highway_common/helper.hpp"

namespace off_highway_uss
{

Sender::Sender()
: off_highway_common::Sender()
{
  using off_highway_common::get_param_or_throw;

  allowed_age_ = get_param_or_throw<double>(private_nh_, "allowed_age");

  outside_temperature_id_ = get_param_or_throw<FrameId>(private_nh_, "outside_temperature_id");

  fillMessageDefinitions();

  input_sub_ = nh_.subscribe("temperature", 10, &Sender::callback_input, this);
}

void Sender::callback_input(const UssInput & msg)
{
  if (abs((ros::Time::now() - msg.header.stamp).toSec()) < allowed_age_) {
    auto & outside_temperature = messages_[outside_temperature_id_].signals["OutsideTemperature"];
    if (!outside_temperature.set(msg.temperature, "OutsideTemperature")) {
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
  using Message = off_highway_common::Message;

  Message vehicle_data_1;
  vehicle_data_1.name = "VehicleData1";
  // Start bit, length, big endian, signed, factor, offset, min, max
  vehicle_data_1.message_counter = {8, 2, false, false, 1, 0};
  vehicle_data_1.signals["OutsideTemperature"] = {0, 7, false, false, 1, -40, -40, 87};
  messages_[outside_temperature_id_] = vehicle_data_1;
}
}  // namespace off_highway_uss
