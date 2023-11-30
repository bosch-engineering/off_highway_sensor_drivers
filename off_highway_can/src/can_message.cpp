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

#include "off_highway_can/can_message.hpp"

#include "rclcpp/rclcpp.hpp"

namespace off_highway_can
{
bool Signal::set(double value, const std::string & signal_name)
{
  if (value < min || value > max) {
    RCLCPP_WARN(
      rclcpp::get_logger("Signal"),
      "Input %f for signal '%s' is out of range [%f, %f]!",
      value, signal_name.c_str(), min, max
    );
    return false;
  }

  this->value = value;
  return true;
}

void MessageCounter::increase()
{
  first = false;
  value = (static_cast<uint32_t>(value) + 1) % (0x01 << length);
}
}  // namespace off_highway_can
