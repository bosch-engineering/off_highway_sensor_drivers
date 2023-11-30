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

#pragma once

#include <memory>
#include <string>

#include "off_highway_can/sender.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace off_highway_radar
{
/**
 * \brief Radar sender class to encode CAN frames. Encodes twist (forward velocity and yaw rate)
 * input as CAN frame.
 */
class Sender : public off_highway_can::Sender
{
public:
  using Message = off_highway_can::Message;
  using RadarInput = geometry_msgs::msg::TwistStamped;

  /**
   * \brief Construct a new Sender object.
   */
  explicit Sender(const std::string & node_name = "sender");

  /**
   * \brief Destroy the Sender object.
   */
  ~Sender() = default;

protected:
  /**
   * \brief Map received data to message signals, encode the messages and send them as CAN frame.
   *
   * \param msg Received message data
   */
  void callback_input(const RadarInput::SharedPtr msg);

  /**
   * \brief Fill message definitions to encode frames of CAN node. Only stored definitions are sent.
   */
  void fillMessageDefinitions();

  rclcpp::Subscription<RadarInput>::SharedPtr input_sub_;

  uint32_t ego_velocity_id_;
  uint32_t yaw_rate_id_;

  /// Allowed age of input message to process
  double allowed_age_;

  static constexpr double kRadToDegree = 180.0 / M_PI;

private:
  /**
   * \brief Declare and get node parameters
   */
  void declare_and_get_parameters();
};
}  // namespace off_highway_radar
