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

#pragma once

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "geometry_msgs/TwistStamped.h"

#include "off_highway_common/sender.hpp"
#include "off_highway_common/helper.hpp"

namespace off_highway_radar
{

/**
 * \brief Radar sender class to encode CAN frames. Encodes twist (forward velocity and yaw rate)
 * input as CAN frame.
 */
class Sender : public off_highway_common::Sender
{
public:
  using Message = off_highway_common::Message;
  using RadarInput = geometry_msgs::TwistStamped;

  /**
   * \brief Construct a new Sender object.
   */
  Sender();

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
  void callback_input(const RadarInput & msg);

  /**
   * \brief Fill message definitions to encode frames of CAN node. Only stored definitions are sent.
   */
  void fillMessageDefinitions();

  ros::Subscriber input_sub_;

  FrameId ego_velocity_id_;
  FrameId yaw_rate_id_;

  /// Allowed age of input message to process
  double allowed_age_;

  static constexpr double kRadToDegree = 180. / M_PI;
};

}  // namespace off_highway_radar
