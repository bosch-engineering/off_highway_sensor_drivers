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

#include "off_highway_common/sender.hpp"
#include "sensor_msgs/Temperature.h"

namespace off_highway_uss
{

/**
 * \brief Uss sender class to encode CAN frames. Encodes temperature input as CAN frame.
 */
class Sender : public off_highway_common::Sender
{
public:
  using UssInput = sensor_msgs::Temperature;

  /**
   * \brief Construct a new Sender object.
   */
  Sender();

  /**
   * \brief Destroy the Receiver object.
   */
  ~Sender() = default;

protected:
  /**
   * \brief Map received data to message signals, encode the messages and send them as CAN frame.
   *
   * \param msg Received message data
   */
  void callback_input(const UssInput & msg);

  /**
   * \brief Fill message definitions to encode frames of CAN node. Only stored definitions are sent.
   */
  void fillMessageDefinitions();

  ros::Subscriber input_sub_;

  FrameId outside_temperature_id_;

  /// Allowed age of input message to process
  double allowed_age_;
};

}  // namespace off_highway_uss
