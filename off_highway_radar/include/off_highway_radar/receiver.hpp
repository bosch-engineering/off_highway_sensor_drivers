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

#include <memory>
#include <optional>

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include "off_highway_common/receiver.hpp"
#include "off_highway_radar_msgs/Object.h"
#include "off_highway_radar_msgs/Objects.h"
#include "off_highway_radar_msgs/Information.h"

namespace off_highway_radar
{

/**
 * \brief Radar receiver class to decode CAN frames into object list to publish.
 *
 * Object list is published as simple list or as point cloud. Sensor information frame is used as
 * diagnostic input and also published directly.
 */
class Receiver : public off_highway_common::Receiver
{
public:
  using Message = off_highway_common::Message;
  using Object = off_highway_radar_msgs::Object;
  using ObjectA = off_highway_radar_msgs::ObjectA;
  using ObjectB = off_highway_radar_msgs::ObjectB;
  using Objects = off_highway_radar_msgs::Objects;
  using Information = off_highway_radar_msgs::Information;

  /**
   * \brief Construct a new Receiver object.
   */
  Receiver();

  /**
   * \brief Destroy the Receiver object.
   */
  ~Receiver() = default;

private:
  // API to fill
  /**
   * \brief Fill message definitions to decode frames of CAN node. Only stored definitions are
   * processed.
   *
   * \return Messages of CAN node to decode and process
   */
  Messages fillMessageDefinitions() override;

  /**
   * \brief Process CAN message (e.g. convert into own data type).
   *
   * \param header Header of corresponding ROS message
   * \param id Id of respective CAN frame
   * \param message Decoded message (values) of frame to use for processing
   */
  void process(std_msgs::Header header, const FrameId & id, Message & message) override;

  /**
   * \brief Check if object is invalid.
   *
   * \param object Object to check
   * \return True if object is invalid and should be filtered, false otherwise
   */
  bool filter(const Object & object);

  /**
   * \brief Manage object list and publish it.
   */
  void manage_and_publish(const ros::TimerEvent &);

  /**
   * \brief Filter objects and remove too old objects or too old B message information from objects.
   */
  void manage_objects();

  /**
   * \brief Publish objects as list.
   */
  void publish_objects();

  /**
   * \brief Publish objects as point cloud.
   */
  void publish_pcl();

  /**
   * \brief Update diagnostics status by checking last sensor information.
   *
   * Uses sensor blind, SW / HW / CAN / config fail, a set DTC, sensor not safe and reception error
   * of sensor information message to indicate sensor error.
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  static constexpr uint8_t kCountObjects = 40;

  // Doubled since each object consists of A and B message
  static constexpr uint8_t kCountObjectMessages = kCountObjects * 2;

  Information info_;
  std::shared_ptr<DiagTask> diag_task_;

  ros::Publisher pub_objects_;
  ros::Publisher pub_objects_pcl_;
  ros::Publisher pub_info_;
  ros::Timer publish_timer_;

  /// CAN id of first object message (Object_00_Data_A)
  FrameId object_base_id_;
  FrameId info_id_;

  /// Allowed age of objects and B message information
  double allowed_age_;

  /// Objects stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<Object>, kCountObjects> objects_;
};

}  // namespace off_highway_radar
