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
#include "off_highway_uss_msgs/Object.h"
#include "off_highway_uss_msgs/Objects.h"
#include "off_highway_uss_msgs/DirectEcho.h"
#include "off_highway_uss_msgs/DirectEchos.h"
#include "off_highway_uss_msgs/MaxDetectionRange.h"
#include "off_highway_uss_msgs/Information.h"

namespace off_highway_uss
{

/**
 * \brief Uss receiver class to decode CAN frames into object list and echo list to publish.
 *
 * Object list is published as simple list or as point cloud. Echo list is only published as simple
 * list. Sensor information frame is used as diagnostic input and also published directly.
 *
 * Line objects in point cloud are sampled along the line.
 */
class Receiver : public off_highway_common::Receiver
{
public:
  using Message = off_highway_common::Message;
  using Object = off_highway_uss_msgs::Object;
  using Objects = off_highway_uss_msgs::Objects;
  using Echo = off_highway_uss_msgs::Echo;
  using DirectEcho = off_highway_uss_msgs::DirectEcho;
  using DirectEchos = off_highway_uss_msgs::DirectEchos;
  using MaxDetectionRange = off_highway_uss_msgs::MaxDetectionRange;
  using Information = off_highway_uss_msgs::Information;

  /**
   * \brief Construct a new Receiver object.
   */
  Receiver();

  /**
   * \brief Destroy the Receiver object.
   */
  ~Receiver() = default;

protected:
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
   * \brief Filter list and remove too old elements from list.
   */
  template<typename T>
  void manage(T & l);

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
  void manage_and_publish_objects(const ros::TimerEvent &);

  /**
   * \brief Publish objects as list.
   */
  void publish_objects();

  /**
   * \brief Publish objects as point cloud.
   */
  void publish_pcl();

  /**
   * \brief Check if echo is invalid.
   *
   * \param direct_echo Echo to check
   * \return True if echo is invalid and should be filtered, false otherwise
   */
  bool filter(const DirectEcho & direct_echo);

  /**
   * \brief Manage echo list and publish it.
   */
  void manage_and_publish_direct_echos(const ros::TimerEvent &);

  /**
   * \brief Publish echos as list.
   */
  void publish_direct_echos();

  /**
   * \brief Update diagnostics status by checking last sensor information.
   *
   * Uses sensor blindness, sensor faulted and failure status of sensor information message to
   * indicate sensor error.
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  static constexpr uint8_t kCountObjects = 20;
  static constexpr uint8_t kCountObjectFrames = kCountObjects / 2;
  static constexpr uint8_t kCountDirectEchos = 12;
  static constexpr double kCentimeterToMeter = 0.01;
  static constexpr std::array<double, 8> kExistProbabilitySteps = {0.0, 17.0, 23.0, 30.0,
    40.0, 60.0, 80.0, 100.0};

  Information info_;
  std::shared_ptr<DiagTask> diag_task_;

  ros::Publisher pub_objects_;
  ros::Publisher pub_objects_pcl_;
  ros::Publisher pub_direct_echos_;
  ros::Publisher pub_max_detection_range_;
  ros::Publisher pub_info_;
  ros::Timer publish_objects_timer_;
  ros::Timer publish_direct_echos_timer_;

  /// CAN id of first object message (USS_MAP_OBJ_01)
  FrameId object_base_id_;
  /// CAN id of first direct echo message (USS_DEMsg_Sens_01)
  FrameId direct_echo_base_id_;
  FrameId info_id_;
  FrameId max_detection_range_id_;

  /// Allowed age of objects and echos
  double allowed_age_;
  /// Sample distance along line of line objects
  double line_sample_distance_;

  /// Objects stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<Object>, kCountObjects> objects_;

  /// Echos stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<DirectEcho>, kCountDirectEchos> direct_echos_;
};

template<typename T>
void Receiver::manage(T & l)
{
  for (auto & o : l) {
    auto now = ros::Time::now();
    if (o && (filter(*o) || abs((now - o->header.stamp).toSec()) > allowed_age_)) {
      o = {};
    }
  }
}

}  // namespace off_highway_uss
