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
#include <optional>
#include <string>

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "can_msgs/msg/frame.hpp"

#include "off_highway_can/receiver.hpp"
#include "off_highway_radar_msgs/msg/object.hpp"
#include "off_highway_radar_msgs/msg/object_a.hpp"
#include "off_highway_radar_msgs/msg/object_b.hpp"
#include "off_highway_radar_msgs/msg/objects.hpp"
#include "off_highway_radar_msgs/msg/information.hpp"

namespace off_highway_radar
{
/**
 * \brief Radar receiver class to decode CAN frames into object list to publish.
 *
 * Object list is published as simple list or as point cloud. Sensor information frame is used as
 * diagnostic input and also published directly.
 */
class Receiver : public off_highway_can::Receiver
{
public:
  using Message = off_highway_can::Message;
  using Object = off_highway_radar_msgs::msg::Object;
  using ObjectA = off_highway_radar_msgs::msg::ObjectA;
  using ObjectB = off_highway_radar_msgs::msg::ObjectB;
  using Objects = off_highway_radar_msgs::msg::Objects;
  using Information = off_highway_radar_msgs::msg::Information;

  /**
   * \brief Construct a new Receiver object.
   */
  explicit Receiver(const std::string & node_name = "receiver");

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
  void process(std_msgs::msg::Header header, const FrameId & id, Message & message) override;

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
  void manage_and_publish();

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
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters
   */
  void declare_and_get_parameters();

  static constexpr uint8_t kCountObjects = 40;
  // Doubled since each object consists of A and B message
  static constexpr uint8_t kCountObjectMessages = kCountObjects * 2;

  Information info_;
  std::shared_ptr<DiagTask> diag_task_;

  rclcpp::Publisher<Objects>::SharedPtr pub_objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_objects_pcl_;
  rclcpp::Publisher<Information>::SharedPtr pub_info_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// CAN id of first object message (Object_00_Data_A)
  uint32_t object_base_id_;
  uint32_t info_id_;
  /// Allowed age of objects and B message information
  double allowed_age_;
  double publish_frequency_;

  /// Objects stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<Object>, kCountObjects> objects_;
};
}  // namespace off_highway_radar
