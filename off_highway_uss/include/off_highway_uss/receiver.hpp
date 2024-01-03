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

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/parameter.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "can_msgs/msg/frame.hpp"

#include "off_highway_can/receiver.hpp"
#include "off_highway_can/can_message.hpp"
#include "off_highway_uss_msgs/msg/objects.hpp"
#include "off_highway_uss_msgs/msg/direct_echos.hpp"
#include "off_highway_uss_msgs/msg/max_detection_range.hpp"
#include "off_highway_uss_msgs/msg/information.hpp"

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
class Receiver : public off_highway_can::Receiver
{
public:
  using Message = off_highway_can::Message;
  using Object = off_highway_uss_msgs::msg::Object;
  using Objects = off_highway_uss_msgs::msg::Objects;
  using Echo = off_highway_uss_msgs::msg::Echo;
  using DirectEcho = off_highway_uss_msgs::msg::DirectEcho;
  using DirectEchos = off_highway_uss_msgs::msg::DirectEchos;
  using MaxDetectionRange = off_highway_uss_msgs::msg::MaxDetectionRange;
  using Information = off_highway_uss_msgs::msg::Information;

  /**
   * \brief Construct a new Receiver object.
   */
  explicit Receiver(
    const std::string & node_name = "receiver",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  void process(std_msgs::msg::Header header, const FrameId & id, Message & message) override;

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
  void manage_and_publish_objects();

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
  void manage_and_publish_direct_echos();

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
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters
   */
  void declare_and_get_parameters();

  static constexpr uint8_t kDirectEchoBaseIdOffset = 0x0;
  static constexpr uint8_t kInfoIdOffset = 0x0C;
  static constexpr uint8_t kMaxDetectionRangeIdOffset = 0xD;
  static constexpr uint8_t kObjectBaseIdOffset = 0x10;
  static constexpr uint8_t kCountObjects = 20;
  static constexpr uint8_t kCountObjectFrames = kCountObjects / 2;
  static constexpr uint8_t kCountDirectEchos = 12;
  static constexpr double kCentimeterToMeter = 0.01;
  static constexpr std::array<double, 8> kExistProbabilitySteps = {0.0, 17.0, 23.0, 30.0,
    40.0, 60.0, 80.0, 100.0};

  Information info_;
  std::shared_ptr<DiagTask> diag_task_;

  rclcpp::Publisher<Objects>::SharedPtr pub_objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_objects_pcl_;
  rclcpp::Publisher<DirectEchos>::SharedPtr pub_direct_echos_;
  rclcpp::Publisher<MaxDetectionRange>::SharedPtr pub_max_detection_range_;
  rclcpp::Publisher<Information>::SharedPtr pub_info_;
  rclcpp::TimerBase::SharedPtr publish_objects_timer_;
  rclcpp::TimerBase::SharedPtr publish_direct_echos_timer_;

  /// CAN frame id offset for functional frames
  uint32_t can_id_offset_;
  /// CAN id of first object message (USS_MAP_OBJ_01)
  uint32_t object_base_id_;
  /// CAN id of first direct echo message (USS_DEMsg_Sens_01)
  uint32_t direct_echo_base_id_;
  uint32_t info_id_;
  uint32_t max_detection_range_id_;

  /// Allowed age of objects and echos
  double allowed_age_;
  /// Sample distance along line of line objects
  double line_sample_distance_;
  double publish_frequency_;

  /// Objects stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<Object>, kCountObjects> objects_;
  /// Echos stored as optionals in fixed-size array to encode validity while ensuring order
  std::array<std::optional<DirectEcho>, kCountDirectEchos> direct_echos_;
};

template<typename T>
void Receiver::manage(T & l)
{
  for (auto & o : l) {
    if (o && (filter(*o) || abs((now() - o->header.stamp).seconds()) > allowed_age_)) {
      o = {};
    }
  }
}

}  // namespace off_highway_uss
