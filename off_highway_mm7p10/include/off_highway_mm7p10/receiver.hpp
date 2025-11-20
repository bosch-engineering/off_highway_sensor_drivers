// Copyright 2025 Robert Bosch GmbH and its subsidiaries
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

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "can_msgs/msg/frame.hpp"

#include "off_highway_can/receiver.hpp"
#include "off_highway_mm7p10_msgs/msg/information.hpp"

namespace off_highway_mm7p10
{

// Conversion factors
static constexpr double G_TO_MPS2_FACTOR = 9.80665f;
static constexpr double DEGREE_TO_RAD_FACTOR = M_PI / 180;

// Status bit definitions for diagnostics
enum class StatusBits : uint8_t
{
  IMU_NOT_AVAILABLE = 0x01,
  SIGNAL_FAILURE = 0x02,
  INITIALIZATION_IN_PROGRESS = 0x04
};

// Status mask for all status bits
static constexpr uint8_t STATUS_MASK =
  static_cast<uint8_t>(StatusBits::IMU_NOT_AVAILABLE) |
  static_cast<uint8_t>(StatusBits::SIGNAL_FAILURE) |
  static_cast<uint8_t>(StatusBits::INITIALIZATION_IN_PROGRESS);

// Temperature status values
static constexpr uint8_t TEMP_INVALID_LOW = 0xFF;   // CRC error or temp < -50°C
static constexpr uint8_t TEMP_INVALID_HIGH = 0xC9;  // temp > 150°C

/**
 * \brief MM7P10 receiver class to decode CAN frames into sensor_msgs::Imu to publish.
 *
 * Imu frame is published as simple sensor_msgs::Imu message.
 */
class Receiver : public off_highway_can::Receiver
{
public:
  using Message = off_highway_can::Message;
  using Imu = sensor_msgs::msg::Imu;
  using Information = off_highway_mm7p10_msgs::msg::Information;
  /**
   * \brief Construct a new Receiver object.
   */
  explicit Receiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
   * \brief Filter IMU data.
   *
   * \param imu IMU data to filter
   * \return True if imu data is invalid and should be filtered, false otherwise
   */
  bool filter(const sensor_msgs::msg::Imu & imu);

  /**
   * \brief Manage and publish IMU data.
   */
  void manage_and_publish_imu();

  /**
   * \brief Publish IMU and information data.
   */
  void publish_imu_data();

  /**
   * \brief Publish imu data.
   */
  void publish_imu();

  /**
   * \brief Publish information data.
   */
  void publish_info();

  /**
   * \brief Update diagnostics status by checking last sensor information.
   *
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters
   */
  void declare_and_get_parameters();

  Information info_;
  std::shared_ptr<DiagTask> diag_task_;

  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<Information>::SharedPtr pub_info_;

  /// CAN frame ids
  uint32_t MM7_10_Z_AY_ID = 0x174;
  uint32_t MM7_10_X_AX_ID = 0x178;
  uint32_t MM7_10_Y_AZ_ID = 0x17C;

  /// Allowed age of messages and publish frequency
  double allowed_age_;

  /// Frame counter to detect lost frames
  uint8_t frame_counter_ = 0;

  /// Message counter for frame synchronization validation
  uint8_t current_can_frame_msg_counter_ = 0;

  /// Stored info message data
  sensor_msgs::msg::Imu imu_;
};
}  // namespace off_highway_mm7p10
