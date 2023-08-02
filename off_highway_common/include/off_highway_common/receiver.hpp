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

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/subscription.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "can_msgs/msg/frame.hpp"

#include "off_highway_common/can_message.hpp"

namespace off_highway_common
{
/**
 * \brief Abstract receiver class to decode CAN node frames. Needs to be extended for specific CAN
 * node.
 */
class Receiver : public rclcpp::Node
{
public:
  using FrameId = can_msgs::msg::Frame::_id_type;
  using FrameData = can_msgs::msg::Frame::_data_type;

  using Messages = std::unordered_map<FrameId, Message>;

  using DiagTask =
    diagnostic_updater::GenericFunctionDiagnosticTask<diagnostic_updater::DiagnosticStatusWrapper>;
  using DiagCompositeTask = diagnostic_updater::CompositeDiagnosticTask;

  /**
   * \brief Construct a new Receiver object.
   */
  explicit Receiver(const std::string & node_name = "receiver");

  /**
   * \brief Destroy the Receiver object.
   */
  ~Receiver() = default;

  /**
   * \brief Initialize object (2nd step) by filling message definitions.
   */
  void initialize();

  /**
   * \brief Initialize and start watchdog and subscription to receive frame messages.
   */
  void start();

  /**
   * \brief Stop subscription and watchdog.
   */
  void stop();

  /**
   * \brief Callback on received frames.
   *
   * Checks if frame id is relevant for this instance. If yes, tries to decode message and checks
   * for valid CRC and message counter. If valid, processes message.
   *
   * \param frame ROS CAN frame
   */
  void callback_can(const can_msgs::msg::Frame::SharedPtr frame);

  /**
   * \brief Returns message definitions of configured CAN messages
   */
  Messages get_messages() const;

protected:
  /**
   * \brief Add diagnostic task to composite task for including it in the diagnostic updater.
   *
   * \param task Task to add
   */
  void add_diag_task(const std::shared_ptr<diagnostic_updater::DiagnosticTask> & task);

  /**
   * \brief Enforce diagnose update.
   */
  void force_diag_update();

  // API to fill
  /**
   * \brief Fill message definitions to decode frames of CAN node. Only stored definitions are
   * processed.
   *
   * \return Messages of CAN node to decode and process
   */
  virtual Messages fillMessageDefinitions() = 0;

  /**
   * \brief Process CAN message (e.g. convert into own data type).
   *
   * \param header Header of corresponding ROS message
   * \param id Id of respective CAN frame
   * \param message Decoded message (values) of frame to use for processing
   */
  virtual void process(std_msgs::msg::Header header, const FrameId & id, Message & message) = 0;

  /// Node TF frame id, needed in derived implementations for publishing with correct frame in
  /// header
  std::string node_frame_id_;

private:
  /**
   * \brief Periodically check last received message time stamp to detect sensor timeout.
   */
  void callback_watchdog();

  /**
   * \brief Update diagnostics status by checking timeout of CAN node.
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters.
   */
  void declare_and_get_parameters();

  bool initialized_{false};

  std::shared_ptr<DiagTask> diag_task_;
  std::shared_ptr<DiagCompositeTask> diag_composite_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_message_received_;

  /// CAN message storage, maps CAN id to message data including signals and their en-/decoding
  /// information
  Messages messages_;


  double timeout_;
  double watchdog_frequency_;
};
}  // namespace off_highway_common
