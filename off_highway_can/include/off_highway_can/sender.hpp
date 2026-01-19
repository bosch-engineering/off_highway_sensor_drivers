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
#include <unordered_map>
#include <string>

#include "rclcpp/node.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"

#include "can_message.hpp"

namespace off_highway_can
{
/**
 * \brief Abstract sender class to encode CAN (FD) frames. Needs to be extended for specific CAN
 * messages.
 */
class Sender : public rclcpp::Node
{
public:
  using FrameId = uint32_t;
  using Messages = std::unordered_map<FrameId, Message>;

  using DiagTask =
    diagnostic_updater::GenericFunctionDiagnosticTask<diagnostic_updater::DiagnosticStatusWrapper>;
  using DiagCompositeTask = diagnostic_updater::CompositeDiagnosticTask;

  /**
   * \brief Construct a new Sender object.
   *
   * \param node_name Name of the node
   * \param options Node options
   * \param use_fd Enable CAN FD frames
   */
  explicit Sender(
    const std::string & node_name = "sender",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    bool use_fd = false);

  /**
   * \brief Destroy the Sender object.
   */
  virtual ~Sender() = default;

  /**
   * \brief Encode messages into CAN frames and send them.
   *
   */
  void send_can();

  /**
   * \brief Returns message definitions of configured CAN messages
   */
  Messages get_messages() const;

protected:
  /**
   * \brief Encode message into CAN frame and send it.
   * \param id Frame ID
   * \param message Message to send
   * \tparam Msg Type of ROS message to send
   */
  template<typename Msg>
  void send_can(FrameId id, Message & message);

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

  Messages messages_;

  /// Use CAN FD frames, defaults to false
  const bool use_fd_{false};

private:
  /**
   * \brief Periodically check last sent message time stamp to detect sensor timeout.
   */
  void callback_watchdog();

  /**
   * \brief Update diagnostics status by checking timeout of received messages.
   *
   * \param stat Status wrapper of diagnostics.
   */
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  /**
   * \brief Declare and get node parameters.
   */
  void declare_and_get_parameters();

  static constexpr uint32_t kMaxBaseIdentifier{1 << 11};

  /// Store the timeout status
  bool is_timeout_{false};

  std::shared_ptr<DiagTask> diag_task_;
  std::shared_ptr<DiagCompositeTask> diag_composite_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr fd_pub_;
  std::function<void(Messages::value_type &)> send_msg_;
  rclcpp::Time last_message_sent_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  /// Node TF frame id
  std::string node_frame_id_;
  double timeout_;
  double watchdog_frequency_;
};
}  // namespace off_highway_can

#include "off_highway_can/impl/sender_impl.hpp"
