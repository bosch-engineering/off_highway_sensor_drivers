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

#include "off_highway_can/sender.hpp"

namespace off_highway_can
{
Sender::Sender(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  last_message_sent_ = now();
  declare_and_get_parameters();

  can_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);

  diag_task_ =
    std::make_shared<DiagTask>("sender", [this](auto & status) {this->diagnostics(status);});

  diag_composite_ = std::make_shared<DiagCompositeTask>("sender");
  diag_composite_->addTask(diag_task_.get());

  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID(get_name());
  diag_updater_->add(*diag_composite_);

  watchdog_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::duration<double>(1.0 / watchdog_frequency_),
    std::bind(&Sender::callback_watchdog, this)
  );
}

void Sender::callback_watchdog()
{
  if ((now() - last_message_sent_).seconds() > timeout_) {
    RCLCPP_WARN(get_logger(), "Timeout of watchdog for sending node %s", get_name());
    force_diag_update();
    // Reset to not trigger in each run
    last_message_sent_ = now();
  }
}

void Sender::send_can()
{
#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
  using std::chrono::steady_clock;
  using std::chrono::duration;
  auto start = steady_clock::now();
#endif

  for (auto &[id, message] : messages_) {
    can_msgs::msg::Frame frame;

    frame.header.stamp = now();
    frame.header.frame_id = node_frame_id_;

    frame.id = id;
    frame.dlc = frame.data.size();
    frame.is_extended = id > kMaxBaseIdentifier;

    message.encode(frame.data);

    can_pub_->publish(frame);

    last_message_sent_ = now();
    diag_updater_->force_update();
  }

  RCLCPP_DEBUG(
    get_logger(), "Sending of messages in %s took %f s", get_name(),
    duration<double>(steady_clock::now() - start).count());
}

Sender::Messages Sender::get_messages() const
{
  return messages_;
}

void Sender::add_diag_task(const std::shared_ptr<diagnostic_updater::DiagnosticTask> & task)
{
  diag_composite_->addTask(task.get());
}

void Sender::force_diag_update()
{
  diag_updater_->force_update();
}

void Sender::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  bool timeout = (now() - last_message_sent_).seconds() > timeout_;
  stat.add("Timeout", timeout);

  if (timeout) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}

void Sender::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description =
    "Timeout period. Sender goes into timeout error if for the "
    "specified period no input message was received.";
  declare_parameter<double>("timeout", 0.2, param_desc);
  timeout_ = this->get_parameter("timeout").as_double();

  param_desc.description = "Frequency of watchdog to check if a input message was received";
  declare_parameter<double>("watchdog_frequency", 10.0, param_desc);
  watchdog_frequency_ = this->get_parameter("watchdog_frequency").as_double();

  param_desc.description = "TF frame id for all published messages";
  declare_parameter<std::string>("node_frame_id", "base_link", param_desc);
  node_frame_id_ = this->get_parameter("node_frame_id").as_string();
}
}  // namespace off_highway_can
