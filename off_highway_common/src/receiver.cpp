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

#include "off_highway_common/receiver.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace off_highway_common
{
Receiver::Receiver(const std::string & node_name)
: rclcpp::Node(node_name)
{
  this->last_message_received_ = now();
  this->declare_and_get_parameters();

  diag_task_ =
    std::make_shared<DiagTask>("receiver", [this](auto & status) {diagnostics(status);});

  diag_composite_ = std::make_shared<DiagCompositeTask>("receiver");
  diag_composite_->addTask(diag_task_.get());

  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID(get_name());
  diag_updater_->add(*diag_composite_);
}

void Receiver::initialize()
{
  if (!initialized_) {
    messages_ = fillMessageDefinitions();
    initialized_ = true;
  }
}

void Receiver::start()
{
  initialize();

  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    "received_messages",
    100,
    std::bind(&Receiver::callback_can, this, std::placeholders::_1)
  );

  watchdog_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::duration<double>(1.0 / watchdog_frequency_),
    std::bind(&Receiver::callback_watchdog, this)
  );
}

void Receiver::stop()
{
  can_sub_.reset();
  watchdog_timer_->cancel();
}

void Receiver::callback_watchdog()
{
  if ((now() - last_message_received_).seconds() > timeout_) {
    RCLCPP_WARN(get_logger(), "Timeout of watchdog for receiving node %s", get_name());
    force_diag_update();
    last_message_received_ = now();
  }
}

void Receiver::callback_can(const can_msgs::msg::Frame::SharedPtr frame)
{
#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
  using std::chrono::steady_clock;
  using std::chrono::duration;
  auto start = steady_clock::now();
#endif

  // Check if received frame ID is from node
  auto msg_it = messages_.find(frame->id);
  if (msg_it == messages_.end()) {
    RCLCPP_DEBUG(
      this->get_logger(), "Filtering of frame in %s took %f s", get_name(),
      duration<double>(steady_clock::now() - start).count());
    return;
  }

  Message & msg = msg_it->second;
  if (!msg.decode(frame->data)) {
    return;
  }

  last_message_received_ = now();
  diag_updater_->force_update();

  auto header = frame->header;
  header.frame_id = node_frame_id_;
  process(header, frame->id, msg);

  RCLCPP_DEBUG(
    get_logger(), "Processing of frame in %s took %f s", get_name(),
    duration<double>(steady_clock::now() - start).count());
}

Receiver::Messages Receiver::get_messages() const
{
  return messages_;
}

void Receiver::add_diag_task(const std::shared_ptr<diagnostic_updater::DiagnosticTask> & task)
{
  diag_composite_->addTask(task.get());
}

void Receiver::force_diag_update()
{
  diag_updater_->force_update();
}

void Receiver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  bool timeout = (now() - last_message_received_).seconds() > timeout_;

  stat.add("Timeout", timeout);

  if (timeout) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}

void Receiver::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description =
    "Timeout period. Receiver goes into timeout error if "
    "for the specified period no sensor message was received.";
  declare_parameter<double>("timeout", 0.2, param_desc);
  timeout_ = get_parameter("timeout").as_double();

  param_desc.description = "Frequency of watchdog to check if a sensor message was received";
  declare_parameter<double>("watchdog_frequency", 10.0, param_desc);
  watchdog_frequency_ = get_parameter("watchdog_frequency").as_double();

  param_desc.description = "TF frame id for all published messages of the receiver";
  declare_parameter<std::string>("node_frame_id", "base_link", param_desc);
  node_frame_id_ = get_parameter("node_frame_id").as_string();
}
}  // namespace off_highway_common
