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

#include "off_highway_common/receiver.hpp"

#include "off_highway_common/helper.hpp"

namespace off_highway_common
{
Receiver::Receiver()
{
  double timeout = get_param_or_throw<double>(private_nh_, "timeout");
  double watchdog_frequency = get_param_or_throw<double>(private_nh_, "watchdog_frequency");
  node_frame_id_ = get_param_or_throw<std::string>(private_nh_, "node_frame_id");

  diag_task_ =
    std::make_shared<DiagTask>("receiver", [this](auto & status) {diagnostics(status);});

  diag_composite_ = std::make_shared<DiagCompositeTask>("receiver");
  diag_composite_->addTask(diag_task_.get());

  diag_updater_.setHardwareID(ros::this_node::getName());
  diag_updater_.add(*diag_composite_);

  timeout_ = ros::Duration(timeout);

  watchdog_timer_ = private_nh_.createTimer(
    ros::Rate(watchdog_frequency), &Receiver::callback_watchdog, this, false, false);
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
  // TODO(rcp1-beg) Replace two-step initialization
  initialize();
  can_sub_ =
    std::make_unique<ros::Subscriber>(
    nh_.subscribe("received_messages", 100, &Receiver::callback_can, this));
  watchdog_timer_.start();
}

void Receiver::stop()
{
  can_sub_.reset();
  watchdog_timer_.stop();
}

void Receiver::callback_watchdog(const ros::TimerEvent & /* event */)
{
  if (ros::Time::now() - last_message_received_ > timeout_) {
    ROS_WARN_STREAM("Timeout of watchdog for receiving node " << ros::this_node::getName());
    force_diag_update();
    // Reset to not trigger in each run
    last_message_received_ = ros::Time::now();
  }
}

void Receiver::callback_can(const can_msgs::Frame & frame)
{
#if (ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG)
  // Only used for debug timing output
  using ros::SteadyTime;
  using ros::this_node::getName;
  auto start = SteadyTime::now();
#endif

  // Check if received frame ID is from node
  auto msg_it = messages_.find(frame.id);
  if (msg_it == messages_.end()) {
    ROS_DEBUG_STREAM(
      "Filtering of frame in " << getName() << " took " << SteadyTime::now() - start << " s");
    return;
  }

  Message & msg = msg_it->second;
  if (!msg.decode(frame.data)) {
    return;
  }

  last_message_received_ = ros::Time::now();
  diag_updater_.update();

  auto header = frame.header;
  header.frame_id = node_frame_id_;
  process(header, frame.id, msg);

  ROS_DEBUG_STREAM(
    "Processing of frame in " << getName() << " took " << SteadyTime::now() - start << " s");
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
  diag_updater_.force_update();
}

void Receiver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::DiagnosticStatus;

  bool timeout = ros::Time::now() - last_message_received_ > timeout_;
  stat.add("Timeout", timeout);

  if (timeout) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}

}  // namespace off_highway_common
