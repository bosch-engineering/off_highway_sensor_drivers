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

#include "off_highway_common/sender.hpp"

#include "off_highway_common/helper.hpp"

namespace off_highway_common
{

Sender::Sender()
{
  double timeout = get_param_or_throw<double>(private_nh_, "timeout");
  double watchdog_frequency = get_param_or_throw<double>(private_nh_, "watchdog_frequency");
  node_frame_id_ = get_param_or_throw<std::string>(private_nh_, "node_frame_id");

  can_pub_ = nh_.advertise<can_msgs::Frame>("sent_messages", 10);

  diag_task_ =
    std::make_shared<DiagTask>("sender", [this](auto & status) {diagnostics(status);});

  diag_composite_ = std::make_shared<DiagCompositeTask>("sender");
  diag_composite_->addTask(diag_task_.get());

  diag_updater_.setHardwareID(ros::this_node::getName());
  diag_updater_.add(*diag_composite_);

  timeout_ = ros::Duration(timeout);

  watchdog_timer_ = private_nh_.createTimer(
    ros::Rate(watchdog_frequency), &Sender::callback_watchdog, this);
}

void Sender::callback_watchdog(const ros::TimerEvent & /* event */)
{
  if (ros::Time::now() - last_message_sent_ > timeout_) {
    ROS_WARN_STREAM("Timeout of watchdog for sending node " << ros::this_node::getName());
    force_diag_update();
    // Reset to not trigger in each run
    last_message_sent_ = ros::Time::now();
  }
}

void Sender::send_can()
{
#if (ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_DEBUG)
  // Only used for debug timing output
  using ros::SteadyTime;
  using ros::this_node::getName;
  auto start = SteadyTime::now();
#endif

  for (auto &[id, message] : messages_) {
    can_msgs::Frame frame;

    frame.header.stamp = ros::Time::now();
    frame.header.frame_id = node_frame_id_;

    frame.id = id;
    frame.dlc = can_msgs::Frame::_data_type::size();
    frame.is_extended = id > kMaxBaseIdentifier;

    message.encode(frame.data);

    can_pub_.publish(frame);

    last_message_sent_ = ros::Time::now();
    diag_updater_.update();
  }

  ROS_DEBUG_STREAM(
    "Sending of messages in " << getName() << " took " << SteadyTime::now() - start << " s");
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
  diag_updater_.force_update();
}

void Sender::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::DiagnosticStatus;

  bool timeout = ros::Time::now() - last_message_sent_ > timeout_;
  stat.add("Timeout", timeout);

  if (timeout) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}
}  // namespace off_highway_common
