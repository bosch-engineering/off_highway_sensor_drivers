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

#include <type_traits>

namespace off_highway_can
{

template<typename Msg>
void Sender::send_can(FrameId id, Message & message)
{
  Msg frame;

  frame.header.stamp = now();
  frame.header.frame_id = node_frame_id_;

  frame.id = id;
  frame.is_extended = id > kMaxBaseIdentifier;

  message.encode(frame.data);

  if constexpr (std::is_same_v<Msg, can_msgs::msg::Frame>) {
    frame.dlc = frame.data.size();
    can_pub_->publish(frame);
  } else if constexpr (std::is_same_v<Msg, ros2_socketcan_msgs::msg::FdFrame>) {
    frame.len = frame.data.size();
    fd_pub_->publish(frame);
  }

  last_message_sent_ = now();

  if (is_timeout_) {
    is_timeout_ = false;
    force_diag_update();
  }
}

}  // namespace off_highway_can
