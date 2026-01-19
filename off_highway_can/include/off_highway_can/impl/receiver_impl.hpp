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

#include "off_highway_can/j1939.hpp"

namespace off_highway_can
{

template<typename Msg>
void Receiver::callback_can(const typename Msg::ConstSharedPtr & frame)
{
#if (RCLCPP_LOG_MIN_SEVERITY <= RCLCPP_LOG_MIN_SEVERITY_DEBUG)
  using std::chrono::steady_clock;
  using std::chrono::duration;
  auto start = steady_clock::now();
#endif
  auto id = frame->id;
  if (use_j1939_) {
    if (!is_j1939_source_address_matching(get_j1939_source_address(frame->id))) {
      return;
    }
    id = get_j1939_pgn(frame->id);
  }

  // Check if received frame ID is from node
  auto msg_it = messages_.find(id);
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

  if (is_timeout_) {
    is_timeout_ = false;
    force_diag_update();
  }

  auto header = frame->header;
  header.frame_id = node_frame_id_;
  process(header, id, msg);

  RCLCPP_DEBUG(
    get_logger(), "Processing of frame in %s took %f s", get_name(),
    duration<double>(steady_clock::now() - start).count());
}

}  // namespace off_highway_can
