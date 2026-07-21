// Copyright 2021 LeoDrive.
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

// Developed by LeoDrive, 2021

// Modifications:
// Copyright 2026 Robert Bosch GmbH and its subsidiaries
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

#include "rclcpp/logging.hpp"

#include "off_highway_premium_radar/io_context.hpp"

namespace off_highway_premium_radar
{

IoContext::IoContext()
: IoContext(std::thread::hardware_concurrency()) {}

IoContext::IoContext(size_t threads_count)
: m_ios(new asio::io_context()),
  m_work(new asio::executor_work_guard<asio::io_context::executor_type>(ios().get_executor()))
{
  m_ios_thread_workers.reserve(threads_count);
  for (size_t i = 0; i < threads_count; ++i) {
    m_ios_thread_workers.emplace_back(
      [this]() {
        ios().run();
      });
  }

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("IoContext::IoContext"), "Thread(s) Created: " <<
      serviceThreadCount());
}

IoContext::~IoContext()
{
  waitForExit();
}

asio::io_context & IoContext::ios() const
{
  return *m_ios;
}

bool IoContext::isServiceStopped()
{
  return ios().stopped();
}

uint32_t IoContext::serviceThreadCount()
{
  return m_ios_thread_workers.size();
}

void IoContext::waitForExit()
{
  if (!ios().stopped()) {
    asio::post(ios(), [&]() {m_work.reset();});
  }

  ios().stop();
  for (auto & thread : m_ios_thread_workers) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

}  // namespace off_highway_premium_radar
