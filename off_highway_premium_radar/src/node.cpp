// Copyright 2023 Robert Bosch GmbH and its subsidiaries
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

#include "off_highway_premium_radar/node.hpp"

#include <functional>

namespace off_highway_premium_radar
{

Node::Node(const std::string & node_name)
: rclcpp::Node(node_name)
{
}

void Node::configure(Converters converters)
{
  declare_and_get_parameters();

  // Setup driver
  driver_ =
    std::make_shared<Driver>(host_ip_, host_port_, sensor_ip_, sensor_port_, connect_socket_);

  for (auto & converter : converters) {
    converter->configure(shared_from_this(), driver_);
  }

  for (auto & converter : converters) {
    driver_->register_receiver(converter);
  }

  driver_->start_receiving();
}

void Node::declare_and_get_parameters()
{
  host_ip_ = declare_parameter("host_ip", host_ip_);
  host_port_ = declare_parameter("host_port", host_port_);
  sensor_ip_ = declare_parameter("sensor_ip", sensor_ip_);
  sensor_port_ = declare_parameter("sensor_port", sensor_port_);
  connect_socket_ = declare_parameter("connect_socket", connect_socket_);
}

}  // namespace off_highway_premium_radar
