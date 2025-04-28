// Copyright 2025 Robert Bosch GmbH and its subsidiaries
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
#include <tuple>

#include "off_highway_premium_radar/node.hpp"

namespace off_highway_premium_radar
{

template<typename ... Converters>
Node<Converters...>::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("driver", options),
  converters_(std::make_tuple(std::make_shared<Converters>()...))
{
  // TODO(rcp1-beg) Replace timer for two-stage initialization due to shared_from_this usage with
  // something more clever
  configure_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(0), [this]() {configure();});
}

template<typename ... Converters>
void Node<Converters...>::configure()
{
  declare_and_get_parameters();

  // Setup driver
  driver_ =
    std::make_shared<Driver>(host_ip_, host_port_, sensor_ip_, sensor_port_, connect_socket_);

  std::apply(
    [this](auto &... converter) {
      (..., std::visit(
        [this](auto & ptr) {
          ptr->configure(shared_from_this(), driver_);
        }, converter));
    }, converters_);

  std::apply(
    [this](auto &... converter) {
      (..., std::visit(
        [this](auto & ptr) {
          driver_->register_receiver(ptr);
        }, converter));
    }, converters_);

  driver_->start_receiving();

  configure_timer_.reset();
}

template<typename ... Converters>
void Node<Converters...>::declare_and_get_parameters()
{
  host_ip_ = declare_parameter("host_ip", host_ip_);
  host_port_ = declare_parameter("host_port", host_port_);
  sensor_ip_ = declare_parameter("sensor_ip", sensor_ip_);
  sensor_port_ = declare_parameter("sensor_port", sensor_port_);
  connect_socket_ = declare_parameter("connect_socket", connect_socket_);
}

}  // namespace off_highway_premium_radar
