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

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "io_context/io_context.hpp"

#include "off_highway_premium_radar/driver.hpp"
#include "off_highway_premium_radar/interface/converter.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Node to as central manager of all classes of the driver.
 *
 * Creates driver, configures converters with node and sender interface and registers receivers in
 * driver.
 *
 */
class Node : public rclcpp::Node
{
public:
  using Converters = std::vector<Converter::SharedPtr>;

  /**
   * \brief Construct a new Node object
   *
   * \param node_name Name of node
   */
  explicit Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * \brief Create driver, configure converters, register receivers and start receiving
   *
   * \param converters List of converters to connect to driver
   */
  void configure(Converters converters);

private:
  /**
   * \brief Declare and get ROS parameters
   */
  void declare_and_get_parameters();

  //! Driver to send data to sensor and register receivers to process incoming data
  std::shared_ptr<Driver> driver_;

  // Parameters
  //! Host IP, empty to use all interfaces
  std::string host_ip_;
  //! Host port
  uint16_t host_port_{0x76C0};
  //! Sensor IP
  std::string sensor_ip_{"192.168.40.50"};
  //! Sensor port
  uint16_t sensor_port_{0x76C6};
  //! Connect socket to sensor IP for running multiple drivers in parallel using the same port (but
  //! broadcast messages are dropped)
  bool connect_socket_{false};
};

}  // namespace off_highway_premium_radar
