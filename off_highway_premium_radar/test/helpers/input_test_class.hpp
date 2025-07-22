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

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "off_highway_premium_radar/node.hpp"
#include "off_highway_premium_radar/converters/default_converter.hpp"
#include "rclcpp/executor.hpp"
#include "../src/helper.hpp"

using std::chrono::seconds;
using NodeWithDefaultConverter =
  off_highway_premium_radar::Node<off_highway_premium_radar::DefaultConverter>;

using off_highway_premium_radar::to_pdu;

template<typename MsgType, typename PduType>
class InputTestClass : public ::testing::Test
{
public:
  explicit InputTestClass(
    int sensor_port,
    int host_port,
    const std::string & loopback_ip,
    std::chrono::seconds spin_timeout)
  : sensor_port_(sensor_port),
    host_port_(host_port),
    loopback_ip_(loopback_ip),
    spin_timeout_(spin_timeout),
    ctx_(1),
    udp_socket_(ctx_, loopback_ip, host_port, loopback_ip, sensor_port)
  {
    udp_socket_.open();
    udp_socket_.bind();
  }

  ~InputTestClass() override
  {
    udp_socket_.close();
  }

protected:
  void SetUp(
    const std::string & node_name,
    const std::string & topic_name)
  {
    std::string pub_name = node_name + "_subscriber";

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("host_ip", loopback_ip_),
      rclcpp::Parameter("sensor_ip", loopback_ip_),
      rclcpp::Parameter("host_port", host_port_),
      rclcpp::Parameter("sensor_port", sensor_port_)
    };

    if constexpr (std::is_same_v<MsgType, off_highway_premium_radar_msgs::msg::EgoVehicleInput>) {
      params.emplace_back("send_ego_vehicle_data", true);
    }

    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);

    node_ = std::make_shared<NodeWithDefaultConverter>(node_options);

    sensor_pub_node_ = std::make_shared<rclcpp::Node>(pub_name);
    sensor_publisher_ =
      sensor_pub_node_->create_publisher<MsgType>(
      topic_name, rclcpp::QoS(1).transient_local());

    executor_.add_node(node_);
    executor_.add_node(sensor_pub_node_);
    // Spin node to configure driver
    executor_.spin_some();
  }

  virtual void publish_pdu_data(const MsgType & msg_data) = 0;

  const int sensor_port_;
  const int host_port_;
  const std::string loopback_ip_;
  const std::chrono::seconds spin_timeout_;

  IoContext ctx_;
  off_highway_premium_radar::UdpSocket udp_socket_;

  std::shared_ptr<NodeWithDefaultConverter> node_;
  rclcpp::Node::SharedPtr sensor_pub_node_;
  rclcpp::Publisher<MsgType>::SharedPtr sensor_publisher_;

  rclcpp::executors::SingleThreadedExecutor executor_;
};
