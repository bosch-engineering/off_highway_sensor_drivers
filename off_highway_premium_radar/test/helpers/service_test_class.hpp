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

#include <algorithm>
#include <future>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "../src/converters/ros_message_conversions.hpp"
#include "../src/helper.hpp"
#include "gtest/gtest.h"
#include "off_highway_premium_radar/converters/default_converter.hpp"
#include "off_highway_premium_radar/node.hpp"
#include "rclcpp/executor.hpp"

using std::chrono::seconds;
using NodeWithDefaultConverter =
  off_highway_premium_radar::Node<off_highway_premium_radar::DefaultConverter>;

using off_highway_premium_radar::to_pdu;

template<typename ServiceType, typename PduType>
class ServiceTestClass : public ::testing::Test
{
public:
  explicit ServiceTestClass(
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

  ~ServiceTestClass() override
  {
    udp_socket_.close();
  }

protected:
  void SetUp(
    const std::string & client_node_name,
    const std::string & service_name)
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("host_ip", loopback_ip_),
      rclcpp::Parameter("sensor_ip", loopback_ip_),
      rclcpp::Parameter("host_port", host_port_),
      rclcpp::Parameter("sensor_port", sensor_port_)
    };

    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);

    node_ = std::make_shared<NodeWithDefaultConverter>(node_options);

    service_client_node_ = std::make_shared<rclcpp::Node>(client_node_name);
    service_client_ =
      service_client_node_->create_client<ServiceType>(service_name);

    executor_.add_node(node_);
    executor_.add_node(service_client_node_);
    // Spin node to configure driver
    executor_.spin_some();
  }

  template<typename RequestData>
  void send_service_request(const RequestData & request_data)
  {
    auto request = std::make_shared<typename ServiceType::Request>();

    // Set request data based on service type
    if constexpr (
      std::is_same_v<ServiceType, off_highway_premium_radar_msgs::srv::MeasurementProgram>)
    {
      request->id = request_data;
    }

    // Wait for service to be available
    ASSERT_TRUE(service_client_->wait_for_service(spin_timeout_));

    auto future = service_client_->async_send_request(request);

    // Spin until we get a response or timeout
    auto status = executor_.spin_until_future_complete(future, spin_timeout_);
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

    auto response = future.get();
    ASSERT_TRUE(response->success);
  }

  // PDU data reception
  PduType receive_pdu_data()
  {
    std::vector<uint8_t> buffer;
    buffer.resize(PduType::kPduSize);

    udp_socket_.receive(buffer);

    return to_pdu<PduType>(buffer);
  }

  const int sensor_port_;
  const int host_port_;
  const std::string loopback_ip_;
  const std::chrono::seconds spin_timeout_;

  IoContext ctx_;
  off_highway_premium_radar::UdpSocket udp_socket_;

  std::shared_ptr<NodeWithDefaultConverter> node_;
  rclcpp::Node::SharedPtr service_client_node_;
  typename rclcpp::Client<ServiceType>::SharedPtr service_client_;

  rclcpp::executors::SingleThreadedExecutor executor_;

  off_highway_premium_radar::MeasurementProgram received_data_;
};
