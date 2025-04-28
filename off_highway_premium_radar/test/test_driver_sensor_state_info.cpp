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

#include <future>

#include "gtest/gtest.h"
#include "off_highway_premium_radar/node.hpp"
#include "off_highway_premium_radar/converters/default_converter.hpp"
#include "rclcpp/executor.hpp"
#include "../src/helper.hpp"

using namespace std::chrono_literals;
using NodeWithDefaultConverter =
  off_highway_premium_radar::Node<off_highway_premium_radar::DefaultConverter>;

using off_highway_premium_radar::to_pdu;

class TestRadarDriver : public ::testing::Test
{
public:
  TestRadarDriver()
  : ::testing::Test(),
    ctx_(1),
    udp_socket_(ctx_, loopback_ip_, host_port_, loopback_ip_, sensor_port_)
  {
    udp_socket_.open();
    udp_socket_.bind();
  }

protected:
  void SetUp() override
  {
    using off_highway_premium_radar_msgs::msg::SensorStateInformation;

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("host_ip", loopback_ip_),
      rclcpp::Parameter("sensor_ip", loopback_ip_),
      rclcpp::Parameter("host_port", host_port_),
      rclcpp::Parameter("sensor_port", sensor_port_)
    };
    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);

    node_ = std::make_shared<NodeWithDefaultConverter>(node_options);

    future_ = promise_.get_future();
    sensor_state_info_node_ = std::make_shared<rclcpp::Node>("sensor_state_info_subscriber");
    sensor_state_info_subscription_ =
      sensor_state_info_node_->create_subscription<SensorStateInformation>(
      "/driver/sensor_state_information", 10,
      [&](const SensorStateInformation msg) {
        received_sensor_state_info_ = msg;
        promise_.set_value(true);
      });

    executor_.add_node(node_);
    executor_.add_node(sensor_state_info_node_);
    // Spin node to configure driver
    executor_.spin_some();
  }
  void send_sensor_state_info(off_highway_premium_radar::SensorStateInformation info);
  off_highway_premium_radar_msgs::msg::SensorStateInformation get_sensor_state_info();
  void verify_sensor_state_info(
    off_highway_premium_radar::SensorStateInformation ref_sensor_state_info,
    off_highway_premium_radar_msgs::msg::SensorStateInformation sub_sensor_state_info);

private:
  const int sensor_port_ = 0x76C6;
  const int host_port_ = 0x76C0;
  const std::string loopback_ip_ = "127.0.0.1";
  const std::chrono::seconds spin_timeout_ = 1s;

  IoContext ctx_;
  off_highway_premium_radar::UdpSocket udp_socket_;

  std::shared_ptr<NodeWithDefaultConverter> node_;

  rclcpp::Node::SharedPtr sensor_state_info_node_;
  rclcpp::Subscription<off_highway_premium_radar_msgs::msg::SensorStateInformation>::
  SharedPtr
    sensor_state_info_subscription_;
  off_highway_premium_radar_msgs::msg::SensorStateInformation received_sensor_state_info_;

  std::promise<bool> promise_;
  std::shared_future<bool> future_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void TestRadarDriver::send_sensor_state_info(
  off_highway_premium_radar::SensorStateInformation info)
{
  auto data = info.serialize();
  udp_socket_.send(data);
}

off_highway_premium_radar_msgs::msg::SensorStateInformation TestRadarDriver::get_sensor_state_info()
{
  auto ret = executor_.spin_until_future_complete(future_, spin_timeout_);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  return received_sensor_state_info_;
}

void TestRadarDriver::verify_sensor_state_info(
  off_highway_premium_radar::SensorStateInformation ref_sensor_state_info,
  off_highway_premium_radar_msgs::msg::SensorStateInformation sub_sensor_state_info)
{
  EXPECT_EQ(
    sub_sensor_state_info.sensor_state,
    ref_sensor_state_info.sensor_state_data.SenStInfo_SenSt);
}

TEST_F(TestRadarDriver, testSensorStateInformationZeroValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);
  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;
  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt = 0;
  send_sensor_state_info(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_sensor_state_info());
}

TEST_F(TestRadarDriver, testSensorStateInformationAnyValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);
  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;
  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt = 115;
  send_sensor_state_info(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_sensor_state_info());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
