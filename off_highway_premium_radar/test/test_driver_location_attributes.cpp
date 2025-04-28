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
    using off_highway_premium_radar_msgs::msg::LocationAttributes;

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
    location_attributes_node_ = std::make_shared<rclcpp::Node>("location_attributes_subscriber");
    location_attributes_subscription_ =
      location_attributes_node_->create_subscription<LocationAttributes>(
      "/driver/location_attributes", 10,
      [&](const LocationAttributes msg) {
        received_location_attributes_ = msg;
        promise_.set_value(true);
      });

    executor_.add_node(node_);
    executor_.add_node(location_attributes_node_);
    // Spin node to configure driver
    executor_.spin_some();
  }
  void send_location_attributes(off_highway_premium_radar::LocationAttributes info);
  off_highway_premium_radar_msgs::msg::LocationAttributes get_location_attributes();
  void verify_location_attributes(
    off_highway_premium_radar::LocationAttributes ref_location_attributes,
    off_highway_premium_radar_msgs::msg::LocationAttributes sub_location_attributes);

private:
  const int sensor_port_ = 0x76C6;
  const int host_port_ = 0x76C0;
  const std::string loopback_ip_ = "127.0.0.1";
  const std::chrono::seconds spin_timeout_ = 1s;

  IoContext ctx_;
  off_highway_premium_radar::UdpSocket udp_socket_;

  std::shared_ptr<NodeWithDefaultConverter> node_;

  rclcpp::Node::SharedPtr location_attributes_node_;
  rclcpp::Subscription<off_highway_premium_radar_msgs::msg::LocationAttributes>::
  SharedPtr
    location_attributes_subscription_;
  off_highway_premium_radar_msgs::msg::LocationAttributes received_location_attributes_;

  std::promise<bool> promise_;
  std::shared_future<bool> future_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void TestRadarDriver::send_location_attributes(
  off_highway_premium_radar::LocationAttributes info)
{
  auto data = info.serialize();
  udp_socket_.send(data);
}

off_highway_premium_radar_msgs::msg::LocationAttributes TestRadarDriver::get_location_attributes()
{
  auto ret = executor_.spin_until_future_complete(future_, spin_timeout_);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  return received_location_attributes_;
}

void TestRadarDriver::verify_location_attributes(
  off_highway_premium_radar::LocationAttributes ref_location_attributes,
  off_highway_premium_radar_msgs::msg::LocationAttributes sub_location_attributes)
{
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.theta_indicator_mimo,
    ref_location_attributes.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.theta_indicator_mimo_valid,
    ref_location_attributes.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.phi_indicator,
    ref_location_attributes.loc_atr_packet.sensor_coating.mdPhiIndcr);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.phi_indicator_valid,
    ref_location_attributes.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.reflections_indicator,
    ref_location_attributes.loc_atr_packet.sensor_coating.nRefIndcr);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.reflections_indicator_valid,
    ref_location_attributes.loc_atr_packet.sensor_coating.nRefIndcrVldFlg);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.theta_mimo_rate,
    ref_location_attributes.loc_atr_packet.sensor_coating.thetaMIMORate);
  EXPECT_EQ(
    sub_location_attributes.location_attributes_packet.sensor_coating.theta_mimo_rate_valid,
    ref_location_attributes.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag);
}

TEST_F(TestRadarDriver, testLocationAttributesZeroValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);
  test_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = 0.0F;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 0;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = 0.0F;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 0;
  test_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = 0.0F;
  test_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 0;
  test_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = 0.0F;
  test_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 0;
  send_location_attributes(test_location_attributes_);
  verify_location_attributes(test_location_attributes_, get_location_attributes());
}

TEST_F(TestRadarDriver, testLocationAttributesAnyValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);
  test_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = 0.42F;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 1;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = 0.42F;
  test_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 1;
  test_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = 0.42F;
  test_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 1;
  test_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = 0.42F;
  test_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 1;
  send_location_attributes(test_location_attributes_);
  verify_location_attributes(test_location_attributes_, get_location_attributes());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
