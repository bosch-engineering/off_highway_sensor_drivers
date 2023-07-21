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

#include <random>

#include "gtest/gtest.h"

#include "off_highway_radar/receiver.hpp"
#include "off_highway_common/helper.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using off_highway_common::auto_static_cast;
using namespace std::chrono_literals;

class InfoPublisher : public rclcpp::Node
{
public:
  explicit InfoPublisher(off_highway_radar_msgs::msg::Information information)
  : Node("off_highway_radar_info_pub")
  {
    off_highway_radar::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_info;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Cast and encode information values
    auto_static_cast(can_msg_info.id, 0x100);
    auto_static_cast(can_msg_info.header.stamp, now());
    off_highway_common::Message & info_msg = msg_def[can_msg_info.id];
    auto_static_cast(info_msg.signals["SensorType"].value, information.sensor_type);
    auto_static_cast(info_msg.signals["HWTemperature"].value, information.hw_temperature);
    auto_static_cast(info_msg.signals["SensorBlind"].value, information.sensor_blind);
    auto_static_cast(info_msg.signals["SWFail"].value, information.sw_fail);
    auto_static_cast(info_msg.signals["HWFail"].value, information.hw_fail);
    auto_static_cast(info_msg.signals["CANFail"].value, information.can_fail);
    auto_static_cast(info_msg.signals["ConfigFail"].value, information.config_fail);
    auto_static_cast(info_msg.signals["DiagMode"].value, information.diag_mode);
    auto_static_cast(info_msg.signals["DTC"].value, information.dtc);
    auto_static_cast(info_msg.signals["DTCOrderId"].value, information.dtc_order_id);
    auto_static_cast(info_msg.signals["SensorNotSafe"].value, information.sensor_not_safe);
    auto_static_cast(info_msg.signals["ReceptionError"].value, information.reception_error);

    // Encode message
    info_msg.encode(can_msg_info.data);

    // Publish
    publisher_->publish(can_msg_info);
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};  // InfoPublisher

class InfoSubscriber : public rclcpp::Node
{
public:
  InfoSubscriber()
  : Node("off_highway_radar_info_sub")
  {
    subscriber_ = this->create_subscription<off_highway_radar_msgs::msg::Information>(
      "info", 1,
      std::bind(&InfoSubscriber::info_callback, this, std::placeholders::_1));
  }

  off_highway_radar_msgs::msg::Information get_info()
  {
    return received_info_;
  }

private:
  void info_callback(const off_highway_radar_msgs::msg::Information msg)
  {
    received_info_ = msg;
  }
  rclcpp::Subscription<off_highway_radar_msgs::msg::Information>::SharedPtr subscriber_;
  off_highway_radar_msgs::msg::Information received_info_;
};  // InfoSubscriber

class TestRadarReceiver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<off_highway_radar::Receiver>("radar_receiver_test_node");

    // Get parameter values from yaml file
    std::string package_directory =
      ament_index_cpp::get_package_share_directory("off_highway_radar");
    std::string filename = "/config/receiver_params.yaml";
    auto params = rclcpp::parameter_map_from_yaml_file(package_directory + filename);
    ASSERT_TRUE(
      node_->set_parameters_atomically(params.at("/off_highway_radar_receiver")).successful);

    // Overwrite allowed age to avoid timing issues in unit tests
    ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("allowed_age", 0.1)).successful);

    info_subscriber_ = std::make_shared<InfoSubscriber>();

    rclcpp::spin_some(node_);
  }

  void publish_info(off_highway_radar_msgs::msg::Information information);
  off_highway_radar_msgs::msg::Information get_info();
  void verify_info(
    off_highway_radar_msgs::msg::Information ref_info,
    off_highway_radar_msgs::msg::Information sub_info);

private:
  void wait_some(const std::chrono::nanoseconds & duration);

  off_highway_radar::Receiver::SharedPtr node_;

  std::shared_ptr<InfoPublisher> info_publisher_;
  std::shared_ptr<InfoSubscriber> info_subscriber_;
};

void TestRadarReceiver::publish_info(off_highway_radar_msgs::msg::Information information)
{
  info_publisher_ = std::make_shared<InfoPublisher>(information);
  wait_some(100ms);
}

off_highway_radar_msgs::msg::Information TestRadarReceiver::get_info()
{
  rclcpp::spin_some(info_subscriber_);
  return info_subscriber_->get_info();
}

void TestRadarReceiver::wait_some(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(info_subscriber_);
  }
}

void TestRadarReceiver::verify_info(
  off_highway_radar_msgs::msg::Information ref_info,
  off_highway_radar_msgs::msg::Information sub_info)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("info"), 1U);
  EXPECT_EQ(node_->count_subscribers("info"), 1U);

  // Check info values
  EXPECT_EQ(sub_info.sensor_type, ref_info.sensor_type);
  EXPECT_EQ(sub_info.hw_temperature, ref_info.hw_temperature);
  EXPECT_EQ(sub_info.sensor_blind, ref_info.sensor_blind);
  EXPECT_EQ(sub_info.sw_fail, ref_info.sw_fail);
  EXPECT_EQ(sub_info.hw_fail, ref_info.hw_fail);
  EXPECT_EQ(sub_info.can_fail, ref_info.can_fail);
  EXPECT_EQ(sub_info.config_fail, ref_info.config_fail);
  EXPECT_EQ(sub_info.diag_mode, ref_info.diag_mode);
  EXPECT_EQ(sub_info.dtc, ref_info.dtc);
  EXPECT_EQ(sub_info.dtc_order_id, ref_info.dtc_order_id);
  EXPECT_EQ(sub_info.sensor_not_safe, ref_info.sensor_not_safe);
  EXPECT_EQ(sub_info.reception_error, ref_info.reception_error);
}

TEST_F(TestRadarReceiver, testInfoZeroValues) {
  // Set all values to 0
  off_highway_radar_msgs::msg::Information test_information;
  test_information.sensor_type = 0;
  test_information.hw_temperature = 0.0;
  test_information.sensor_blind = false;
  test_information.sw_fail = false;
  test_information.hw_fail = false;
  test_information.can_fail = false;
  test_information.config_fail = false;
  test_information.diag_mode = false;
  test_information.dtc = 0;
  test_information.dtc_order_id = 0;
  test_information.sensor_not_safe = false;
  test_information.reception_error = false;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestRadarReceiver, testInfoMinValues) {
  // Set values to min
  off_highway_radar_msgs::msg::Information test_information;
  test_information.sensor_type = 0;
  test_information.hw_temperature = -78;
  test_information.sensor_blind = false;
  test_information.sw_fail = false;
  test_information.hw_fail = false;
  test_information.can_fail = false;
  test_information.config_fail = false;
  test_information.diag_mode = false;
  test_information.dtc = 0;
  test_information.dtc_order_id = 0;
  test_information.sensor_not_safe = false;
  test_information.reception_error = false;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestRadarReceiver, testInfoMaxValues) {
  // Set values to max
  off_highway_radar_msgs::msg::Information test_information;
  test_information.sensor_type = 7;
  test_information.hw_temperature = 177;
  test_information.sensor_blind = true;
  test_information.sw_fail = true;
  test_information.hw_fail = true;
  test_information.can_fail = true;
  test_information.config_fail = true;
  test_information.diag_mode = true;
  test_information.dtc = 16777215;
  test_information.dtc_order_id = 255;
  test_information.sensor_not_safe = true;
  test_information.reception_error = true;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestRadarReceiver, testInfoAnyValues) {
  // Set any valid values
  off_highway_radar_msgs::msg::Information test_information;
  test_information.sensor_type = 4;
  test_information.hw_temperature = 100;
  test_information.sensor_blind = true;
  test_information.sw_fail = false;
  test_information.hw_fail = true;
  test_information.can_fail = false;
  test_information.config_fail = true;
  test_information.diag_mode = false;
  test_information.dtc = 123456;
  test_information.dtc_order_id = 123;
  test_information.sensor_not_safe = false;
  test_information.reception_error = true;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
