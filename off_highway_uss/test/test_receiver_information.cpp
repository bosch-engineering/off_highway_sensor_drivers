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
#include "rclcpp/executors.hpp"

#include "off_highway_uss/receiver.hpp"
#include "off_highway_can/helper.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

class InfoPublisher : public rclcpp::Node
{
public:
  explicit InfoPublisher(off_highway_uss_msgs::msg::Information information)
  : Node("off_highway_uss_info_pub")
  {
    off_highway_uss::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_info;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Cast and encode information values
    auto_static_cast(can_msg_info.id, 0x17C);
    auto_static_cast(can_msg_info.header.stamp, now());
    off_highway_can::Message & info_msg = msg_def[can_msg_info.id];
    auto_static_cast(info_msg.signals["NumberSensors"].value, information.number_sensors);
    auto_static_cast(info_msg.signals["SendingPattern"].value, information.sending_pattern);
    auto_static_cast(info_msg.signals["OperatingMode"].value, information.operating_mode);
    auto_static_cast(info_msg.signals["OutsideTemperature"].value, information.outside_temperature);
    auto_static_cast(info_msg.signals["SensorBlindness"].value, information.sensor_blindness);
    auto_static_cast(info_msg.signals["Sensitivity"].value, information.sensitivity);
    auto_static_cast(info_msg.signals["SensorFaulted"].value, information.sensor_faulted);
    auto_static_cast(info_msg.signals["FailureStatus"].value, information.failure_status);

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
  : Node("off_highway_uss_info_sub")
  {
    subscriber_ = this->create_subscription<off_highway_uss_msgs::msg::Information>(
      "info", 1,
      std::bind(&InfoSubscriber::info_callback, this, std::placeholders::_1));
  }

  off_highway_uss_msgs::msg::Information get_info()
  {
    return received_info_;
  }

private:
  void info_callback(const off_highway_uss_msgs::msg::Information msg)
  {
    received_info_ = msg;
  }
  rclcpp::Subscription<off_highway_uss_msgs::msg::Information>::SharedPtr subscriber_;
  off_highway_uss_msgs::msg::Information received_info_;
};  // InfoSubscriber

class TestUssReceiver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Overwrite allowed age to avoid timing issues in unit tests
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("allowed_age", 1.0)
    };
    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);
    node_ = std::make_shared<off_highway_uss::Receiver>("uss_receiver_test_node", node_options);

    ASSERT_EQ(node_->get_parameter("allowed_age").as_double(), 1.0);

    info_subscriber_ = std::make_shared<InfoSubscriber>();

    rclcpp::spin_some(node_);
  }

  void publish_info(off_highway_uss_msgs::msg::Information information);
  off_highway_uss_msgs::msg::Information get_info();
  void verify_info(
    off_highway_uss_msgs::msg::Information ref_info,
    off_highway_uss_msgs::msg::Information sub_info);

private:
  void wait_some(const std::chrono::nanoseconds & duration);

  off_highway_uss::Receiver::SharedPtr node_;

  std::shared_ptr<InfoPublisher> info_publisher_;
  std::shared_ptr<InfoSubscriber> info_subscriber_;
};

void TestUssReceiver::publish_info(off_highway_uss_msgs::msg::Information information)
{
  info_publisher_ = std::make_shared<InfoPublisher>(information);
  wait_some(500ms);
}

off_highway_uss_msgs::msg::Information TestUssReceiver::get_info()
{
  rclcpp::spin_some(info_subscriber_);
  off_highway_uss_msgs::msg::Information subscribed_info_ = info_subscriber_->get_info();
  return subscribed_info_;
}

void TestUssReceiver::wait_some(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(info_subscriber_);
  }
}

void TestUssReceiver::verify_info(
  off_highway_uss_msgs::msg::Information ref_info,
  off_highway_uss_msgs::msg::Information sub_info)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("info"), 1U);
  EXPECT_EQ(node_->count_subscribers("info"), 1U);

  // Check info values
  EXPECT_EQ(sub_info.number_sensors, ref_info.number_sensors);
  EXPECT_EQ(sub_info.sending_pattern, ref_info.sending_pattern);
  EXPECT_EQ(sub_info.operating_mode, ref_info.operating_mode);
  EXPECT_EQ(sub_info.outside_temperature, ref_info.outside_temperature);
  EXPECT_EQ(sub_info.sensor_blindness, ref_info.sensor_blindness);
  EXPECT_EQ(sub_info.sensitivity, ref_info.sensitivity);
  EXPECT_EQ(sub_info.sensor_faulted, ref_info.sensor_faulted);
  EXPECT_EQ(sub_info.failure_status, ref_info.failure_status);
}

TEST_F(TestUssReceiver, testInfoZeroValues) {
  // Set all values to 0
  off_highway_uss_msgs::msg::Information test_information;
  test_information.number_sensors = 0;
  test_information.sending_pattern = 0;
  test_information.operating_mode = 0;
  test_information.outside_temperature = 0;
  test_information.sensor_blindness = 0;
  test_information.sensitivity = 0;
  test_information.sensor_faulted = 0;
  test_information.failure_status = 0;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestUssReceiver, testInfoMinValues) {
  // Set values to min
  off_highway_uss_msgs::msg::Information test_information;
  test_information.number_sensors = 0;
  test_information.sending_pattern = 0;
  test_information.operating_mode = 0;
  test_information.outside_temperature = -40;
  test_information.sensor_blindness = 0;
  test_information.sensitivity = 0;
  test_information.sensor_faulted = 0;
  test_information.failure_status = 0;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestUssReceiver, testInfoMaxValues) {
  // Set values to max
  off_highway_uss_msgs::msg::Information test_information;
  test_information.number_sensors = 7;
  test_information.sending_pattern = 3;
  test_information.operating_mode = 7;
  test_information.outside_temperature = 215;
  test_information.sensor_blindness = 4095;
  test_information.sensitivity = 7;
  test_information.sensor_faulted = 4095;
  test_information.failure_status = 1;

  publish_info(test_information);
  verify_info(test_information, get_info());
}

TEST_F(TestUssReceiver, testInfoAnyValues) {
  // Set any valid values
  off_highway_uss_msgs::msg::Information test_information;
  test_information.number_sensors = 4;
  test_information.sending_pattern = 2;
  test_information.operating_mode = 1;
  test_information.outside_temperature = 123;
  test_information.sensor_blindness = 1234;
  test_information.sensitivity = 3;
  test_information.sensor_faulted = 2345;
  test_information.failure_status = 0;

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
