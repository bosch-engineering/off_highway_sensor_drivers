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

#include "gtest/gtest.h"

#include "off_highway_uss/sender.hpp"
#include "off_highway_can/helper.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

class TemperaturePublisher : public rclcpp::Node
{
public:
  explicit TemperaturePublisher(double temperature)
  : Node("off_highway_uss_temperature_pub")
  {
    // Initialize publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 1);

    // Publish temperature topic
    sensor_msgs::msg::Temperature temperature_publish;
    temperature_publish.temperature = temperature;
    temperature_publish.header.stamp = now();
    publisher_->publish(temperature_publish);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
};  // TemperaturePublisher

class TemperatureSubscriber : public rclcpp::Node
{
public:
  TemperatureSubscriber()
  : Node("off_highway_uss_temperature_sub")
  {
    subscriber_ =
      this->create_subscription<can_msgs::msg::Frame>(
      "to_can_bus", 2,
      std::bind(&TemperatureSubscriber::sub_callback, this, std::placeholders::_1));
  }

  std::vector<can_msgs::msg::Frame> get_can_buffer()
  {
    return sent_message_buffer;
  }

private:
  void sub_callback(const can_msgs::msg::Frame msg)
  {
    sent_message_buffer.push_back(msg);
  }
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_;
  std::vector<can_msgs::msg::Frame> sent_message_buffer;
};  // TemperatureSubscriber

class TestUssSender : public ::testing::Test
{
protected:
  std::vector<can_msgs::msg::Frame> received_can_buffer_;
  can_msgs::msg::Frame can_temperature_expected;

  void create_sender();
  void publish_temperature(double temperature);
  std::vector<can_msgs::msg::Frame> get_can_buffer();
  void reset();
  void run_test(double temperature);

private:
  void spin_sender(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);
  static constexpr uint32_t temperature_id = 0xB500000;

  off_highway_uss::Sender::SharedPtr node_;

  std::shared_ptr<TemperaturePublisher> temperature_publisher_;
  std::shared_ptr<TemperatureSubscriber> temperature_subscriber_;
};

void TestUssSender::create_sender()
{
  node_ = std::make_shared<off_highway_uss::Sender>("uss_sender_test_node");

  // Get parameter values from yaml file
  std::string package_directory =
    ament_index_cpp::get_package_share_directory("off_highway_uss");
  std::string filename = "/config/sender_params.yaml";
  auto params = rclcpp::parameter_map_from_yaml_file(package_directory + filename);
  ASSERT_TRUE(
    node_->set_parameters_atomically(params.at("/off_highway_uss_sender")).successful);

  // Overwrite allowed age to avoid timing issues in unit tests
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("allowed_age", 0.1)).successful);

  temperature_subscriber_ = std::make_shared<TemperatureSubscriber>();
}

void TestUssSender::publish_temperature(double temperature)
{
  temperature_publisher_ = std::make_shared<TemperaturePublisher>(temperature);

  spin_sender(10ms);

  // Generate and encode expected CAN values
  off_highway_uss::Sender sender;
  auto msg_def = sender.get_messages();

  // Set up expected temperature CAN message
  auto_static_cast(can_temperature_expected.id, temperature_id);
  auto_static_cast(msg_def[temperature_id].signals["OutsideTemperature"].value, temperature);
  msg_def[temperature_id].encode(can_temperature_expected.data);
}

std::vector<can_msgs::msg::Frame> TestUssSender::get_can_buffer()
{
  spin_subscriber(20ms);  // Needs to spin twice to receive both can messages
  return temperature_subscriber_->get_can_buffer();
}

void TestUssSender::spin_sender(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(10ms);
  }
}

void TestUssSender::spin_subscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(temperature_subscriber_);
    rclcpp::sleep_for(10ms);
  }
}

void TestUssSender::reset()
{
  temperature_publisher_.reset();
}

void TestUssSender::run_test(double temperature)
{
  create_sender();
  publish_temperature(temperature);
  received_can_buffer_ = get_can_buffer();

  static const double temperature_min = -40.0;
  static const double temperature_max = 87.0;

  // If value is in valid range, compare value and ID to expected values
  if (temperature >= temperature_min && temperature <= temperature_max) {
    EXPECT_EQ(received_can_buffer_.size(), 1u);

    auto received = received_can_buffer_.front();
    EXPECT_EQ(received.data, can_temperature_expected.data);
    EXPECT_EQ(received.id, temperature_id);
    EXPECT_TRUE(received.is_extended);
    EXPECT_FALSE(received.is_error);
    EXPECT_FALSE(received.is_rtr);
  } else {
    EXPECT_TRUE(received_can_buffer_.empty());
  }
  reset();
}

TEST_F(TestUssSender, temperatureValidRange) {
  // Min, max, 0 and value in valid range
  run_test(-40);
  run_test(87);
  run_test(0);
  run_test(20);
}

TEST_F(TestUssSender, temperatureInvalidScale) {
  // Since scale of the CAN msg is 1, decimal places shall be cut off
  run_test(20.1);
  run_test(20.5);
  run_test(20.9);
}

TEST_F(TestUssSender, temperatureOutOfRange) {
  // One value < min and one > max
  run_test(-41);
  run_test(-40.1);
  run_test(87.1);
  run_test(88);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
