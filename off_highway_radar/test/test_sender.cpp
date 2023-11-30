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

#include "off_highway_radar/sender.hpp"
#include "off_highway_can/helper.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;
static constexpr double kRadToDegree = 180. / M_PI;

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher(double velocity, double yaw)
  : Node("off_highway_radar_velocity_pub")
  {
    // Initialize publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1);

    // Publish velocity topic
    geometry_msgs::msg::TwistStamped radar_publish;
    radar_publish.twist.linear.x = velocity;
    radar_publish.twist.angular.z = yaw;
    radar_publish.header.stamp = now();
    publisher_->publish(radar_publish);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};  // VelocityPublisher

class VelocitySubscriber : public rclcpp::Node
{
public:
  VelocitySubscriber()
  : Node("off_highway_radar_velocity_sub")
  {
    subscriber_ =
      this->create_subscription<can_msgs::msg::Frame>(
      "to_can_bus", 2,
      std::bind(&VelocitySubscriber::sub_callback, this, std::placeholders::_1));
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
};  // VelocitySubscriber

class TestRadarSender : public ::testing::Test
{
protected:
  can_msgs::msg::Frame can_velocity_expected;
  can_msgs::msg::Frame can_yaw_expected;

  void create_sender();
  void publish_velocity(double velocity, double yaw);
  std::vector<can_msgs::msg::Frame> get_can_buffer();
  void verify_velocity(
    can_msgs::msg::Frame velocity_expected, can_msgs::msg::Frame yaw_expected,
    std::vector<can_msgs::msg::Frame> received_buffer);
  void reset();
  void run_test(double velocity, double yaw);

private:
  void spinSender(const std::chrono::nanoseconds & duration);
  void spinSubscriber(const std::chrono::nanoseconds & duration);

  off_highway_radar::Sender::SharedPtr node_;

  std::shared_ptr<VelocityPublisher> velocity_publisher_;
  std::shared_ptr<VelocitySubscriber> velocity_subscriber_;
};

void TestRadarSender::create_sender()
{
  node_ = std::make_shared<off_highway_radar::Sender>("radar_sender_test_node");

  // Get parameter values from yaml file
  std::string package_directory =
    ament_index_cpp::get_package_share_directory("off_highway_radar");
  std::string filename = "/config/sender_params.yaml";
  auto params = rclcpp::parameter_map_from_yaml_file(package_directory + filename);
  ASSERT_TRUE(
    node_->set_parameters_atomically(params.at("/off_highway_radar_sender")).successful);

  // Overwrite allowed age to avoid timing issues in unit tests
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("allowed_age", 0.1)).successful);

  velocity_subscriber_ = std::make_shared<VelocitySubscriber>();
}

void TestRadarSender::publish_velocity(double velocity, double yaw)
{
  velocity_publisher_ = std::make_shared<VelocityPublisher>(velocity, yaw);

  spinSender(10ms);

  // Generate and encode expected CAN values
  static const uint32_t velocity_id = 0x50;
  static const uint32_t yaw_id = 0x174;

  off_highway_radar::Sender sender;
  auto msg_def = sender.get_messages();

  // Set up expected velocity CAN message
  auto_static_cast(can_velocity_expected.id, velocity_id);
  auto_static_cast(msg_def[velocity_id].signals["v"].value, velocity);
  msg_def[velocity_id].encode(can_velocity_expected.data);

  // Set up expected yaw CAN message
  auto_static_cast(can_yaw_expected.id, yaw_id);
  auto_static_cast(msg_def[yaw_id].signals["psidt"].value, yaw * kRadToDegree);
  msg_def[yaw_id].encode(can_yaw_expected.data);
}

std::vector<can_msgs::msg::Frame> TestRadarSender::get_can_buffer()
{
  spinSubscriber(20ms);  // Needs to spin twice to receive both can messages
  return velocity_subscriber_->get_can_buffer();
}

void TestRadarSender::spinSender(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(10ms);
  }
}

void TestRadarSender::spinSubscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(velocity_subscriber_);
    rclcpp::sleep_for(10ms);
  }
}

void TestRadarSender::verify_velocity(
  can_msgs::msg::Frame velocity_expected,
  can_msgs::msg::Frame yaw_expected,
  std::vector<can_msgs::msg::Frame> received_buffer)
{
  EXPECT_EQ(node_->count_publishers("to_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("to_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("velocity"), 1U);
  EXPECT_EQ(node_->count_subscribers("velocity"), 1U);

  static const uint32_t velocity_id = 0x50;
  static const uint32_t yaw_id = 0x174;

  // Check velocity and yaw messages
  bool velocity_received = false;
  bool yaw_received = false;
  for (can_msgs::msg::Frame can_msg : received_buffer) {
    switch (can_msg.id) {
      case velocity_id:
        EXPECT_EQ(can_msg.data, velocity_expected.data);
        velocity_received = true;
        break;
      case yaw_id:
        EXPECT_EQ(can_msg.data, yaw_expected.data);
        yaw_received = true;
        break;
    }
    EXPECT_FALSE(can_msg.is_extended);
    EXPECT_FALSE(can_msg.is_error);
    EXPECT_FALSE(can_msg.is_rtr);
  }

  EXPECT_TRUE(velocity_received);
  EXPECT_TRUE(yaw_received);
}

void TestRadarSender::reset()
{
  velocity_publisher_.reset();
  velocity_subscriber_.reset();
  node_.reset();
}

void TestRadarSender::run_test(double velocity, double yaw)
{
  create_sender();
  publish_velocity(velocity, yaw);
  std::vector<can_msgs::msg::Frame> received_can_buffer_ = get_can_buffer();

  static const double velocity_min = -81.92;
  static const double velocity_max = 81.9175;
  static const double yaw_min = -163.84 / kRadToDegree;
  static const double yaw_max = 163.83 / kRadToDegree;
  // If velocity or yaw is out of range, nothing should have been received
  if ((velocity < velocity_min || velocity > velocity_max) || (yaw < yaw_min || yaw > yaw_max)) {
    EXPECT_TRUE(received_can_buffer_.empty());
    return;
  }
  EXPECT_EQ(received_can_buffer_.size(), 2u);

  verify_velocity(can_velocity_expected, can_yaw_expected, received_can_buffer_);
  reset();
}


TEST_F(TestRadarSender, validValues) {
  run_test(0, 0);
  run_test(-10, -1);
  run_test(10, 1);
  run_test(-10.1, -1.1);
  run_test(10.1, 1.1);
  run_test(-81.92, -163.84 / kRadToDegree);
  run_test(81.9175, 163.83 / kRadToDegree);
}

TEST_F(TestRadarSender, invalidScale) {
  run_test(10.0036, 10.003 / kRadToDegree);
  run_test(11.1111, 33.3333 / kRadToDegree);
  run_test(33.3333, 77.7777 / kRadToDegree);
}

TEST_F(TestRadarSender, outOfRange) {
  run_test(-81.921, -163.845 / kRadToDegree);
  run_test(81.91751, 163.835 / kRadToDegree);
  run_test(-83, 0);
  run_test(83, 0);
  run_test(0, -164 / kRadToDegree);
  run_test(0, 164 / kRadToDegree);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
