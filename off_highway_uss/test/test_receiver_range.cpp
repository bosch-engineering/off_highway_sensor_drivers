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

static constexpr double kMetersToCentimeters = 0.01;
double range_scaling = 36.667 * kMetersToCentimeters;

class RangePublisher : public rclcpp::Node
{
public:
  explicit RangePublisher(off_highway_uss_msgs::msg::MaxDetectionRange range)
  : Node("off_highway_uss_range_pub")
  {
    off_highway_uss::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_range;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Cast and encode range values
    auto_static_cast(can_msg_range.id, 0x17D);
    auto_static_cast(can_msg_range.header.stamp, now());
    off_highway_can::Message & range_msg = msg_def[can_msg_range.id];
    auto_static_cast(range_msg.signals["sens01_MaxDetRange"].value, range.max_detection_ranges[0]);
    auto_static_cast(range_msg.signals["sens02_MaxDetRange"].value, range.max_detection_ranges[1]);
    auto_static_cast(range_msg.signals["sens03_MaxDetRange"].value, range.max_detection_ranges[2]);
    auto_static_cast(range_msg.signals["sens04_MaxDetRange"].value, range.max_detection_ranges[3]);
    auto_static_cast(range_msg.signals["sens05_MaxDetRange"].value, range.max_detection_ranges[4]);
    auto_static_cast(range_msg.signals["sens06_MaxDetRange"].value, range.max_detection_ranges[5]);
    auto_static_cast(range_msg.signals["sens07_MaxDetRange"].value, range.max_detection_ranges[6]);
    auto_static_cast(range_msg.signals["sens08_MaxDetRange"].value, range.max_detection_ranges[7]);
    auto_static_cast(range_msg.signals["sens09_MaxDetRange"].value, range.max_detection_ranges[8]);
    auto_static_cast(range_msg.signals["sens10_MaxDetRange"].value, range.max_detection_ranges[9]);
    auto_static_cast(range_msg.signals["sens11_MaxDetRange"].value, range.max_detection_ranges[10]);
    auto_static_cast(range_msg.signals["sens12_MaxDetRange"].value, range.max_detection_ranges[11]);

    // Encode message
    range_msg.encode(can_msg_range.data);

    // Publish
    publisher_->publish(can_msg_range);
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};  // RangePublisher

class RangeSubscriber : public rclcpp::Node
{
public:
  RangeSubscriber()
  : Node("off_highway_uss_range_sub")
  {
    subscriber_ = this->create_subscription<off_highway_uss_msgs::msg::MaxDetectionRange>(
      "max_detection_range", 1,
      std::bind(&RangeSubscriber::range_callback, this, std::placeholders::_1));
  }

  off_highway_uss_msgs::msg::MaxDetectionRange get_range()
  {
    return received_range_;
  }

private:
  void range_callback(const off_highway_uss_msgs::msg::MaxDetectionRange msg)
  {
    received_range_ = msg;
  }
  rclcpp::Subscription<off_highway_uss_msgs::msg::MaxDetectionRange>::SharedPtr subscriber_;
  off_highway_uss_msgs::msg::MaxDetectionRange received_range_;
};  // RangeSubscriber

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

    range_subscriber_ = std::make_shared<RangeSubscriber>();

    rclcpp::spin_some(node_);
  }

  void publish_range(off_highway_uss_msgs::msg::MaxDetectionRange range);
  off_highway_uss_msgs::msg::MaxDetectionRange get_range();
  void verify_range(
    off_highway_uss_msgs::msg::MaxDetectionRange ref_range,
    off_highway_uss_msgs::msg::MaxDetectionRange sub_range);

private:
  void wait_some(const std::chrono::nanoseconds & duration);

  off_highway_uss::Receiver::SharedPtr node_;

  std::shared_ptr<RangePublisher> range_publisher_;
  std::shared_ptr<RangeSubscriber> range_subscriber_;
};

void TestUssReceiver::publish_range(off_highway_uss_msgs::msg::MaxDetectionRange range)
{
  range_publisher_ = std::make_shared<RangePublisher>(range);
  wait_some(500ms);
}

off_highway_uss_msgs::msg::MaxDetectionRange TestUssReceiver::get_range()
{
  rclcpp::spin_some(range_subscriber_);
  return range_subscriber_->get_range();
}

void TestUssReceiver::wait_some(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(range_subscriber_);
  }
}

void TestUssReceiver::verify_range(
  off_highway_uss_msgs::msg::MaxDetectionRange ref_range,
  off_highway_uss_msgs::msg::MaxDetectionRange sub_range)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("max_detection_range"), 1U);
  EXPECT_EQ(node_->count_subscribers("max_detection_range"), 1U);

  // Check range values
  for (auto i = 0; i < 12; i++) {
    EXPECT_NEAR(
      sub_range.max_detection_ranges[i], ref_range.max_detection_ranges[i], 0.36667);
  }
}

TEST_F(TestUssReceiver, testRangeMinValues) {
  // Set all values to 0
  off_highway_uss_msgs::msg::MaxDetectionRange test_range;
  test_range.max_detection_ranges[0] = 0;
  test_range.max_detection_ranges[1] = 0;
  test_range.max_detection_ranges[2] = 0;
  test_range.max_detection_ranges[3] = 0;
  test_range.max_detection_ranges[4] = 0;
  test_range.max_detection_ranges[5] = 0;
  test_range.max_detection_ranges[6] = 0;
  test_range.max_detection_ranges[7] = 0;
  test_range.max_detection_ranges[8] = 0;
  test_range.max_detection_ranges[9] = 0;
  test_range.max_detection_ranges[10] = 0;
  test_range.max_detection_ranges[11] = 0;

  publish_range(test_range);
  verify_range(test_range, get_range());
}

TEST_F(TestUssReceiver, testRangeMaxValues) {
  // Set all values to 0
  off_highway_uss_msgs::msg::MaxDetectionRange test_range;
  test_range.max_detection_ranges[0] = 15 * range_scaling;  // = 550.005 * 0.01
  test_range.max_detection_ranges[1] = 15 * range_scaling;
  test_range.max_detection_ranges[2] = 15 * range_scaling;
  test_range.max_detection_ranges[3] = 15 * range_scaling;
  test_range.max_detection_ranges[4] = 15 * range_scaling;
  test_range.max_detection_ranges[5] = 15 * range_scaling;
  test_range.max_detection_ranges[6] = 15 * range_scaling;
  test_range.max_detection_ranges[7] = 15 * range_scaling;
  test_range.max_detection_ranges[8] = 15 * range_scaling;
  test_range.max_detection_ranges[9] = 15 * range_scaling;
  test_range.max_detection_ranges[10] = 15 * range_scaling;
  test_range.max_detection_ranges[11] = 15 * range_scaling;

  publish_range(test_range);
  verify_range(test_range, get_range());
}

TEST_F(TestUssReceiver, testRangeAnyValues) {
  // Set all values to any
  off_highway_uss_msgs::msg::MaxDetectionRange test_range;
  test_range.max_detection_ranges[0] = 1 * range_scaling;
  test_range.max_detection_ranges[1] = 2 * range_scaling;
  test_range.max_detection_ranges[2] = 3 * range_scaling;
  test_range.max_detection_ranges[3] = 4 * range_scaling;
  test_range.max_detection_ranges[4] = 5 * range_scaling;
  test_range.max_detection_ranges[5] = 6 * range_scaling;
  test_range.max_detection_ranges[6] = 7 * range_scaling;
  test_range.max_detection_ranges[7] = 8 * range_scaling;
  test_range.max_detection_ranges[8] = 9 * range_scaling;
  test_range.max_detection_ranges[9] = 10 * range_scaling;
  test_range.max_detection_ranges[10] = 11 * range_scaling;
  test_range.max_detection_ranges[11] = 12 * range_scaling;

  publish_range(test_range);
  verify_range(test_range, get_range());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
