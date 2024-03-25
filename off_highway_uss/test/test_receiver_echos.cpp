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

#include "test_helper.hpp"
#include "off_highway_uss/receiver.hpp"
#include "off_highway_can/helper.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

class EchosPublisher : public rclcpp::Node
{
public:
  explicit EchosPublisher(off_highway_uss_msgs::msg::DirectEchos test_echos)
  : Node("off_highway_uss_echos_pub")
  {
    off_highway_uss::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_echo;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Publish echos
    for (off_highway_uss_msgs::msg::DirectEcho test_echo : test_echos.direct_echos) {
      // Cast echo
      auto_static_cast(can_msg_echo.id, 0x170 + test_echo.id);
      auto_static_cast(can_msg_echo.header.stamp, now());
      off_highway_can::Message & echo_msg = msg_def[can_msg_echo.id];
      auto_static_cast(echo_msg.signals["De1Distance"].value, test_echo.first.distance);
      auto_static_cast(echo_msg.signals["De2Distance"].value, test_echo.second.distance);
      auto_static_cast(
        echo_msg.signals["De1FilteredDistance"].value,
        test_echo.first_filtered.distance);
      auto_static_cast(echo_msg.signals["Amplitude1"].value, test_echo.first.amplitude);
      auto_static_cast(echo_msg.signals["Amplitude2"].value, test_echo.second.amplitude);
      auto_static_cast(
        echo_msg.signals["FilteredAmplitude1"].value,
        test_echo.first_filtered.amplitude);

      // Encode message
      echo_msg.encode(can_msg_echo.data);

      // Publish
      publisher_->publish(can_msg_echo);
      defined_echo_ids++;
    }
  }

  uint8_t get_defined_echo_ids()
  {
    return defined_echo_ids;
  }

protected:
  uint8_t defined_echo_ids = 0;

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};  // EchosPublisher

class EchosSubscriber : public rclcpp::Node
{
public:
  EchosSubscriber()
  : Node("off_highway_uss_echos_sub")
  {
    subscriber_ = this->create_subscription<off_highway_uss_msgs::msg::DirectEchos>(
      "direct_echos", 2,
      std::bind(&EchosSubscriber::echos_callback, this, std::placeholders::_1));
  }

  off_highway_uss_msgs::msg::DirectEchos get_echos()
  {
    return received_echos_;
  }

private:
  void echos_callback(const off_highway_uss_msgs::msg::DirectEchos msg)
  {
    received_echos_ = msg;
  }
  rclcpp::Subscription<off_highway_uss_msgs::msg::DirectEchos>::SharedPtr subscriber_;
  off_highway_uss_msgs::msg::DirectEchos received_echos_;
};  // EchosSubscriber

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

    echos_subscriber_ = std::make_shared<EchosSubscriber>();
  }

  void publish_echos(off_highway_uss_msgs::msg::DirectEchos echos);
  off_highway_uss_msgs::msg::DirectEchos get_echos();
  void verify_echos(
    off_highway_uss_msgs::msg::DirectEchos test_echos,
    off_highway_uss_msgs::msg::DirectEchos received_echos);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  off_highway_uss::Receiver::SharedPtr node_;

  std::shared_ptr<EchosPublisher> echos_publisher_;
  std::shared_ptr<EchosSubscriber> echos_subscriber_;
};

void TestUssReceiver::publish_echos(off_highway_uss_msgs::msg::DirectEchos echos)
{
  echos_publisher_ = std::make_shared<EchosPublisher>(echos);
  spin_receiver(60ms);
}

off_highway_uss_msgs::msg::DirectEchos TestUssReceiver::get_echos()
{
  spin_subscriber(500ms);
  off_highway_uss_msgs::msg::DirectEchos subscribed_echos_ = echos_subscriber_->get_echos();
  return subscribed_echos_;
}

void TestUssReceiver::spin_receiver(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
  }
}

void TestUssReceiver::spin_subscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(echos_subscriber_);
    rclcpp::sleep_for(100ms);
  }
}

void TestUssReceiver::verify_echos(
  off_highway_uss_msgs::msg::DirectEchos test_echos,
  off_highway_uss_msgs::msg::DirectEchos received_echos)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("direct_echos"), 1U);
  EXPECT_EQ(node_->count_subscribers("direct_echos"), 1U);

  // Check echo
  uint8_t found_echo_ids = 0;
  for (off_highway_uss_msgs::msg::DirectEcho received_echo : received_echos.direct_echos) {
    // Search received echo in test echos
    off_highway_uss_msgs::msg::DirectEcho current_test_echo;
    for (off_highway_uss_msgs::msg::DirectEcho test_echo : test_echos.direct_echos) {
      if (received_echo.id == test_echo.id) {
        current_test_echo = test_echo;
        break;
      }
    }

    EXPECT_EQ(received_echo.first.distance, current_test_echo.first.distance);
    EXPECT_EQ(received_echo.second.distance, current_test_echo.second.distance);
    EXPECT_EQ(received_echo.first_filtered.distance, current_test_echo.first_filtered.distance);
    EXPECT_EQ(received_echo.first.amplitude, current_test_echo.first.amplitude);
    EXPECT_EQ(received_echo.second.amplitude, current_test_echo.second.amplitude);
    EXPECT_EQ(received_echo.first_filtered.amplitude, current_test_echo.first_filtered.amplitude);
    found_echo_ids++;
  }
  EXPECT_EQ(echos_publisher_->get_defined_echo_ids(), found_echo_ids);
}

TEST_F(TestUssReceiver, testEchosZero) {
  off_highway_uss_msgs::msg::DirectEchos test_echos;
  off_highway_uss_msgs::msg::DirectEcho test_echo_0;
  // Set all values to 0
  test_echo_0.id = 0;
  test_echo_0.first.distance = 0;
  test_echo_0.second.distance = 0;
  test_echo_0.first_filtered.distance = 0;
  test_echo_0.first.amplitude = 0;
  test_echo_0.second.amplitude = 0;
  test_echo_0.first_filtered.amplitude = 0;
  test_echos.direct_echos.push_back(test_echo_0);

  publish_echos(test_echos);
  verify_echos(test_echos, get_echos());
}

TEST_F(TestUssReceiver, testEchosMin) {
  off_highway_uss_msgs::msg::DirectEchos test_echos;
  off_highway_uss_msgs::msg::DirectEcho test_echo_min;
  // Set all values to min
  test_echo_min.id = 0;
  test_echo_min.first.distance = 0;
  test_echo_min.second.distance = 0;
  test_echo_min.first_filtered.distance = 0;
  test_echo_min.first.amplitude = 0;
  test_echo_min.second.amplitude = 0;
  test_echo_min.first_filtered.amplitude = 0;
  test_echos.direct_echos.push_back(test_echo_min);

  publish_echos(test_echos);
  verify_echos(test_echos, get_echos());
}

TEST_F(TestUssReceiver, testEchosMax) {
  off_highway_uss_msgs::msg::DirectEchos test_echos;
  off_highway_uss_msgs::msg::DirectEcho test_echo_max;
  // Set all values to max
  test_echo_max.id = 10;
  test_echo_max.first.distance = 1023;
  test_echo_max.second.distance = 1023;
  test_echo_max.first_filtered.distance = 1023;
  test_echo_max.first.amplitude = 63;
  test_echo_max.second.amplitude = 63;
  test_echo_max.first_filtered.amplitude = 63;
  test_echos.direct_echos.push_back(test_echo_max);

  publish_echos(test_echos);
  verify_echos(test_echos, get_echos());
}

TEST_F(TestUssReceiver, testEchosRand) {
  off_highway_uss_msgs::msg::DirectEchos test_echos;
  off_highway_uss_msgs::msg::DirectEcho test_echo_rand;
  // Set all values to random
  test_echo_rand.id = 5;
  test_echo_rand.first.distance = 555;
  test_echo_rand.second.distance = 222;
  test_echo_rand.first_filtered.distance = 111;
  test_echo_rand.first.amplitude = 33;
  test_echo_rand.second.amplitude = 22;
  test_echo_rand.first_filtered.amplitude = 11;
  test_echos.direct_echos.push_back(test_echo_rand);

  publish_echos(test_echos);
  verify_echos(test_echos, get_echos());
}

TEST_F(TestUssReceiver, test12RandomValidEchos) {
  off_highway_uss_msgs::msg::DirectEchos test_echos;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(12);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_uss_msgs::msg::DirectEcho test_echo;
    // Set values for object
    test_echo.id = id;
    auto gen_distance = RandomQuantizedGenerator{1, 0, 1023};
    test_echo.first.distance = gen_distance(rng);
    test_echo.second.distance = gen_distance(rng);
    test_echo.first_filtered.distance = gen_distance(rng);
    auto gen_amplitude = RandomQuantizedGenerator{1, 0, 63};
    test_echo.first.amplitude = gen_amplitude(rng);
    test_echo.second.amplitude = gen_amplitude(rng);
    test_echo.first_filtered.amplitude = gen_amplitude(rng);
    test_echos.direct_echos.push_back(test_echo);
  }

  publish_echos(test_echos);
  verify_echos(test_echos, get_echos());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
