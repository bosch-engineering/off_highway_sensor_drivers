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

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "helpers/random_generator.hpp"
#include "off_highway_can/can_message.hpp"
#include "off_highway_can/receiver.hpp"
#include "off_highway_can/sender.hpp"
#include "rclcpp/rclcpp.hpp"

// Test message timeout
static constexpr double kTestTimeout = 0.2;
// Test watchdog frequency
static constexpr double kTestWatchdogFrequency = 10.0;
// CAN ID for test message
static constexpr uint32_t kTestCanId = 0x123;

class TestReceiver : public off_highway_can::Receiver
{
public:
  explicit TestReceiver(const rclcpp::NodeOptions & options)
  : off_highway_can::Receiver("test_receiver", options, false)
  {
    Receiver::start();
  }

  Messages fillMessageDefinitions() override
  {
    Messages messages;
    off_highway_can::Message test_message;
    test_message.name = "TestMessage";
    test_message.signals["DummyData"] = off_highway_can::Signal{0, 8, false, true, 1, 0};
    messages[kTestCanId] = test_message;
    return messages;
  }

  void process(std_msgs::msg::Header header, const FrameId & id, off_highway_can::Message & message)
  override
  {
    (void)header;
    (void)id;
    (void)message;
  }
};

class DummySender : public off_highway_can::Sender
{
public:
  explicit DummySender(const rclcpp::NodeOptions & options)
  : off_highway_can::Sender("dummy_sender", options, false)
  {
    off_highway_can::Message test_message;
    test_message.name = "TestMessage";
    test_message.signals["DummyData"] = off_highway_can::Signal{0, 8, false, true, 1, 0};
    messages_[kTestCanId] = test_message;
  }

  void send_can()
  {
    auto & dummy_data = messages_[kTestCanId].signals["DummyData"];
    if (!dummy_data.set(0, "DummyData")) {
      return;
    }
    off_highway_can::Sender::send_can();
  }
};

class TestClass : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter("timeout", kTestTimeout),
      rclcpp::Parameter("watchdog_frequency", kTestWatchdogFrequency),
    });
    test_receiver_ = std::make_shared<TestReceiver>(options);
    sender_node_ = std::make_shared<DummySender>(options);
    node_ = std::make_shared<rclcpp::Node>("test_node");

    future_ = promise_.get_future();
    diag_subscription_ =
      node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 100,
      [&](const diagnostic_msgs::msg::DiagnosticArray msg)
      {
        received_buffer_.push_back(msg);
        message_count_++;
        if (message_count_.load() >= expected_messages_number_) {
          promise_.set_value(true);
        }
      });

    executor_.add_node(test_receiver_);
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    // Clean up executor
    executor_.remove_node(test_receiver_);
    executor_.remove_node(node_);

    // Reset shared pointers explicitly (optional, but clear intent)
    diag_subscription_.reset();
    test_receiver_.reset();
    sender_node_.reset();
    node_.reset();

    // Clear buffer
    received_buffer_.clear();

    // Reset atomic counter
    message_count_.store(0);

    // Reset promise/future for next test
    promise_ = std::promise<bool>();
    future_ = promise_.get_future();
  }

  void spinNodes(int expected_msg_number, int sent_msg_number);
  void verifyValues(int expected_msg_number, int sent_msg_number);

  std::shared_ptr<TestReceiver> test_receiver_;
  std::shared_ptr<DummySender> sender_node_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_subscription_;

  std::vector<diagnostic_msgs::msg::DiagnosticArray> received_buffer_;

  std::atomic<int> message_count_{0};
  int expected_messages_number_{5};
  std::promise<bool> promise_;
  std::shared_future<bool> future_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void TestClass::spinNodes(int expected_msg_number, int sent_msg_number)
{
  expected_messages_number_ = expected_msg_number;

  if (sent_msg_number > 0) {
    for (int i = 0; i < sent_msg_number; ++i) {
      sender_node_->send_can();
    }
  }

  executor_.spin_until_future_complete(future_, std::chrono::seconds(20));
}

void TestClass::verifyValues(int expected_msg_number, int sent_msg_number)
{
  EXPECT_EQ(expected_msg_number, static_cast<int>(received_buffer_.size()));

  int true_values = 0;
  int false_values = 0;
  for (const auto & msg : received_buffer_) {
    if (std::string("True") == msg.status[0].values[0].value) {
      true_values++;
    } else {
      false_values++;
    }
  }

  EXPECT_EQ(sent_msg_number, false_values);
  EXPECT_EQ(static_cast<int>(received_buffer_.size() - sent_msg_number), true_values);
}

TEST_F(TestClass, testFullCanMessages)
{
  int messages_to_send = 20;

  spinNodes(messages_to_send, messages_to_send);
  verifyValues(messages_to_send, messages_to_send);
}

TEST_F(TestClass, testZeroCanMessages)
{
  int messages_to_expect = 20;
  int messages_to_send = 0;

  spinNodes(messages_to_expect, messages_to_send);
  verifyValues(messages_to_expect, messages_to_send);
}

TEST_F(TestClass, testPartialBuffer)
{
  int messages_to_expect = 20;
  int messages_to_send = 5;

  spinNodes(messages_to_expect, messages_to_send);
  verifyValues(messages_to_expect, messages_to_send);
}

TEST_F(TestClass, testRandomNumberOfMessages)
{
  int messages_to_expect = RandomQuantizedGenerator{1.0, 0, 100}();
  int messages_to_send = RandomQuantizedGenerator{1.0, 0,
    static_cast<double>(messages_to_expect)}();

  spinNodes(messages_to_expect, messages_to_send);
  verifyValues(messages_to_expect, messages_to_send);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
