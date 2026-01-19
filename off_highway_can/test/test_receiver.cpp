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

#include <chrono>
#include <cmath>
#include <thread>

#include "gtest/gtest.h"

#include "can_msgs/msg/frame.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "helpers/random_generator.hpp"
#include "off_highway_can/can_message.hpp"
#include "off_highway_can/receiver.hpp"
#include "off_highway_can/sender.hpp"
#include "rclcpp/rclcpp.hpp"

// Timeout threshold - triggers forced diagnostic when no messages received for this duration
static constexpr double kTestTimeout = 0.2;
// Watchdog check frequency
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
    test_message.crc_index = std::nullopt;
    messages[kTestCanId] = test_message;
    return messages;
  }

  void process(std_msgs::msg::Header header, const FrameId & id, off_highway_can::Message & message)
  override
  {
    (void)header;
    (void)id;
    (void)message;
    processed_count_++;
  }

  int get_processed_count() const {return processed_count_;}

private:
  std::atomic<int> processed_count_{0};
};

class DummySender : public rclcpp::Node
{
public:
  explicit DummySender(const rclcpp::NodeOptions & options)
  : rclcpp::Node("dummy_sender", options)
  {
    // Publish to the topic the receiver subscribes to
    can_pub_ = create_publisher<can_msgs::msg::Frame>(
      "from_can_bus",
      rclcpp::SystemDefaultsQoS());
  }

  void send_can()
  {
    can_msgs::msg::Frame frame;
    frame.id = kTestCanId;
    frame.data = {0, 0, 0, 0, 0, 0, 0, 0};
    frame.dlc = 8;
    frame.header.stamp = now();
    can_pub_->publish(frame);
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
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

        // Track timeout state transitions
        for (const auto & status : msg.status) {
          if (status.name == "test_receiver: receiver") {
            message_count_++;
            for (const auto & kv : status.values) {
              if (kv.key == "Timeout") {
                if (kv.value == "False") {
                  // Messages are being received - mark that we've seen this state
                  received_messages_.store(true);
                } else if (kv.value == "True") {
                  // Timeout occurred - set promise if:
                  // 1. We've seen False first (for tests with messages), OR
                  // 2. No messages expected (zero-message test)
                  if (received_messages_.load() || expected_messages_number_.load() == 0) {
                    promise_.set_value(true);
                  }
                }
              }
            }
            break;
          }
        }
      });

    executor_.add_node(test_receiver_);
    executor_.add_node(sender_node_);
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    executor_.remove_node(test_receiver_);
    executor_.remove_node(sender_node_);
    executor_.remove_node(node_);
    diag_subscription_.reset();
    test_receiver_.reset();
    sender_node_.reset();
    node_.reset();
    received_buffer_.clear();
    message_count_.store(0);
    received_messages_.store(false);
    expected_messages_number_.store(0);
    promise_ = std::promise<bool>();
    future_ = promise_.get_future();
  }

  void spinNodes(int sent_msg_number);

  std::shared_ptr<TestReceiver> test_receiver_;
  std::shared_ptr<DummySender> sender_node_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_subscription_;

  std::vector<diagnostic_msgs::msg::DiagnosticArray> received_buffer_;

  std::atomic<int> message_count_{0};
  std::atomic<bool> received_messages_{false};
  std::atomic<int> expected_messages_number_{0};
  std::promise<bool> promise_;
  std::shared_future<bool> future_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

void TestClass::spinNodes(int sent_msg_number)
{
  // Set expected message count for this test run
  expected_messages_number_.store(sent_msg_number);

  // Start executor in background thread to process messages continuously
  std::atomic<bool> executor_running{true};
  std::thread executor_thread([&]() {
      while (executor_running) {
        executor_.spin_some();
      }
    });

  // Wait for subscriptions to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Send messages while executor is running in parallel
  for (int i = 0; i < sent_msg_number; ++i) {
    sender_node_->send_can();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // Wait for timeout to occur after all messages have been sent
  future_.wait_for(std::chrono::seconds(20));

  // Stop executor thread
  executor_running = false;
  executor_thread.join();

  // Verify diagnostic content and count
  int timeout_true_count = 0;
  int timeout_false_count = 0;
  int receiver_diag_count = 0;

  for (const auto & diag_array : received_buffer_) {
    for (const auto & status : diag_array.status) {
      if (status.name == "test_receiver: receiver") {
        receiver_diag_count++;
        for (const auto & kv : status.values) {
          if (kv.key == "Timeout") {
            if (kv.value == "True") {
              timeout_true_count++;
            } else if (kv.value == "False") {
              timeout_false_count++;
            }
          }
        }
      }
    }
  }

  int processed_count = test_receiver_->get_processed_count();

  // Verify all sent messages were processed
  EXPECT_EQ(processed_count, sent_msg_number)
    << "All sent messages should be processed by the receiver";

  // Verify total diagnostics equals the sum of timeout states
  EXPECT_EQ(receiver_diag_count, timeout_true_count + timeout_false_count)
    << "All diagnostics should have a Timeout value (True or False)";

  // For tests that send messages, verify the state transition: False -> True
  if (sent_msg_number > 0) {
    // Should see at least one Timeout=False while processing messages
    EXPECT_GT(timeout_false_count, 0)
      << "Expected at least one Timeout=False diagnostic while processing messages";

    // Should see Timeout=True after messages stop
    EXPECT_GT(timeout_true_count, 0)
      << "Expected at least one Timeout=True diagnostic after timeout";

    // Verify we saw the correct state transition (False before True)
    EXPECT_TRUE(received_messages_.load())
      << "Expected to see Timeout=False before Timeout=True (state transition)";

  } else {
    // For zero message tests, should only see Timeout=True (always in timeout)
    EXPECT_EQ(timeout_false_count, 0)
      << "Expected no Timeout=False diagnostics when no messages sent";

    EXPECT_EQ(timeout_true_count, receiver_diag_count)
      << "All diagnostics should be Timeout=True when no messages sent";
  }
}

TEST_F(TestClass, testMultipleCanMessages)
{
  int messages_to_send = 200;
  spinNodes(messages_to_send);
}

TEST_F(TestClass, testZeroCanMessages)
{
  int messages_to_send = 0;
  spinNodes(messages_to_send);
}

TEST_F(TestClass, testRandomNumberOfCanMessages)
{
  int messages_to_send = RandomQuantizedGenerator{1.0, 0, 100}();
  spinNodes(messages_to_send);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
