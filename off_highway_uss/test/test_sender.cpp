// Copyright 2023 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"

#include "gtest/gtest.h"

#include "off_highway_uss/sender.hpp"
#include "off_highway_common/helper.hpp"

using off_highway_common::auto_static_cast;

struct CallbackHelper
{
  void cb_can(const can_msgs::Frame & msg)
  {
    ROS_DEBUG_STREAM(msg);
    sent_message_buffer.push_back(msg);
  }

  std::vector<can_msgs::Frame> sent_message_buffer;
};

inline void test_sender_input(double temperature)
{
  ROS_DEBUG_STREAM("Checking temperature value: " << temperature);
  ros::NodeHandle nh;
  off_highway_uss::Sender sender;
  auto msg_def = sender.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<sensor_msgs::Temperature>("temperature", 10);
  ros::Subscriber sub_can = nh.subscribe("sent_messages", 10, &CallbackHelper::cb_can, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_can.getNumPublishers(), 1U);

  // Publish temperature topic
  sensor_msgs::Temperature temperature_publish;
  temperature_publish.temperature = temperature;
  temperature_publish.header.stamp = ros::Time::now();
  pub.publish(temperature_publish);

  // Spin to receive and process sent_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing topic with test data
  ros::spinOnce();  // Receive temperature in sender node
  ros::spinOnce();  // Publish sent_messages
  ros::WallDuration(0.01).sleep();
  ros::spinOnce();  // Run can callback in test

  static const uint32_t temperature_frame_id = 0x0B500000;
  static const double temperature_min = -40.;
  static const double temperature_max = 87.;

  // Set up expected temperature CAN message
  can_msgs::Frame can_temperature_expected;
  auto_static_cast(can_temperature_expected.id, temperature_frame_id);
  auto_static_cast(msg_def[temperature_frame_id].signals["OutsideTemperature"].value, temperature);
  msg_def[temperature_frame_id].encode(can_temperature_expected.data);

  // If value is in valid range, compare value and ID to expected values
  if (temperature >= temperature_min && temperature <= temperature_max) {
    EXPECT_EQ(cb_helper.sent_message_buffer.size(), 1);

    auto received = cb_helper.sent_message_buffer.front();
    EXPECT_EQ(received.data, can_temperature_expected.data);
    EXPECT_EQ(received.id, temperature_frame_id);
    EXPECT_TRUE(received.is_extended);
    EXPECT_FALSE(received.is_error);
    EXPECT_FALSE(received.is_rtr);
  } else {
    EXPECT_TRUE(cb_helper.sent_message_buffer.empty());
  }
}

TEST(TestUssSender, temperatureValidRange) {
  // Min, max, 0 and value in valid range
  test_sender_input(-40);
  test_sender_input(87);
  test_sender_input(0);
  test_sender_input(20);
}

TEST(TestUssSender, invalidScale) {
  // Since scale of the CAN msg is 1, decimal places shall be cut off
  test_sender_input(20.1);
  test_sender_input(20.5);
  test_sender_input(20.9);
}

TEST(TestUssSender, temperatureOutOfRange) {
  // One value < min and one > max
  test_sender_input(-41);
  test_sender_input(-40.1);
  test_sender_input(87.1);
  test_sender_input(88);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_sender");
  ros::NodeHandle nh_;
  int ret = RUN_ALL_TESTS();
  nh_.shutdown();
  return ret;
}
