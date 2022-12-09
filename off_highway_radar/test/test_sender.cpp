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

#include "off_highway_radar/sender.hpp"
#include "off_highway_common/helper.hpp"

using off_highway_common::auto_static_cast;

static constexpr double kRadToDegree = 180. / M_PI;

struct CallbackHelper
{
  void cb_can(const can_msgs::Frame & msg)
  {
    ROS_DEBUG_STREAM(msg);
    sent_message_buffer.push_back(msg);
  }

  std::vector<can_msgs::Frame> sent_message_buffer;
};

inline void test_sender_input(double velocity, double yaw)
{
  ROS_DEBUG_STREAM("Checking velocity value: " << velocity);
  ROS_DEBUG_STREAM("Checking yaw value: " << yaw * kRadToDegree);

  ros::NodeHandle nh;
  off_highway_radar::Sender sender;
  auto msg_def = sender.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
  ros::Subscriber sub_can = nh.subscribe("sent_messages", 10, &CallbackHelper::cb_can, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_can.getNumPublishers(), 1U);

  // Publish velocity topic
  geometry_msgs::TwistStamped radar_publish;
  radar_publish.twist.linear.x = velocity;
  radar_publish.twist.angular.z = yaw;
  radar_publish.header.stamp = ros::Time::now();
  pub.publish(radar_publish);

  // Spin to receive and process sent_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing topic with test data
  ros::spinOnce();  // Receive velocity in sender node and run process function
  ros::spinOnce();  // Publish sent_messages
  ros::WallDuration(0.01).sleep();
  ros::spinOnce();  // Run CAN callback in test

  static const uint32_t velocity_id = 0x050;
  static const double velocity_min = -81.92;
  static const double velocity_max = 81.9175;

  static const uint32_t yaw_id = 0x174;
  static const double yaw_min = -163.84 / kRadToDegree;
  static const double yaw_max = 163.83 / kRadToDegree;

  // Set up expected velocity CAN message
  can_msgs::Frame can_velocity_expected;
  auto_static_cast(can_velocity_expected.id, velocity_id);
  auto_static_cast(msg_def[velocity_id].signals["v"].value, velocity);
  msg_def[velocity_id].encode(can_velocity_expected.data);

  // Set up expected yaw CAN message
  can_msgs::Frame can_yaw_expected;
  auto_static_cast(can_yaw_expected.id, yaw_id);
  auto_static_cast(msg_def[yaw_id].signals["psidt"].value, yaw * kRadToDegree);
  msg_def[yaw_id].encode(can_yaw_expected.data);

  // If velocity or yaw is out of range, nothing should have been received
  if ((velocity < velocity_min || velocity > velocity_max) || (yaw < yaw_min || yaw > yaw_max)) {
    EXPECT_TRUE(cb_helper.sent_message_buffer.empty());
    return;
  }

  EXPECT_EQ(cb_helper.sent_message_buffer.size(), 2);

  // Check velocity and yaw messages
  bool velocity_received = false;
  bool yaw_received = false;
  for (can_msgs::Frame can_msg : cb_helper.sent_message_buffer) {
    switch (can_msg.id) {
      case velocity_id:
        EXPECT_EQ(can_msg.data, can_velocity_expected.data);
        velocity_received = true;
        break;
      case yaw_id:
        EXPECT_EQ(can_msg.data, can_yaw_expected.data);
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

TEST(TestRadarSender, validValues) {
  test_sender_input(0, 0);
  test_sender_input(-10, -1);
  test_sender_input(10, 1);
  test_sender_input(-10.1, -1.1);
  test_sender_input(10.1, 1.1);
  test_sender_input(-81.92, -163.84 / kRadToDegree);
  test_sender_input(81.9175, 163.83 / kRadToDegree);
}

TEST(TestRadarSender, invalidScale) {
  test_sender_input(10.0036, 10.003 / kRadToDegree);
  test_sender_input(11.1111, 33.3333 / kRadToDegree);
  test_sender_input(33.3333, 77.7777 / kRadToDegree);
}

TEST(TestRadarSender, outOfRange) {
  test_sender_input(-81.921, -163.845 / kRadToDegree);
  test_sender_input(81.91751, 163.835 / kRadToDegree);
  test_sender_input(-83, 0);
  test_sender_input(83, 0);
  test_sender_input(0, -164 / kRadToDegree);
  test_sender_input(0, 164 / kRadToDegree);
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
