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

#include <random>
#include <stdexcept>

#include "ros/ros.h"

#include "gtest/gtest.h"

#include "off_highway_general_purpose_radar/receiver.hpp"
#include "off_highway_common/helper.hpp"

using off_highway_common::auto_static_cast;


struct CallbackHelper
{
  void cb_receiver_info(const off_highway_general_purpose_radar_msgs::Information & msg)
  {
    receiver_info = msg;
    ROS_DEBUG_STREAM(receiver_info);
  }

  void cb_receiver_targets(const off_highway_general_purpose_radar_msgs::Targets & msg)
  {
    receiver_targets = msg;
    ROS_DEBUG_STREAM(receiver_targets);
  }

  off_highway_general_purpose_radar_msgs::Information receiver_info;
  off_highway_general_purpose_radar_msgs::Targets receiver_targets;
};

inline void test_receiver_input_info(
  off_highway_general_purpose_radar::Receiver::Information information)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_general_purpose_radar::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_info = nh.subscribe(
    "test_receiver/info", 10,
    &CallbackHelper::cb_receiver_info, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_info.getNumPublishers(), 1U);

  // Set up info CAN message, cast and encode values
  can_msgs::Frame can_msg_info;

  // Cast and encode info
  auto_static_cast(can_msg_info.id, 0x100);
  auto_static_cast(can_msg_info.header.stamp, ros::Time::now());
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

  // Encode message
  info_msg.encode(can_msg_info.data);

  // Publish received_messages
  pub.publish(can_msg_info);

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing test CAN data
  ros::spinOnce();  // Receive CAN message in receiver node and run process function
  ros::spinOnce();  // Publish topics
  ros::WallDuration(0.01).sleep();
  ros::spinOnce();  // Run CAN callback in test

  // Check info values
  ROS_DEBUG_STREAM("Checking radar receiver info.");
  EXPECT_EQ(cb_helper.receiver_info.sensor_type, information.sensor_type);
  EXPECT_EQ(cb_helper.receiver_info.hw_temperature, information.hw_temperature);
  EXPECT_EQ(cb_helper.receiver_info.sensor_blind, information.sensor_blind);
  EXPECT_EQ(cb_helper.receiver_info.sw_fail, information.sw_fail);
  EXPECT_EQ(cb_helper.receiver_info.hw_fail, information.hw_fail);
  EXPECT_EQ(cb_helper.receiver_info.can_fail, information.can_fail);
  EXPECT_EQ(cb_helper.receiver_info.config_fail, information.config_fail);
  EXPECT_EQ(cb_helper.receiver_info.diag_mode, information.diag_mode);
  EXPECT_EQ(cb_helper.receiver_info.dtc, information.dtc);
  EXPECT_EQ(cb_helper.receiver_info.dtc_order_id, information.dtc_order_id);
  EXPECT_EQ(cb_helper.receiver_info.sensor_not_safe, information.sensor_not_safe);
}

TEST(TestRadarReceiver, testInfoValidValues) {
  // Set all values to 0
  off_highway_general_purpose_radar::Receiver::Information test_information;
  test_information.sensor_type = 0;
  test_information.hw_temperature = 0;
  test_information.sensor_blind = false;
  test_information.sw_fail = false;
  test_information.hw_fail = false;
  test_information.can_fail = false;
  test_information.config_fail = false;
  test_information.diag_mode = false;
  test_information.dtc = 0;
  test_information.dtc_order_id = 0;
  test_information.sensor_not_safe = false;
  test_receiver_input_info(test_information);

  // Set values to min
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
  test_receiver_input_info(test_information);

  // Set values to max
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
  test_receiver_input_info(test_information);

  // Set any valid values
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
  test_receiver_input_info(test_information);
}

inline double sgn(double x)
{
  return (x > 0) - (x < 0);
}

inline void apply_half_increment_offset(double & value, double increment)
{
  value += sgn(value) * increment * 0.5;
}

inline void test_receiver_input_targets(
  off_highway_general_purpose_radar::Receiver::Targets test_targets, bool send_a,
  bool send_b)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_general_purpose_radar::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_targets = nh.subscribe(
    "test_receiver/targets", 40,
    &CallbackHelper::cb_receiver_targets, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_targets.getNumPublishers(), 1U);

  // Set up targets CAN message, cast and encode values
  can_msgs::Frame can_msg_target;

  uint8_t defined_target_ids = 0;
  // Publish targets to received_messages
  for (off_highway_general_purpose_radar::Receiver::Target test_target :
    test_targets.targets)
  {
    if (send_a) {
      auto_static_cast(can_msg_target.id, test_target.a.can_id);
      auto_static_cast(can_msg_target.header.stamp, ros::Time::now());
      off_highway_common::Message & target_a_msg = msg_def[can_msg_target.id];
      auto_static_cast(target_a_msg.signals["AMessage"].value, true);
      auto_static_cast(target_a_msg.signals["ID"].value, test_target.a.id);
      auto_static_cast(target_a_msg.signals["dr"].value, test_target.a.radial_distance);
      auto_static_cast(target_a_msg.signals["vr"].value, test_target.a.radial_velocity);
      auto_static_cast(target_a_msg.signals["dbPower"].value, test_target.a.reflected_power);
      auto_static_cast(target_a_msg.signals["phi"].value, test_target.a.azimuth_angle);
      auto_static_cast(target_a_msg.signals["measured"].value, test_target.a.measured);

      // Encode message
      target_a_msg.encode(can_msg_target.data);

      // Publish received_messages
      pub.publish(can_msg_target);

      // Target only counts if target A was sent and target is valid
      if (test_target.a.measured) {
        defined_target_ids++;
      }
    }

    if (send_b) {
      // Cast target B message
      auto_static_cast(can_msg_target.id, test_target.b.can_id);
      auto_static_cast(can_msg_target.header.stamp, ros::Time::now());
      off_highway_common::Message & target_b_msg = msg_def[can_msg_target.id];
      auto_static_cast(target_b_msg.signals["AMessage"].value, false);
      auto_static_cast(target_b_msg.signals["ID"].value, test_target.b.id);
      auto_static_cast(
        target_b_msg.signals["timeSinceMeas"].value,
        test_target.b.time_since_meas);

      // Apply physical equivalent of half an increment to compensate truncation during encoding
      apply_half_increment_offset(
        test_target.b.azimuth_angle_std,
        target_b_msg.signals["phiSdv"].factor);
      auto_static_cast(target_b_msg.signals["phiSdv"].value, test_target.b.azimuth_angle_std);

      auto_static_cast(
        target_b_msg.signals["vrSdv"].value,
        test_target.b.radial_velocity_std);
      auto_static_cast(
        target_b_msg.signals["drSdv"].value,
        test_target.b.radial_distance_std);
      auto_static_cast(target_b_msg.signals["pExist"].value, test_target.b.exist_probability);

      // Encode message
      target_b_msg.encode(can_msg_target.data);

      // Publish received_messages
      pub.publish(can_msg_target);
    }
  }

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing test CAN data
  ros::spinOnce();  // Publish CAN test msgs, and run callack in receiver node
  ros::WallDuration(0.1).sleep();  // Wait for manage_and_publish callback in receiver node
  ros::spinOnce();  // Run manage_and_publish callback
  ros::WallDuration(0.02).sleep();  // Delay after publishing targets
  ros::spinOnce();  // Run targets callback in test

  // Check targets
  uint8_t found_target_ids = 0;
  for (off_highway_general_purpose_radar_msgs::Target receiver_target :
    cb_helper.receiver_targets.targets)
  {
    // Search received target in test targets
    ROS_DEBUG_STREAM(
      "Searching Target A with ID " << std::to_string(receiver_target.a.id) <<
        " in received test targets.");
    off_highway_general_purpose_radar::Receiver::Target current_test_target;
    for (off_highway_general_purpose_radar::Receiver::Target test_target :
      test_targets.targets)
    {
      if (receiver_target.a.can_id == test_target.a.can_id) {
        current_test_target = test_target;
        break;
      }
    }

    // Target A and B is only sent if target A was sent and target is valid
    if (send_a && receiver_target.a.measured) {
      ROS_DEBUG_STREAM(
        "Checking Target A with ID " << std::to_string(receiver_target.a.id) <<
          " from CAN message with ID " << std::to_string(receiver_target.a.can_id) << ".");
      EXPECT_EQ(receiver_target.a.can_id, current_test_target.a.can_id);
      EXPECT_EQ(receiver_target.a.id, current_test_target.a.id);
      EXPECT_EQ(receiver_target.a.radial_distance, current_test_target.a.radial_distance);
      EXPECT_EQ(receiver_target.a.radial_velocity, current_test_target.a.radial_velocity);
      EXPECT_EQ(receiver_target.a.reflected_power, current_test_target.a.reflected_power);
      EXPECT_NEAR(receiver_target.a.azimuth_angle, current_test_target.a.azimuth_angle, 1e-7);
      EXPECT_EQ(receiver_target.a.measured, current_test_target.a.measured);
      if (send_b) {
        ROS_DEBUG_STREAM(
          "Checking Target B with ID " << std::to_string(receiver_target.b.id) <<
            " from CAN message with ID " << std::to_string(receiver_target.b.can_id) << ".");
        EXPECT_EQ(receiver_target.b.can_id, current_test_target.b.can_id);
        EXPECT_EQ(receiver_target.b.id, current_test_target.b.id);
        EXPECT_NEAR(
          receiver_target.b.time_since_meas,
          current_test_target.b.time_since_meas, 1e-7);
        EXPECT_NEAR(
          receiver_target.b.azimuth_angle_std,
          current_test_target.b.azimuth_angle_std, 1e-4);
        EXPECT_EQ(
          receiver_target.b.radial_velocity_std,
          current_test_target.b.radial_velocity_std);
        EXPECT_EQ(
          receiver_target.b.radial_distance_std,
          current_test_target.b.radial_distance_std);
        EXPECT_EQ(receiver_target.b.exist_probability, current_test_target.b.exist_probability);
      } else {
        EXPECT_EQ(receiver_target.b.can_id, 0);
        EXPECT_EQ(receiver_target.b.id, 0);
        EXPECT_EQ(receiver_target.b.time_since_meas, 0);
        EXPECT_EQ(receiver_target.b.azimuth_angle_std, 0);
        EXPECT_EQ(receiver_target.b.radial_velocity_std, 0);
        EXPECT_EQ(receiver_target.b.radial_distance_std, 0);
        EXPECT_EQ(receiver_target.b.exist_probability, 0);
      }
      found_target_ids++;
    } else {
      // If target A was not sent or target is not valid, no target should be received
      EXPECT_TRUE(cb_helper.receiver_targets.targets.empty());
    }
  }

  ROS_DEBUG_STREAM(
    "Targets defined: " << std::to_string(defined_target_ids) << ", targets found: " <<
      std::to_string(found_target_ids));

  EXPECT_EQ(defined_target_ids, found_target_ids);
}

TEST(TestRadarReceiver, testTarget) {
  off_highway_general_purpose_radar::Receiver::Targets test_targets;
  off_highway_general_purpose_radar::Receiver::Target test_target_0;
  // Set values for target A message
  test_target_0.a.id = 0;
  test_target_0.a.can_id = 0x200;
  test_target_0.a.radial_distance = 0.0;
  test_target_0.a.radial_velocity = 0.0;
  test_target_0.a.reflected_power = 0.0;
  test_target_0.a.azimuth_angle = 0.0;
  test_target_0.a.measured = true;  // Filtered out if false
  // Set values for one target B message
  test_target_0.b.id = 0;
  test_target_0.b.can_id = 0x201;
  test_target_0.b.time_since_meas = 0.0;
  test_target_0.b.azimuth_angle_std = 0.0;
  test_target_0.b.radial_velocity_std = 0.0;
  test_target_0.b.radial_distance_std = 0.0;
  test_target_0.b.exist_probability = 0.0;
  test_targets.targets.push_back(test_target_0);
  test_receiver_input_targets(test_targets, true, true);

  test_targets.targets.clear();
  off_highway_general_purpose_radar::Receiver::Target test_target_rand;
  // Set values for target A message
  test_target_rand.a.id = 19;
  test_target_rand.a.can_id = 0x226;
  test_target_rand.a.radial_distance = 200.0;
  test_target_rand.a.radial_velocity = 100.0;
  test_target_rand.a.reflected_power = 50.0;
  test_target_rand.a.azimuth_angle = -0.5;
  test_target_rand.a.measured = true;
  // Set values for one target B message
  test_target_rand.b.id = 19;
  test_target_rand.b.can_id = 0x227;
  test_target_rand.b.time_since_meas = 0.1;
  test_target_rand.b.azimuth_angle_std = 0.05;
  test_target_rand.b.radial_velocity_std = 2.5;
  test_target_rand.b.radial_distance_std = 2.5;
  test_target_rand.b.exist_probability = 0.5;
  test_targets.targets.push_back(test_target_rand);
  test_receiver_input_targets(test_targets, true, true);

  test_targets.targets.clear();
  off_highway_general_purpose_radar::Receiver::Target test_target_min;
  // Set values for target A message
  test_target_min.a.id = 0;
  test_target_min.a.can_id = 0x200;
  test_target_min.a.radial_distance = 0.0;
  test_target_min.a.radial_velocity = -128.0;
  test_target_min.a.reflected_power = -64.0;
  test_target_min.a.azimuth_angle = -1.6384;
  test_target_min.a.measured = true;
  // Set values for one target B message
  test_target_min.b.id = 0;
  test_target_min.b.can_id = 0x201;
  test_target_min.b.time_since_meas = 0;
  test_target_min.b.azimuth_angle_std = 0.0;
  test_target_min.b.radial_velocity_std = 0.0;
  test_target_min.b.radial_distance_std = 0.0;
  test_target_min.b.exist_probability = 0.0;
  test_targets.targets.push_back(test_target_min);
  test_receiver_input_targets(test_targets, true, true);

  test_targets.targets.clear();
  off_highway_general_purpose_radar::Receiver::Target test_target_max;
  // Set values for target A message
  test_target_max.a.id = 47;
  test_target_max.a.can_id = 0x25E;
  test_target_max.a.radial_distance = 255.9375;
  test_target_max.a.radial_velocity = 127.9375;
  test_target_max.a.reflected_power = 63.5;
  test_target_max.a.azimuth_angle = 1.6382;
  test_target_max.a.measured = true;
  // Set values for one target B message
  test_target_max.b.id = 47;
  test_target_max.b.can_id = 0x25F;
  test_target_max.b.time_since_meas = 0.8191;
  test_target_max.b.azimuth_angle_std = 0.063;
  test_target_max.b.radial_velocity_std = 3.9375;
  test_target_max.b.radial_distance_std = 3.9375;
  test_target_max.b.exist_probability = 0.96875;
  test_targets.targets.push_back(test_target_max);
  test_receiver_input_targets(test_targets, true, true);
}

TEST(TestRadarReceiver, testTargetIncomplete) {
  off_highway_general_purpose_radar::Receiver::Targets test_targets;
  off_highway_general_purpose_radar::Receiver::Target test_target;
  // Set values for target A message
  test_target.a.id = 19;
  test_target.a.can_id = 0x226;
  test_target.a.radial_distance = 200.0;
  test_target.a.radial_velocity = 100.0;
  test_target.a.reflected_power = 50.0;
  test_target.a.azimuth_angle = -0.5;
  test_target.a.measured = true;
  // Set values for one target B message
  test_target.b.id = 19;
  test_target.b.can_id = 0x201;
  test_target.b.time_since_meas = 0.1;
  test_target.b.azimuth_angle_std = 0.05;
  test_target.b.radial_velocity_std = 2.5;
  test_target.b.radial_distance_std = 2.5;
  test_target.b.exist_probability = 0.5;
  test_targets.targets.push_back(test_target);
  test_receiver_input_targets(test_targets, true, false);
  test_receiver_input_targets(test_targets, false, true);

  test_targets.targets.clear();
  off_highway_general_purpose_radar::Receiver::Target test_target_invalid;
  // Set values for target A message
  test_target_invalid.a.id = 39;
  test_target_invalid.a.can_id = 0x24E;
  test_target_invalid.a.radial_distance = 255.9375;
  test_target_invalid.a.radial_velocity = 127.9375;
  test_target_invalid.a.reflected_power = 63.5;
  test_target_invalid.a.azimuth_angle = 1.6382;
  test_target_invalid.a.measured = false;
  // Set values for one target B message
  test_target_invalid.b.id = 39;
  test_target_invalid.b.can_id = 0x24F;
  test_target_invalid.b.time_since_meas = 0.8191;
  test_target_invalid.b.azimuth_angle_std = 0.063;
  test_target_invalid.b.radial_velocity_std = 3.9375;
  test_target_invalid.b.radial_distance_std = 3.9375;
  test_target_invalid.b.exist_probability = 0.96875;
  test_targets.targets.push_back(test_target_invalid);
  test_receiver_input_targets(test_targets, true, true);
}

class RandomQuantizedGenerator
{
public:
  RandomQuantizedGenerator(double resolution, double min, double max)
  : resolution{resolution}
  {
    int64_t min_as_int = min / resolution;
    int64_t max_as_int = max / resolution;
    uniform_distribution = std::uniform_int_distribution<int64_t>{min_as_int, max_as_int};
  }

  template<class T>
  double operator()(T & rng)
  {
    return uniform_distribution(rng) * resolution;
  }

private:
  std::uniform_int_distribution<int64_t> uniform_distribution;
  double resolution;
};

TEST(TestRadarReceiver, test48RandomValidTargets) {
  off_highway_general_purpose_radar::Receiver::Targets test_targets;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(48);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_general_purpose_radar::Receiver::Target test_target;
    // Set values for target A message
    test_target.a.id = id;
    test_target.a.can_id = 0x200 + test_target.a.id * 2;
    test_target.a.radial_distance = RandomQuantizedGenerator{0.0625, 0, 255.9375}(rng);
    test_target.a.radial_velocity = RandomQuantizedGenerator{0.0625, -128, 127.9375}(rng);
    test_target.a.reflected_power = RandomQuantizedGenerator{0.5, -64, 63.5}(rng);
    test_target.a.azimuth_angle = RandomQuantizedGenerator{0.0002, -1.6384, 1.6382}(rng);
    test_target.a.measured = true;

    // Set values for one target B message
    test_target.b.id = test_target.a.id;
    test_target.b.can_id = test_target.a.can_id + 1;
    test_target.b.time_since_meas = RandomQuantizedGenerator{0.0001, 0, 0.8191}(rng);
    test_target.b.azimuth_angle_std = RandomQuantizedGenerator{0.001, 0, 0.063}(rng);
    test_target.b.radial_velocity_std = RandomQuantizedGenerator{0.0625, 0, 3.9375}(rng);
    test_target.b.radial_distance_std = RandomQuantizedGenerator{0.0625, 0, 3.9375}(rng);
    test_target.b.exist_probability = RandomQuantizedGenerator{0.03125, 0, 0.96875}(rng);

    ROS_DEBUG_STREAM(
      "Target ID " << std::to_string(test_target.a.id) <<
        " with CAN ID " << std::to_string(test_target.a.can_id) << " generated.");

    test_targets.targets.push_back(test_target);
  }

  test_receiver_input_targets(test_targets, true, true);
}

TEST(TestRadarReceiver, testInvalidTargetIdA) {
  off_highway_general_purpose_radar::Receiver::Targets test_targets;
  off_highway_general_purpose_radar::Receiver::Target test_target;

  // Set values for object A message
  test_target.a.id = 48;
  test_target.a.can_id = 0x200;

  test_targets.targets.push_back(test_target);

  EXPECT_THROW(test_receiver_input_targets(test_targets, true, true), std::runtime_error);
}

TEST(TestRadarReceiver, testInvalidTargetIdB) {
  off_highway_general_purpose_radar::Receiver::Targets test_targets;
  off_highway_general_purpose_radar::Receiver::Target test_target;

  // Set values for object B message
  test_target.b.id = 48;
  test_target.b.can_id = 0x201;

  test_targets.targets.push_back(test_target);

  EXPECT_THROW(test_receiver_input_targets(test_targets, true, true), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_receiver");
  ros::NodeHandle nh_;
  int ret = RUN_ALL_TESTS();
  nh_.shutdown();
  return ret;
}
