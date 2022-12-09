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

#include "ros/ros.h"

#include "gtest/gtest.h"

#include "off_highway_radar/receiver.hpp"
#include "off_highway_common/helper.hpp"

using off_highway_common::auto_static_cast;


struct CallbackHelper
{
  void cb_receiver_info(const off_highway_radar_msgs::Information & msg)
  {
    receiver_info = msg;
    ROS_DEBUG_STREAM(receiver_info);
  }

  void cb_receiver_objects(const off_highway_radar_msgs::Objects & msg)
  {
    receiver_objects = msg;
    ROS_DEBUG_STREAM(receiver_objects);
  }

  off_highway_radar_msgs::Information receiver_info;
  off_highway_radar_msgs::Objects receiver_objects;
};

inline void test_receiver_input_info(off_highway_radar::Receiver::Information information)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_radar::Receiver receiver;
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
  auto_static_cast(info_msg.signals["ReceptionError"].value, information.reception_error);

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
  EXPECT_EQ(cb_helper.receiver_info.reception_error, information.reception_error);
}

TEST(TestRadarReceiver, testInfoValidValues) {
  // Set all values to 0
  off_highway_radar::Receiver::Information test_information;
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
  test_information.reception_error = false;
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
  test_information.reception_error = false;
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
  test_information.reception_error = true;
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
  test_information.reception_error = true;
  test_receiver_input_info(test_information);
}

inline void test_receiver_input_objects(
  off_highway_radar::Receiver::Objects test_objects, bool send_a,
  bool send_b)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_radar::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_objects = nh.subscribe(
    "test_receiver/objects", 40,
    &CallbackHelper::cb_receiver_objects, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_objects.getNumPublishers(), 1U);

  // Set up objects CAN message, cast and encode values
  can_msgs::Frame can_msg_object;

  uint8_t defined_object_ids = 0;
  // Publish objects to received_messages
  for (off_highway_radar::Receiver::Object test_object : test_objects.objects) {
    if (send_a) {
      auto_static_cast(can_msg_object.id, test_object.a.can_id);
      auto_static_cast(can_msg_object.header.stamp, ros::Time::now());
      off_highway_common::Message & object_a_msg = msg_def[can_msg_object.id];
      auto_static_cast(object_a_msg.signals["ID"].value, test_object.a.id);
      auto_static_cast(object_a_msg.signals["dx"].value, test_object.a.position.x);
      auto_static_cast(object_a_msg.signals["dy"].value, test_object.a.position.y);
      auto_static_cast(object_a_msg.signals["vx"].value, test_object.a.velocity.linear.x);
      auto_static_cast(object_a_msg.signals["vy"].value, test_object.a.velocity.linear.y);
      auto_static_cast(object_a_msg.signals["Meas"].value, test_object.a.meas);
      auto_static_cast(object_a_msg.signals["Valid"].value, test_object.a.valid);
      auto_static_cast(object_a_msg.signals["Hist"].value, test_object.a.hist);

      // Encode message
      object_a_msg.encode(can_msg_object.data);

      // Publish received_messages
      pub.publish(can_msg_object);

      // Object only counts if object A was sent and object is valid
      if (test_object.a.valid) {
        defined_object_ids++;
      }
    }

    if (send_b) {
      // Cast object B message
      auto_static_cast(can_msg_object.id, test_object.b.can_id);
      auto_static_cast(can_msg_object.header.stamp, ros::Time::now());
      off_highway_common::Message & object_b_msg = msg_def[can_msg_object.id];
      auto_static_cast(object_b_msg.signals["ID"].value, test_object.b.id);
      auto_static_cast(object_b_msg.signals["timeSinceMeas"].value, test_object.b.time_since_meas);
      auto_static_cast(object_b_msg.signals["Zone"].value, test_object.b.zone);
      auto_static_cast(object_b_msg.signals["RCS"].value, test_object.b.rcs);
      auto_static_cast(object_b_msg.signals["Moving"].value, test_object.b.moving);
      auto_static_cast(object_b_msg.signals["Near"].value, test_object.b.near);
      auto_static_cast(object_b_msg.signals["wExist"].value, test_object.b.exist_probability);

      // Encode message
      object_b_msg.encode(can_msg_object.data);

      // Publish received_messages
      pub.publish(can_msg_object);
    }
  }

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing test CAN data
  ros::spinOnce();  // Publish CAN test msgs, and run callack in receiver node
  ros::WallDuration(0.1).sleep();  // Wait for manage_and_publish callback in receiver node
  ros::spinOnce();  // Run manage_and_publish callback
  ros::WallDuration(0.02).sleep();  // Delay after publishing objects
  ros::spinOnce();  // Run objects callback in test

  // Check objects
  uint8_t found_object_ids = 0;
  for (off_highway_radar_msgs::Object receiver_object : cb_helper.receiver_objects.objects) {
    // Search received object in test objects
    ROS_DEBUG_STREAM(
      "Searching Object A with ID " << std::to_string(receiver_object.a.id) <<
        " in received test objects.");
    off_highway_radar::Receiver::Object current_test_object;
    for (off_highway_radar::Receiver::Object test_object : test_objects.objects) {
      if (receiver_object.a.can_id == test_object.a.can_id) {
        current_test_object = test_object;
        break;
      }
    }

    // Object A and B is only sent if object A was sent and object is valid
    if (send_a && receiver_object.a.valid) {
      ROS_DEBUG_STREAM(
        "Checking Object A with ID " << std::to_string(receiver_object.a.id) <<
          " from CAN message with ID " << std::to_string(receiver_object.a.can_id) << ".");
      EXPECT_EQ(receiver_object.a.can_id, current_test_object.a.can_id);
      EXPECT_EQ(receiver_object.a.id, current_test_object.a.id);
      EXPECT_EQ(receiver_object.a.position.x, current_test_object.a.position.x);
      EXPECT_EQ(receiver_object.a.position.y, current_test_object.a.position.y);
      EXPECT_EQ(receiver_object.a.velocity.linear.x, current_test_object.a.velocity.linear.x);
      EXPECT_EQ(receiver_object.a.velocity.linear.y, current_test_object.a.velocity.linear.y);
      EXPECT_EQ(receiver_object.a.meas, current_test_object.a.meas);
      EXPECT_EQ(receiver_object.a.valid, current_test_object.a.valid);
      EXPECT_EQ(receiver_object.a.hist, current_test_object.a.hist);
      if (send_b) {
        ROS_DEBUG_STREAM(
          "Checking Object B with ID " << std::to_string(receiver_object.b.id) <<
            " from CAN message with ID " << std::to_string(receiver_object.b.can_id) << ".");
        EXPECT_EQ(receiver_object.b.can_id, current_test_object.b.can_id);
        EXPECT_EQ(receiver_object.b.id, current_test_object.b.id);
        EXPECT_NEAR(receiver_object.b.time_since_meas, current_test_object.b.time_since_meas, 1e-6);
        EXPECT_EQ(receiver_object.b.zone, current_test_object.b.zone);
        EXPECT_EQ(receiver_object.b.rcs, current_test_object.b.rcs);
        EXPECT_EQ(receiver_object.b.moving, current_test_object.b.moving);
        EXPECT_EQ(receiver_object.b.near, current_test_object.b.near);
        EXPECT_EQ(receiver_object.b.exist_probability, current_test_object.b.exist_probability);
      } else {
        EXPECT_EQ(receiver_object.b.can_id, 0);
        EXPECT_EQ(receiver_object.b.id, 0);
        EXPECT_EQ(receiver_object.b.time_since_meas, 0);
        EXPECT_EQ(receiver_object.b.zone, 0);
        EXPECT_EQ(receiver_object.b.rcs, 0);
        EXPECT_EQ(receiver_object.b.moving, 0);
        EXPECT_EQ(receiver_object.b.near, 0);
        EXPECT_EQ(receiver_object.b.exist_probability, 0);
      }
      found_object_ids++;
    } else {
      // If object A was not sent or object is not valid, no object should be received
      EXPECT_TRUE(cb_helper.receiver_objects.objects.empty());
    }
  }

  ROS_DEBUG_STREAM(
    "Objects defined: " << std::to_string(defined_object_ids) << ", objects found: " <<
      std::to_string(found_object_ids));

  EXPECT_EQ(defined_object_ids, found_object_ids);
}

TEST(TestRadarReceiver, testObject) {
  off_highway_radar::Receiver::Objects test_objects;
  off_highway_radar::Receiver::Object test_object_0;
  // Set values for object A message
  test_object_0.a.id = 0;
  test_object_0.a.can_id = 0x200;
  test_object_0.a.position.x = 0;
  test_object_0.a.position.y = 0;
  test_object_0.a.velocity.linear.x = 0;
  test_object_0.a.velocity.linear.y = 0;
  test_object_0.a.meas = false;
  test_object_0.a.valid = true;  // Filtered out if false
  test_object_0.a.hist = false;
  // Set values for one object B message
  test_object_0.b.id = 0;
  test_object_0.b.can_id = 0x201;
  test_object_0.b.time_since_meas = 0.0;
  test_object_0.b.zone = 0;
  test_object_0.b.rcs = 0;
  test_object_0.b.moving = false;
  test_object_0.b.near = false;
  test_object_0.b.exist_probability = 0;
  test_objects.objects.push_back(test_object_0);
  test_receiver_input_objects(test_objects, true, true);

  test_objects.objects.clear();
  off_highway_radar::Receiver::Object test_object_rand;
  // Set values for object A message
  test_object_rand.a.id = 19;
  test_object_rand.a.can_id = 0x226;
  test_object_rand.a.position.x = 200;
  test_object_rand.a.position.y = 100;
  test_object_rand.a.velocity.linear.x = 50;
  test_object_rand.a.velocity.linear.y = -50;
  test_object_rand.a.meas = true;
  test_object_rand.a.valid = true;
  test_object_rand.a.hist = true;
  // Set values for one object B message
  test_object_rand.b.id = 19;
  test_object_rand.b.can_id = 0x227;
  test_object_rand.b.time_since_meas = 0.1;
  test_object_rand.b.zone = 10;
  test_object_rand.b.rcs = 30.5;
  test_object_rand.b.moving = true;
  test_object_rand.b.near = false;
  test_object_rand.b.exist_probability = 0.5;
  test_objects.objects.push_back(test_object_rand);
  test_receiver_input_objects(test_objects, true, true);

  test_objects.objects.clear();
  off_highway_radar::Receiver::Object test_object_min;
  // Set values for object A message
  test_object_min.a.id = 0;
  test_object_min.a.can_id = 0x200;
  test_object_min.a.position.x = -256;
  test_object_min.a.position.y = -128;
  test_object_min.a.velocity.linear.x = -64;
  test_object_min.a.velocity.linear.y = -64;
  test_object_min.a.meas = false;
  test_object_min.a.valid = true;
  test_object_min.a.hist = false;
  // Set values for one object B message
  test_object_min.b.id = 0;
  test_object_min.b.can_id = 0x227;
  test_object_min.b.time_since_meas = 0;
  test_object_min.b.zone = 0;
  test_object_min.b.rcs = -64;
  test_object_min.b.moving = false;
  test_object_min.b.near = false;
  test_object_min.b.exist_probability = 0.0;
  test_objects.objects.push_back(test_object_min);
  test_receiver_input_objects(test_objects, true, true);

  test_objects.objects.clear();
  off_highway_radar::Receiver::Object test_object_max;
  // Set values for object A message
  test_object_max.a.id = 39;
  test_object_max.a.can_id = 0x24E;
  test_object_max.a.position.x = 255.9375;
  test_object_max.a.position.y = 127.9375;
  test_object_max.a.velocity.linear.x = 63.875;
  test_object_max.a.velocity.linear.y = 63.875;
  test_object_max.a.meas = true;
  test_object_max.a.valid = true;
  test_object_max.a.hist = true;
  // Set values for one object B message
  test_object_max.b.id = 39;
  test_object_max.b.can_id = 0x24F;
  test_object_max.b.time_since_meas = 0.8191;
  test_object_max.b.zone = 24;
  test_object_max.b.rcs = 63.5;
  test_object_max.b.moving = true;
  test_object_max.b.near = true;
  test_object_max.b.exist_probability = 0.96875;
  test_objects.objects.push_back(test_object_max);
  test_receiver_input_objects(test_objects, true, true);
}

TEST(TestRadarReceiver, testObjectIncomplete) {
  off_highway_radar::Receiver::Objects test_objects;
  off_highway_radar::Receiver::Object test_object;
  // Set values for object A message
  test_object.a.id = 25;
  test_object.a.can_id = 0x232;
  test_object.a.position.x = 123;
  test_object.a.position.y = 45;
  test_object.a.velocity.linear.x = 12;
  test_object.a.velocity.linear.y = 23;
  test_object.a.meas = true;
  test_object.a.valid = true;
  test_object.a.hist = true;
  // Set values for one object B message
  test_object.b.id = 25;
  test_object.b.can_id = 0x233;
  test_object.b.time_since_meas = 0.1;
  test_object.b.zone = 12;
  test_object.b.rcs = 34;
  test_object.b.moving = true;
  test_object.b.near = true;
  test_object.b.exist_probability = 0.23;
  test_objects.objects.push_back(test_object);
  test_receiver_input_objects(test_objects, true, false);
  test_receiver_input_objects(test_objects, false, true);

  test_objects.objects.clear();
  off_highway_radar::Receiver::Object test_object_invalid;
  // Set values for object A message
  test_object_invalid.a.id = 39;
  test_object_invalid.a.can_id = 0x24E;
  test_object_invalid.a.position.x = 255.9375;
  test_object_invalid.a.position.y = 127.9375;
  test_object_invalid.a.velocity.linear.x = 63.875;
  test_object_invalid.a.velocity.linear.y = 63.875;
  test_object_invalid.a.meas = true;
  test_object_invalid.a.valid = false;
  test_object_invalid.a.hist = true;
  // Set values for one object B message
  test_object_invalid.b.id = 39;
  test_object_invalid.b.can_id = 0x24F;
  test_object_invalid.b.time_since_meas = 0.8191;
  test_object_invalid.b.zone = 24;
  test_object_invalid.b.rcs = 63.5;
  test_object_invalid.b.moving = true;
  test_object_invalid.b.near = true;
  test_object_invalid.b.exist_probability = 0.96875;
  test_objects.objects.push_back(test_object_invalid);
  test_receiver_input_objects(test_objects, true, true);
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

TEST(TestRadarReceiver, test40RandomValidObjects) {
  off_highway_radar::Receiver::Objects test_objects;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(40);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_radar::Receiver::Object test_object;
    // Set values for object A message
    test_object.a.id = id;
    test_object.a.can_id = 0x200 + test_object.a.id * 2;
    test_object.a.position.x = RandomQuantizedGenerator{0.0625, -256, 255.96875}(rng);
    test_object.a.position.y = RandomQuantizedGenerator{0.125, -128, 127.96875}(rng);

    RandomQuantizedGenerator gen_velocity{0.125, -64, 63.9375};
    test_object.a.velocity.linear.x = gen_velocity(rng);
    test_object.a.velocity.linear.y = gen_velocity(rng);
    test_object.a.meas = true;
    test_object.a.valid = true;
    test_object.a.hist = true;

    // Set values for one object B message
    test_object.b.id = test_object.a.id;
    test_object.b.can_id = test_object.a.can_id + 1;
    test_object.b.time_since_meas = RandomQuantizedGenerator{0.0001, 0, 0.8191}(rng);
    test_object.b.zone = RandomQuantizedGenerator{1, 0, 24}(rng);
    test_object.b.rcs = RandomQuantizedGenerator{0.5, -64, 63.5}(rng);
    test_object.b.moving = true;
    test_object.b.near = true;
    test_object.b.exist_probability = RandomQuantizedGenerator{0.03125, 0, 0.96875}(rng);

    ROS_DEBUG_STREAM(
      "Object ID " << std::to_string(test_object.a.id) <<
        " with CAN ID " << std::to_string(test_object.a.can_id) << " generated.");

    test_objects.objects.push_back(test_object);
  }

  test_receiver_input_objects(test_objects, true, true);
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
