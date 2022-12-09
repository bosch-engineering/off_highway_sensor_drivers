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

#include "off_highway_uss/receiver.hpp"
#include "off_highway_common/helper.hpp"

using off_highway_common::auto_static_cast;

static constexpr double kMetersToCentimeters = 0.01;

struct CallbackHelper
{
  void cb_receiver_info(const off_highway_uss_msgs::Information & msg)
  {
    receiver_info = msg;
    ROS_DEBUG_STREAM(receiver_info);
  }

  void cb_receiver_objects(const off_highway_uss_msgs::Objects & msg)
  {
    receiver_objects = msg;
    ROS_DEBUG_STREAM(receiver_objects);
  }


  void cb_receiver_echos(const off_highway_uss_msgs::DirectEchos & msg)
  {
    receiver_echos = msg;
    ROS_DEBUG_STREAM(receiver_echos);
  }

  void cb_receiver_range(const off_highway_uss_msgs::MaxDetectionRange & msg)
  {
    receiver_range = msg;
    ROS_DEBUG_STREAM(receiver_range);
  }

  off_highway_uss_msgs::Information receiver_info;
  off_highway_uss_msgs::Objects receiver_objects;
  off_highway_uss_msgs::DirectEchos receiver_echos;
  off_highway_uss_msgs::MaxDetectionRange receiver_range;
};

inline void test_receiver_input_info(off_highway_uss::Receiver::Information information)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_uss::Receiver receiver;
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
  auto_static_cast(can_msg_info.id, 0x17C);
  auto_static_cast(can_msg_info.header.stamp, ros::Time::now());
  off_highway_common::Message & info_msg = msg_def[can_msg_info.id];
  auto_static_cast(info_msg.signals["NumberSensors"].value, information.number_sensors);
  auto_static_cast(info_msg.signals["SendingPattern"].value, information.sending_pattern);
  auto_static_cast(info_msg.signals["OperatingMode"].value, information.operating_mode);
  auto_static_cast(info_msg.signals["OutsideTemperature"].value, information.outside_temperature);
  auto_static_cast(info_msg.signals["SensorBlindness"].value, information.sensor_blindness);
  auto_static_cast(info_msg.signals["Sensitivity"].value, information.sensitivity);
  auto_static_cast(info_msg.signals["SensorFaulted"].value, information.sensor_faulted);
  auto_static_cast(info_msg.signals["FailureStatus"].value, information.failure_status);

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
  ROS_DEBUG_STREAM("Checking USS receiver info.");
  EXPECT_EQ(cb_helper.receiver_info.number_sensors, information.number_sensors);
  EXPECT_EQ(cb_helper.receiver_info.sending_pattern, information.sending_pattern);
  EXPECT_EQ(cb_helper.receiver_info.operating_mode, information.operating_mode);
  EXPECT_EQ(cb_helper.receiver_info.outside_temperature, information.outside_temperature);
  EXPECT_EQ(cb_helper.receiver_info.sensor_blindness, information.sensor_blindness);
  EXPECT_EQ(cb_helper.receiver_info.sensitivity, information.sensitivity);
  EXPECT_EQ(cb_helper.receiver_info.sensor_faulted, information.sensor_faulted);
  EXPECT_EQ(cb_helper.receiver_info.failure_status, information.failure_status);
}

TEST(TestUssReceiver, testInfoValidValues) {
  // Set all values to 0
  off_highway_uss::Receiver::Information test_information;
  test_information.number_sensors = 0;
  test_information.sending_pattern = 0;
  test_information.operating_mode = 0;
  test_information.outside_temperature = 0;
  test_information.sensor_blindness = 0;
  test_information.sensitivity = 0;
  test_information.sensor_faulted = 0;
  test_information.failure_status = 0;
  test_receiver_input_info(test_information);

  // Set all values to min
  off_highway_uss::Receiver::Information test_information_min;
  test_information_min.number_sensors = 0;
  test_information_min.sending_pattern = 0;
  test_information_min.operating_mode = 0;
  test_information_min.outside_temperature = -40;
  test_information_min.sensor_blindness = 0;
  test_information_min.sensitivity = 0;
  test_information_min.sensor_faulted = 0;
  test_information_min.failure_status = 0;
  test_receiver_input_info(test_information_min);

  // Set all values to max
  off_highway_uss::Receiver::Information test_information_max;
  test_information_max.number_sensors = 7;
  test_information_max.sending_pattern = 3;
  test_information_max.operating_mode = 7;
  test_information_max.outside_temperature = 215;
  test_information_max.sensor_blindness = 4095;
  test_information_max.sensitivity = 7;
  test_information_max.sensor_faulted = 4095;
  test_information_max.failure_status = 1;
  test_receiver_input_info(test_information_max);

  // Set all values to random
  off_highway_uss::Receiver::Information test_information_rand;
  test_information_rand.number_sensors = 4;
  test_information_rand.sending_pattern = 2;
  test_information_rand.operating_mode = 1;
  test_information_rand.outside_temperature = 123;
  test_information_rand.sensor_blindness = 1234;
  test_information_rand.sensitivity = 3;
  test_information_rand.sensor_faulted = 2345;
  test_information_rand.failure_status = 0;
  test_receiver_input_info(test_information_rand);
}

inline void test_receiver_input_objects(off_highway_uss_msgs::Objects test_objects)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_uss::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_objects = nh.subscribe(
    "test_receiver/objects", 1, &CallbackHelper::cb_receiver_objects,
    &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_objects.getNumPublishers(), 1U);

  // Set up object CAN message, cast and encode values
  can_msgs::Frame can_msg_object;

  uint8_t defined_object_ids = 0;
  for (off_highway_uss_msgs::Object test_object : test_objects.objects) {
    // Check if Multiplexor should be active or not
    if (test_object.id < 10) {
      auto_static_cast(can_msg_object.id, 0x180 + test_object.id);
      auto_static_cast(msg_def[can_msg_object.id].signals["Multiplexor"].value, 0);
    } else {
      auto_static_cast(can_msg_object.id, 0x180 + test_object.id - 10);
      auto_static_cast(msg_def[can_msg_object.id].signals["Multiplexor"].value, 1);
    }
    // Cast object
    ROS_DEBUG_STREAM(
      "Casting message with CAN ID " << std::to_string(
        can_msg_object.id) << " with Object ID " << std::to_string(test_object.id));
    auto_static_cast(can_msg_object.header.stamp, ros::Time::now());
    off_highway_common::Message & object_msg = msg_def[can_msg_object.id];
    auto_static_cast(object_msg.signals["1stPointX"].value, test_object.position_first.x);
    auto_static_cast(object_msg.signals["1stPointY"].value, test_object.position_first.y);
    auto_static_cast(object_msg.signals["ExistProbability"].value, test_object.exist_probability);
    auto_static_cast(object_msg.signals["2ndPointX"].value, test_object.position_second.x);
    auto_static_cast(object_msg.signals["2ndPointY"].value, test_object.position_second.y);
    auto_static_cast(object_msg.signals["ObjectType"].value, test_object.object_type);

    // Encode message
    object_msg.encode(can_msg_object.data);

    // Publish object message as input to receiver
    pub.publish(can_msg_object);

    // If object_type == 0, the object is not valid and won't be processed
    if (test_object.object_type != 0) {
      defined_object_ids++;
    }
  }

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing test CAN data
  ros::spinOnce();  // Publish CAN test msgs, and run callack in receiver node
  ros::WallDuration(0.04).sleep();  // Wait for manage_and_publish callback in receiver node
  ros::spinOnce();  // Run manage_and_publish callback
  ros::WallDuration(0.02).sleep();  // Delay after publishing objects
  ros::spinOnce();  // Run objects callback in test

  static constexpr std::array<double, 8> kExistProbabilitySteps{0.0, 17.0, 23.0, 30.0, 40.0, 60.0,
    80.0, 100.0};

  // Check object
  uint8_t found_object_ids = 0;
  for (off_highway_uss_msgs::Object receiver_object : cb_helper.receiver_objects.objects) {
    // Search received object in test objects
    ROS_DEBUG_STREAM(
      "Searching Object with ID " << std::to_string(
        receiver_object.id) << " in received test objects.");
    off_highway_uss_msgs::Object current_test_object;
    for (off_highway_uss_msgs::Object test_object : test_objects.objects) {
      if (receiver_object.id == test_object.id) {
        current_test_object = test_object;
        break;
      }
    }

    ROS_DEBUG_STREAM("Checking Object with ID " << std::to_string(receiver_object.id) << ".");
    uint8_t exist_probability_step = static_cast<uint8_t>(current_test_object.exist_probability);
    EXPECT_EQ(receiver_object.id, current_test_object.id);
    EXPECT_NEAR(receiver_object.position_first.x, current_test_object.position_first.x, 0.01);
    EXPECT_NEAR(receiver_object.position_first.y, current_test_object.position_first.y, 0.01);
    EXPECT_EQ(receiver_object.exist_probability, kExistProbabilitySteps.at(exist_probability_step));
    EXPECT_NEAR(receiver_object.position_second.x, current_test_object.position_second.x, 0.01);
    EXPECT_NEAR(receiver_object.position_second.y, current_test_object.position_second.y, 0.01);
    EXPECT_EQ(receiver_object.object_type, current_test_object.object_type);
    found_object_ids++;
  }
  ROS_DEBUG_STREAM(
    "Objects defined: " << std::to_string(
      defined_object_ids) << ", objects found: " << std::to_string(found_object_ids));

  EXPECT_EQ(defined_object_ids, found_object_ids);
}

TEST(TestUssReceiver, testObject) {
  off_highway_uss_msgs::Objects test_objects;
  off_highway_uss_msgs::Object test_object_0;
  // Set all values to 0
  test_object_0.id = 0;
  test_object_0.position_first.x = 0;
  test_object_0.position_first.y = 0;
  test_object_0.exist_probability = 0;
  test_object_0.position_second.x = 0;
  test_object_0.position_second.y = 0;
  test_object_0.object_type = 1;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_0);
  test_receiver_input_objects(test_objects);

  test_objects.objects.clear();
  off_highway_uss_msgs::Object test_object_min;
  // Set all values to min
  test_object_min.id = 0;
  test_object_min.position_first.x = -1024 * kMetersToCentimeters;
  test_object_min.position_first.y = -1024 * kMetersToCentimeters;
  test_object_min.exist_probability = 0;
  test_object_min.position_second.x = -1024 * kMetersToCentimeters;
  test_object_min.position_second.y = -1024 * kMetersToCentimeters;
  test_object_min.object_type = 1;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_min);
  test_receiver_input_objects(test_objects);

  test_objects.objects.clear();
  off_highway_uss_msgs::Object test_object_max;
  // Set all values to max
  test_object_max.id = 19;
  test_object_max.position_first.x = 1022 * kMetersToCentimeters;
  test_object_max.position_first.y = 1022 * kMetersToCentimeters;
  test_object_max.exist_probability = 7;
  test_object_max.position_second.x = 1022 * kMetersToCentimeters;
  test_object_max.position_second.y = 1022 * kMetersToCentimeters;
  test_object_max.object_type = 3;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_max);
  test_receiver_input_objects(test_objects);

  test_objects.objects.clear();
  off_highway_uss_msgs::Object test_object_rand;
  // Set all values to random
  test_object_rand.id = 12;
  test_object_rand.position_first.x = 1001 * kMetersToCentimeters;
  test_object_rand.position_first.y = -1002 * kMetersToCentimeters;
  test_object_rand.exist_probability = 4;
  test_object_rand.position_second.x = -1024 * kMetersToCentimeters;
  test_object_rand.position_second.y = 1022 * kMetersToCentimeters;
  test_object_rand.object_type = 2;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_rand);
  test_receiver_input_objects(test_objects);

  test_objects.objects.clear();
  off_highway_uss_msgs::Object test_object_type_0;
  // Set all values to random and object_type to 0
  test_object_type_0.id = 15;
  test_object_type_0.position_first.x = 100 * kMetersToCentimeters;
  test_object_type_0.position_first.y = -200 * kMetersToCentimeters;
  test_object_type_0.exist_probability = 3;
  test_object_type_0.position_second.x = -300 * kMetersToCentimeters;
  test_object_type_0.position_second.y = 400 * kMetersToCentimeters;
  test_object_type_0.object_type = 0;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_type_0);
  test_receiver_input_objects(test_objects);
}

class RandomQuantizedGenerator
{
public:
  RandomQuantizedGenerator(double resolution, double min, double max)
  : resolution{resolution}
  {
    int64_t min_as_int = min / resolution;
    int64_t max_as_int = max / resolution;
    this->uniform_distribution = std::uniform_int_distribution<int64_t>{min_as_int, max_as_int};
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

TEST(TestUssReceiver, testObjects) {
  off_highway_uss_msgs::Objects test_objects;
  // Create array with unique IDs
  std::vector<uint8_t> ids(20);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  // Push 20 objects
  for (auto id : ids) {
    off_highway_uss_msgs::Object test_object;
    // Set values for object
    test_object.id = id;
    // Set position values in m
    RandomQuantizedGenerator gen{2, -1024, 1022};
    test_object.position_first.x = gen(rng) * kMetersToCentimeters;
    test_object.position_first.y = gen(rng) * kMetersToCentimeters;
    test_object.position_second.x = gen(rng) * kMetersToCentimeters;
    test_object.position_second.y = gen(rng) * kMetersToCentimeters;
    test_object.exist_probability = RandomQuantizedGenerator{1, 0, 7}(rng);
    test_object.object_type = RandomQuantizedGenerator{1, 1, 3}(rng);
    ROS_DEBUG_STREAM(
      "Object ID " << std::to_string(test_object.id) << " with CAN ID " <<
        std::to_string(0x180 + test_object.id % 10) << " generated.");
    test_objects.objects.push_back(test_object);
  }

  test_receiver_input_objects(test_objects);
}

inline void test_receiver_input_echos(off_highway_uss_msgs::DirectEchos test_echos)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_uss::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_echos = nh.subscribe(
    "test_receiver/direct_echos", 10, &CallbackHelper::cb_receiver_echos,
    &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_echos.getNumPublishers(), 1U);

  // Set up echo CAN message, cast and encode values
  can_msgs::Frame can_msg_echo;

  uint8_t defined_echo_ids = 0;
  for (off_highway_uss_msgs::DirectEcho test_echo : test_echos.direct_echos) {
    // Cast echo
    auto_static_cast(can_msg_echo.id, 0x170 + test_echo.id);
    ROS_DEBUG_STREAM(
      "Casting message with CAN ID " << std::to_string(
        can_msg_echo.id) << " with Object ID " << std::to_string(test_echo.id));
    auto_static_cast(can_msg_echo.header.stamp, ros::Time::now());
    off_highway_common::Message & echo_msg = msg_def[can_msg_echo.id];
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

    // Publish echo in received_messages
    pub.publish(can_msg_echo);
    defined_echo_ids++;
  }

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();  // Delay after publishing test CAN data
  ros::spinOnce();  // Publish CAN test msgs, and run callack in receiver node
  ros::WallDuration(0.04).sleep();  // Wait for manage_and_publish callback in receiver node
  ros::spinOnce();  // Run manage_and_publish callback
  ros::WallDuration(0.02).sleep();  // Delay after publishing echos
  ros::spinOnce();  // Run echo callback in test

  // Check echos
  uint8_t found_echo_ids = 0;
  for (off_highway_uss_msgs::DirectEcho receiver_echo : cb_helper.receiver_echos.direct_echos) {
    // Search received echo in test echos
    ROS_DEBUG_STREAM(
      "Searching Echo with ID " << std::to_string(receiver_echo.id) << " in received test echos.");
    off_highway_uss_msgs::DirectEcho current_test_echo;
    for (off_highway_uss_msgs::DirectEcho test_echo : test_echos.direct_echos) {
      if (receiver_echo.id == test_echo.id) {
        current_test_echo = test_echo;
        break;
      }
    }

    ROS_DEBUG_STREAM("Checking Echo with ID " << std::to_string(current_test_echo.id) << ".");
    EXPECT_EQ(receiver_echo.first.distance, current_test_echo.first.distance);
    EXPECT_EQ(receiver_echo.second.distance, current_test_echo.second.distance);
    EXPECT_EQ(receiver_echo.first_filtered.distance, current_test_echo.first_filtered.distance);
    EXPECT_EQ(receiver_echo.first.amplitude, current_test_echo.first.amplitude);
    EXPECT_EQ(receiver_echo.second.amplitude, current_test_echo.second.amplitude);
    EXPECT_EQ(receiver_echo.first_filtered.amplitude, current_test_echo.first_filtered.amplitude);
    found_echo_ids++;
  }
  ROS_DEBUG_STREAM(
    "Echos defined: " << std::to_string(defined_echo_ids) << ", echos found: " <<
      std::to_string(found_echo_ids));

  EXPECT_EQ(defined_echo_ids, found_echo_ids);
}


TEST(TestUssReceiver, testEcho) {
  off_highway_uss_msgs::DirectEchos test_echos;
  off_highway_uss_msgs::DirectEcho test_echo_0;
  // Set all values to 0
  test_echo_0.id = 0;
  test_echo_0.first.distance = 0;
  test_echo_0.second.distance = 0;
  test_echo_0.first_filtered.distance = 0;
  test_echo_0.first.amplitude = 0;
  test_echo_0.second.amplitude = 0;
  test_echo_0.first_filtered.amplitude = 0;
  test_echos.direct_echos.push_back(test_echo_0);
  test_receiver_input_echos(test_echos);

  test_echos.direct_echos.clear();
  off_highway_uss_msgs::DirectEcho test_echo_min;
  // Set all values to min
  test_echo_min.id = 0;
  test_echo_min.first.distance = 0;
  test_echo_min.second.distance = 0;
  test_echo_min.first_filtered.distance = 0;
  test_echo_min.first.amplitude = 0;
  test_echo_min.second.amplitude = 0;
  test_echo_min.first_filtered.amplitude = 0;
  test_echos.direct_echos.push_back(test_echo_min);
  test_receiver_input_echos(test_echos);

  test_echos.direct_echos.clear();
  off_highway_uss_msgs::DirectEcho test_echo_max;
  // Set all values to max
  test_echo_max.id = 10;
  test_echo_max.first.distance = 1023;
  test_echo_max.second.distance = 1023;
  test_echo_max.first_filtered.distance = 1023;
  test_echo_max.first.amplitude = 63;
  test_echo_max.second.amplitude = 63;
  test_echo_max.first_filtered.amplitude = 63;
  test_echos.direct_echos.push_back(test_echo_max);
  test_receiver_input_echos(test_echos);

  test_echos.direct_echos.clear();
  off_highway_uss_msgs::DirectEcho test_echo_rand;
  // Set all values to random
  test_echo_rand.id = 5;
  test_echo_rand.first.distance = 555;
  test_echo_rand.second.distance = 222;
  test_echo_rand.first_filtered.distance = 111;
  test_echo_rand.first.amplitude = 33;
  test_echo_rand.second.amplitude = 22;
  test_echo_rand.first_filtered.amplitude = 11;
  test_echos.direct_echos.push_back(test_echo_rand);
  test_receiver_input_echos(test_echos);
}

TEST(TestUssReceiver, testEchos) {
  off_highway_uss_msgs::DirectEchos test_echos;
  // Create array with unique IDs
  std::vector<uint8_t> ids(12);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  // Push 12 echos
  for (auto id : ids) {
    off_highway_uss_msgs::DirectEcho test_echo;
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
    ROS_DEBUG_STREAM("Echo ID " << std::to_string(test_echo.id) << " generated.");
    test_echos.direct_echos.push_back(test_echo);
  }

  test_receiver_input_echos(test_echos);
}

inline void test_receiver_range(off_highway_uss_msgs::MaxDetectionRange range)
{
  // Set up ROS test environment
  ros::NodeHandle nh;
  off_highway_uss::Receiver receiver;
  auto msg_def = receiver.get_messages();
  CallbackHelper cb_helper;

  ros::Publisher pub = nh.advertise<can_msgs::Frame>("received_messages", 100);
  ros::Subscriber sub_range = nh.subscribe(
    "test_receiver/max_detection_range", 10,
    &CallbackHelper::cb_receiver_range, &cb_helper);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub_range.getNumPublishers(), 1U);

  // Set up range CAN message, cast and encode values
  can_msgs::Frame can_msg_range;

  // Cast and encode range
  auto_static_cast(can_msg_range.id, 0x17D);
  auto_static_cast(can_msg_range.header.stamp, ros::Time::now());
  off_highway_common::Message & range_msg = msg_def[can_msg_range.id];
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

  // Publish received_messages
  pub.publish(can_msg_range);

  // Spin to receive and process received_messages
  ros::WallDuration(0.01).sleep();   // Delay after publishing test CAN data
  ros::spinOnce();  // Receive CAN message in receiver node and run process function
  ros::spinOnce();  // Publish topics
  ros::WallDuration(0.01).sleep();
  ros::spinOnce();  // Run CAN callback in test

  // Check info values
  ROS_DEBUG_STREAM("Checking USS receiver MaxDetRange.");
  for (auto i = 0; i < 12; i++) {
    EXPECT_NEAR(
      cb_helper.receiver_range.max_detection_ranges[i], range.max_detection_ranges[i], 0.36667);
  }
}

TEST(TestUssReceiver, testMaxDetRange) {
  double range_scaling = 36.667 * kMetersToCentimeters;

  // Set all values to any
  off_highway_uss_msgs::MaxDetectionRange test_range_rand;
  test_range_rand.max_detection_ranges[0] = 1 * range_scaling;
  test_range_rand.max_detection_ranges[1] = 2 * range_scaling;
  test_range_rand.max_detection_ranges[2] = 3 * range_scaling;
  test_range_rand.max_detection_ranges[3] = 4 * range_scaling;
  test_range_rand.max_detection_ranges[4] = 5 * range_scaling;
  test_range_rand.max_detection_ranges[5] = 6 * range_scaling;
  test_range_rand.max_detection_ranges[6] = 7 * range_scaling;
  test_range_rand.max_detection_ranges[7] = 8 * range_scaling;
  test_range_rand.max_detection_ranges[8] = 9 * range_scaling;
  test_range_rand.max_detection_ranges[9] = 10 * range_scaling;
  test_range_rand.max_detection_ranges[10] = 11 * range_scaling;
  test_range_rand.max_detection_ranges[11] = 12 * range_scaling;
  test_receiver_range(test_range_rand);

  // Set all values to min
  off_highway_uss_msgs::MaxDetectionRange test_range_min;
  test_range_min.max_detection_ranges[0] = 0;
  test_range_min.max_detection_ranges[1] = 0;
  test_range_min.max_detection_ranges[2] = 0;
  test_range_min.max_detection_ranges[3] = 0;
  test_range_min.max_detection_ranges[4] = 0;
  test_range_min.max_detection_ranges[5] = 0;
  test_range_min.max_detection_ranges[6] = 0;
  test_range_min.max_detection_ranges[7] = 0;
  test_range_min.max_detection_ranges[8] = 0;
  test_range_min.max_detection_ranges[9] = 0;
  test_range_min.max_detection_ranges[10] = 0;
  test_range_min.max_detection_ranges[11] = 0;
  test_receiver_range(test_range_min);

  // Set all values to max
  off_highway_uss_msgs::MaxDetectionRange test_range_max;
  test_range_max.max_detection_ranges[0] = 15 * range_scaling;  // = 550.005 * 0.01
  test_range_max.max_detection_ranges[1] = 15 * range_scaling;
  test_range_max.max_detection_ranges[2] = 15 * range_scaling;
  test_range_max.max_detection_ranges[3] = 15 * range_scaling;
  test_range_max.max_detection_ranges[4] = 15 * range_scaling;
  test_range_max.max_detection_ranges[5] = 15 * range_scaling;
  test_range_max.max_detection_ranges[6] = 15 * range_scaling;
  test_range_max.max_detection_ranges[7] = 15 * range_scaling;
  test_range_max.max_detection_ranges[8] = 15 * range_scaling;
  test_range_max.max_detection_ranges[9] = 15 * range_scaling;
  test_range_max.max_detection_ranges[10] = 15 * range_scaling;
  test_range_max.max_detection_ranges[11] = 15 * range_scaling;
  test_receiver_range(test_range_max);
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
