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

#include "off_highway_radar/receiver.hpp"
#include "off_highway_common/helper.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using off_highway_common::auto_static_cast;
using namespace std::chrono_literals;

class ObjectsPublisher : public rclcpp::Node
{
public:
  ObjectsPublisher(off_highway_radar_msgs::msg::Objects test_objects, bool send_a, bool send_b)
  : Node("off_highway_radar_objects_pub")
  {
    off_highway_radar::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_object;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Publish objects to from_can_bus
    for (off_highway_radar_msgs::msg::Object test_object : test_objects.objects) {
      if (send_a) {
        auto_static_cast(can_msg_object.id, test_object.a.can_id);
        auto_static_cast(can_msg_object.header.stamp, now());
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

        // Publish
        publisher_->publish(can_msg_object);

        // Object only counts if object A was sent and object is valid
        if (test_object.a.valid) {
          defined_object_ids++;
        }
      }

      if (send_b) {
        // Cast object B message
        auto_static_cast(can_msg_object.id, test_object.b.can_id);
        auto_static_cast(can_msg_object.header.stamp, now());
        off_highway_common::Message & object_b_msg = msg_def[can_msg_object.id];
        auto_static_cast(object_b_msg.signals["ID"].value, test_object.b.id);
        auto_static_cast(
          object_b_msg.signals["timeSinceMeas"].value,
          test_object.b.time_since_meas);
        auto_static_cast(object_b_msg.signals["Zone"].value, test_object.b.zone);
        auto_static_cast(object_b_msg.signals["RCS"].value, test_object.b.rcs);
        auto_static_cast(object_b_msg.signals["Moving"].value, test_object.b.moving);
        auto_static_cast(object_b_msg.signals["Near"].value, test_object.b.near);
        auto_static_cast(object_b_msg.signals["wExist"].value, test_object.b.exist_probability);

        // Encode message
        object_b_msg.encode(can_msg_object.data);

        // Publish
        publisher_->publish(can_msg_object);
      }
    }
  }

  uint8_t get_defined_object_ids()
  {
    return defined_object_ids;
  }

protected:
  uint8_t defined_object_ids = 0;

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};  // ObjectsPublisher

class ObjectsSubscriber : public rclcpp::Node
{
public:
  ObjectsSubscriber()
  : Node("off_highway_radar_objects_sub"), objects_updated_(false)
  {
    subscriber_ = this->create_subscription<off_highway_radar_msgs::msg::Objects>(
      "objects", 1,
      std::bind(&ObjectsSubscriber::objectsCallback, this, std::placeholders::_1));
  }

  off_highway_radar_msgs::msg::Objects get_objects()
  {
    return received_objects_;
  }

  inline bool objectsUpdated()
  {
    return objects_updated_;
  }

  inline void resetObjectsIndicator()
  {
    objects_updated_ = false;
  }

private:
  void objectsCallback(const off_highway_radar_msgs::msg::Objects msg)
  {
    received_objects_ = msg;
    objects_updated_ = true;
  }
  rclcpp::Subscription<off_highway_radar_msgs::msg::Objects>::SharedPtr subscriber_;
  off_highway_radar_msgs::msg::Objects received_objects_;
  bool objects_updated_;
};  // ObjectsSubscriber

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

class TestRadarReceiver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<off_highway_radar::Receiver>("radar_receiver_test_node");

    // Get parameter values from yaml file
    std::string package_directory =
      ament_index_cpp::get_package_share_directory("off_highway_radar");
    std::string filename = "/config/receiver_params.yaml";
    auto params = rclcpp::parameter_map_from_yaml_file(package_directory + filename);
    ASSERT_TRUE(
      node_->set_parameters_atomically(params.at("/off_highway_radar_receiver")).successful);

    // Overwrite allowed age to avoid timing issues in unit tests
    ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("allowed_age", 0.1)).successful);

    objects_subscriber_ = std::make_shared<ObjectsSubscriber>();
  }

  void publish_objects(off_highway_radar_msgs::msg::Objects objects, bool send_a, bool send_b);
  off_highway_radar_msgs::msg::Objects get_objects();
  void verify_objects(
    off_highway_radar_msgs::msg::Objects test_objects,
    off_highway_radar_msgs::msg::Objects received_objects, bool send_a,
    bool send_b);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  off_highway_radar::Receiver::SharedPtr node_;

  std::shared_ptr<ObjectsPublisher> objects_publisher_;
  std::shared_ptr<ObjectsSubscriber> objects_subscriber_;
};

void TestRadarReceiver::publish_objects(
  off_highway_radar_msgs::msg::Objects objects, bool send_a,
  bool send_b)
{
  objects_publisher_ = std::make_shared<ObjectsPublisher>(objects, send_a, send_b);
  spin_receiver(100ms);
}

off_highway_radar_msgs::msg::Objects TestRadarReceiver::get_objects()
{
  spin_subscriber(10ms);
  off_highway_radar_msgs::msg::Objects subscribed_objects_ = objects_subscriber_->get_objects();
  return subscribed_objects_;
}

void TestRadarReceiver::spin_receiver(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
  }
}

void TestRadarReceiver::spin_subscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(objects_subscriber_);
    rclcpp::sleep_for(10ms);
  }
}

void TestRadarReceiver::verify_objects(
  off_highway_radar_msgs::msg::Objects test_objects,
  off_highway_radar_msgs::msg::Objects received_objects, bool send_a, bool send_b)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("objects"), 1U);
  EXPECT_EQ(node_->count_subscribers("objects"), 1U);

  // Check objects
  uint8_t found_object_ids = 0;
  for (off_highway_radar_msgs::msg::Object received_object : received_objects.objects) {
    // Search received object in test objects
    off_highway_radar_msgs::msg::Object current_test_object;
    for (off_highway_radar_msgs::msg::Object test_object : test_objects.objects) {
      if (received_object.a.can_id == test_object.a.can_id) {
        current_test_object = test_object;
        break;
      }
    }

    // Object A and B is only sent if object A was sent and object is valid
    if (send_a && received_object.a.valid) {
      EXPECT_EQ(received_object.a.can_id, current_test_object.a.can_id);
      EXPECT_EQ(received_object.a.id, current_test_object.a.id);
      EXPECT_EQ(received_object.a.position.x, current_test_object.a.position.x);
      EXPECT_EQ(received_object.a.position.y, current_test_object.a.position.y);
      EXPECT_EQ(received_object.a.velocity.linear.x, current_test_object.a.velocity.linear.x);
      EXPECT_EQ(received_object.a.velocity.linear.y, current_test_object.a.velocity.linear.y);
      EXPECT_EQ(received_object.a.meas, current_test_object.a.meas);
      EXPECT_EQ(received_object.a.valid, current_test_object.a.valid);
      EXPECT_EQ(received_object.a.hist, current_test_object.a.hist);
      if (send_b) {
        EXPECT_EQ(received_object.b.can_id, current_test_object.b.can_id);
        EXPECT_EQ(received_object.b.id, current_test_object.b.id);
        EXPECT_NEAR(received_object.b.time_since_meas, current_test_object.b.time_since_meas, 1e-6);
        EXPECT_EQ(received_object.b.zone, current_test_object.b.zone);
        EXPECT_EQ(received_object.b.rcs, current_test_object.b.rcs);
        EXPECT_EQ(received_object.b.moving, current_test_object.b.moving);
        EXPECT_EQ(received_object.b.near, current_test_object.b.near);
        EXPECT_EQ(received_object.b.exist_probability, current_test_object.b.exist_probability);
      } else {
        EXPECT_EQ(received_object.b.can_id, 0u);
        EXPECT_EQ(received_object.b.id, 0u);
        EXPECT_EQ(received_object.b.time_since_meas, 0);
        EXPECT_EQ(received_object.b.zone, 0u);
        EXPECT_EQ(received_object.b.rcs, 0);
        EXPECT_EQ(received_object.b.moving, false);
        EXPECT_EQ(received_object.b.near, false);
        EXPECT_EQ(received_object.b.exist_probability, 0);
      }
      found_object_ids++;
    } else {
      // If object A was not sent or object is not valid, no object should be received
      EXPECT_TRUE(received_objects.objects.empty());
    }
  }
  EXPECT_EQ(objects_publisher_->get_defined_object_ids(), found_object_ids);
}

TEST_F(TestRadarReceiver, testObjectsZero) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object_0;
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

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsValidValues) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object_rand;
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

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsMinValues) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object_min;
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

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsMaxValues) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object_max;
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

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsIncompleteValuesA) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object;
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

  bool send_a = true;
  bool send_b = false;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsIncompleteValuesB) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object;
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

  bool send_a = false;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testObjectsInvalidValues) {
  off_highway_radar_msgs::msg::Objects test_objects;
  off_highway_radar_msgs::msg::Object test_object_invalid;
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

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

TEST_F(TestRadarReceiver, test40RandomValidObjects) {
  off_highway_radar_msgs::msg::Objects test_objects;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(40);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_radar_msgs::msg::Object test_object;
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

    test_objects.objects.push_back(test_object);
  }

  bool send_a = true;
  bool send_b = true;
  publish_objects(test_objects, send_a, send_b);
  verify_objects(test_objects, get_objects(), send_a, send_b);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
