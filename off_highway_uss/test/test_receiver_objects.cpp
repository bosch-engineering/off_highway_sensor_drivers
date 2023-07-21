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

#include "test_helper.hpp"
#include "off_highway_uss/receiver.hpp"
#include "off_highway_common/helper.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using off_highway_common::auto_static_cast;
using namespace std::chrono_literals;

static constexpr double kMetersToCentimeters = 0.01;

class ObjectsPublisher : public rclcpp::Node
{
public:
  explicit ObjectsPublisher(off_highway_uss_msgs::msg::Objects test_objects)
  : Node("off_highway_uss_objects_pub")
  {
    off_highway_uss::Receiver receiver;
    auto msg_def = receiver.get_messages();

    // Initialize can msg
    can_msgs::msg::Frame can_msg_object;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Publish objects
    for (off_highway_uss_msgs::msg::Object test_object : test_objects.objects) {
      // Check if Multiplexor should be active or not
      if (test_object.id < 10) {
        auto_static_cast(can_msg_object.id, 0x180 + test_object.id);
        auto_static_cast(msg_def[can_msg_object.id].signals["Multiplexor"].value, 0);
      } else {
        auto_static_cast(can_msg_object.id, 0x180 + test_object.id - 10);
        auto_static_cast(msg_def[can_msg_object.id].signals["Multiplexor"].value, 1);
      }
      // Cast object
      auto_static_cast(can_msg_object.header.stamp, now());
      off_highway_common::Message & object_msg = msg_def[can_msg_object.id];
      auto_static_cast(object_msg.signals["1stPointX"].value, test_object.position_first.x);
      auto_static_cast(object_msg.signals["1stPointY"].value, test_object.position_first.y);
      auto_static_cast(object_msg.signals["ExistProbability"].value, test_object.exist_probability);
      auto_static_cast(object_msg.signals["2ndPointX"].value, test_object.position_second.x);
      auto_static_cast(object_msg.signals["2ndPointY"].value, test_object.position_second.y);
      auto_static_cast(object_msg.signals["ObjectType"].value, test_object.object_type);

      // Encode message
      object_msg.encode(can_msg_object.data);

      // Publish
      publisher_->publish(can_msg_object);

      // If object_type == 0, the object is not valid and won't be processed
      if (test_object.object_type != 0) {
        defined_object_ids++;
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
  : Node("off_highway_uss_objects_sub")
  {
    subscriber_ = this->create_subscription<off_highway_uss_msgs::msg::Objects>(
      "objects", 2,
      std::bind(&ObjectsSubscriber::objects_callback, this, std::placeholders::_1));
  }

  off_highway_uss_msgs::msg::Objects get_objects()
  {
    return received_objects_;
  }

private:
  void objects_callback(const off_highway_uss_msgs::msg::Objects msg)
  {
    received_objects_ = msg;
  }
  rclcpp::Subscription<off_highway_uss_msgs::msg::Objects>::SharedPtr subscriber_;
  off_highway_uss_msgs::msg::Objects received_objects_;
};  // ObjectsSubscriber

class TestUssReceiver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<off_highway_uss::Receiver>("uss_receiver_test_node");

    // Get parameter values from yaml file
    std::string package_directory =
      ament_index_cpp::get_package_share_directory("off_highway_uss");
    std::string filename = "/config/receiver_params.yaml";
    auto params = rclcpp::parameter_map_from_yaml_file(package_directory + filename);
    ASSERT_TRUE(
      node_->set_parameters_atomically(params.at("/off_highway_uss_receiver")).successful);

    // Overwrite allowed age to avoid timing issues in unit tests
    ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("allowed_age", 0.1)).successful);

    objects_subscriber_ = std::make_shared<ObjectsSubscriber>();
  }

  void publish_objects(off_highway_uss_msgs::msg::Objects objects);
  off_highway_uss_msgs::msg::Objects get_objects();
  void verify_objects(
    off_highway_uss_msgs::msg::Objects test_objects,
    off_highway_uss_msgs::msg::Objects received_objects);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  off_highway_uss::Receiver::SharedPtr node_;

  std::shared_ptr<ObjectsPublisher> objects_publisher_;
  std::shared_ptr<ObjectsSubscriber> objects_subscriber_;
};

void TestUssReceiver::publish_objects(off_highway_uss_msgs::msg::Objects objects)
{
  objects_publisher_ = std::make_shared<ObjectsPublisher>(objects);
  spin_receiver(100ms);
}

off_highway_uss_msgs::msg::Objects TestUssReceiver::get_objects()
{
  spin_subscriber(10ms);
  off_highway_uss_msgs::msg::Objects subscribed_objects_ = objects_subscriber_->get_objects();
  return subscribed_objects_;
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
    rclcpp::spin_some(objects_subscriber_);
    rclcpp::sleep_for(10ms);
  }
}

void TestUssReceiver::verify_objects(
  off_highway_uss_msgs::msg::Objects test_objects,
  off_highway_uss_msgs::msg::Objects received_objects)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("objects"), 1U);
  EXPECT_EQ(node_->count_subscribers("objects"), 1U);

  static constexpr std::array<double, 8> kExistProbabilitySteps{0.0, 17.0, 23.0, 30.0, 40.0, 60.0,
    80.0, 100.0};

  // Check object
  uint8_t found_object_ids = 0;
  for (off_highway_uss_msgs::msg::Object receiver_object : received_objects.objects) {
    // Search received object in test objects
    off_highway_uss_msgs::msg::Object current_test_object;
    for (off_highway_uss_msgs::msg::Object test_object : test_objects.objects) {
      if (receiver_object.id == test_object.id) {
        current_test_object = test_object;
        break;
      }
    }

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
  EXPECT_EQ(objects_publisher_->get_defined_object_ids(), found_object_ids);
}

TEST_F(TestUssReceiver, testObjectsZero) {
  off_highway_uss_msgs::msg::Objects test_objects;
  off_highway_uss_msgs::msg::Object test_object_0;
  // Set values for object A message
  test_object_0.id = 0;
  test_object_0.position_first.x = 0;
  test_object_0.position_first.y = 0;
  test_object_0.exist_probability = 0;
  test_object_0.position_second.x = 0;
  test_object_0.position_second.y = 0;
  test_object_0.object_type = 1;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_0);

  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}


TEST_F(TestUssReceiver, testObjectsMin) {
  off_highway_uss_msgs::msg::Objects test_objects;
  off_highway_uss_msgs::msg::Object test_object_min;
  // Set all values to min
  test_object_min.id = 0;
  test_object_min.position_first.x = -1024 * kMetersToCentimeters;
  test_object_min.position_first.y = -1024 * kMetersToCentimeters;
  test_object_min.exist_probability = 0;
  test_object_min.position_second.x = -1024 * kMetersToCentimeters;
  test_object_min.position_second.y = -1024 * kMetersToCentimeters;
  test_object_min.object_type = 1;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_min);

  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}

TEST_F(TestUssReceiver, testObjectsMax) {
  off_highway_uss_msgs::msg::Objects test_objects;
  off_highway_uss_msgs::msg::Object test_object_max;
  // Set all values to max
  test_object_max.id = 19;
  test_object_max.position_first.x = 1022 * kMetersToCentimeters;
  test_object_max.position_first.y = 1022 * kMetersToCentimeters;
  test_object_max.exist_probability = 7;
  test_object_max.position_second.x = 1022 * kMetersToCentimeters;
  test_object_max.position_second.y = 1022 * kMetersToCentimeters;
  test_object_max.object_type = 3;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_max);

  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}

TEST_F(TestUssReceiver, testObjectsRand) {
  off_highway_uss_msgs::msg::Objects test_objects;
  off_highway_uss_msgs::msg::Object test_object_rand;
  // Set all values to random
  test_object_rand.id = 12;
  test_object_rand.position_first.x = 1001 * kMetersToCentimeters;
  test_object_rand.position_first.y = -1002 * kMetersToCentimeters;
  test_object_rand.exist_probability = 4;
  test_object_rand.position_second.x = -1024 * kMetersToCentimeters;
  test_object_rand.position_second.y = 1022 * kMetersToCentimeters;
  test_object_rand.object_type = 2;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_rand);

  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}

TEST_F(TestUssReceiver, testObjectsType0) {
  off_highway_uss_msgs::msg::Objects test_objects;
  off_highway_uss_msgs::msg::Object test_object_type_0;
  // Set all values to random and object_type to 0
  test_object_type_0.id = 15;
  test_object_type_0.position_first.x = 100 * kMetersToCentimeters;
  test_object_type_0.position_first.y = -200 * kMetersToCentimeters;
  test_object_type_0.exist_probability = 3;
  test_object_type_0.position_second.x = -300 * kMetersToCentimeters;
  test_object_type_0.position_second.y = 400 * kMetersToCentimeters;
  test_object_type_0.object_type = 0;  // Object is filtered out if type == 0
  test_objects.objects.push_back(test_object_type_0);

  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}

TEST_F(TestUssReceiver, test20RandomValidObjects) {
  off_highway_uss_msgs::msg::Objects test_objects;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(20);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_uss_msgs::msg::Object test_object;

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


    test_objects.objects.push_back(test_object);
  }
  publish_objects(test_objects);
  verify_objects(test_objects, get_objects());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
