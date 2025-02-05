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
#include <stdexcept>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "off_highway_general_purpose_radar/receiver.hpp"
#include "off_highway_can/helper.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

inline double sgn(double x)
{
  return (x > 0) - (x < 0);
}

class TargetsPublisher : public rclcpp::Node
{
public:
  TargetsPublisher(
    off_highway_general_purpose_radar_msgs::msg::Targets test_targets, bool send_a, bool send_b,
    off_highway_can::Receiver::Messages & msg_def)
  : Node("off_highway_general_purpose_radar_targets_pub")
  {
    // Initialize can msg
    can_msgs::msg::Frame can_msg_target;

    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);


    // Publish targets to from_can_bus
    for (off_highway_general_purpose_radar_msgs::msg::Target test_target : test_targets.targets) {
      if (send_a) {
        auto_static_cast(can_msg_target.id, test_target.a.can_id);
        auto_static_cast(can_msg_target.header.stamp, now());
        off_highway_can::Message & target_a_msg = msg_def[can_msg_target.id];
        auto_static_cast(target_a_msg.signals["AMessage"].value, true);
        auto_static_cast(target_a_msg.signals["ID"].value, test_target.a.id);
        auto_static_cast(target_a_msg.signals["dr"].value, test_target.a.radial_distance);
        auto_static_cast(target_a_msg.signals["vr"].value, test_target.a.radial_velocity);
        auto_static_cast(target_a_msg.signals["dbPower"].value, test_target.a.reflected_power);
        auto_static_cast(target_a_msg.signals["phi"].value, test_target.a.azimuth_angle);
        auto_static_cast(target_a_msg.signals["measured"].value, test_target.a.measured);

        // Encode message
        target_a_msg.encode(can_msg_target.data);

        // Publish
        publisher_->publish(can_msg_target);

        // Target only counts if target A was sent
        if (test_target.a.measured) {
          defined_target_ids++;
        }
      }

      if (send_b) {
        // Cast target B message
        auto_static_cast(can_msg_target.id, test_target.b.can_id);
        auto_static_cast(can_msg_target.header.stamp, now());
        off_highway_can::Message & target_b_msg = msg_def[can_msg_target.id];
        auto_static_cast(target_b_msg.signals["AMessage"].value, false);
        auto_static_cast(target_b_msg.signals["ID"].value, test_target.b.id);
        auto_static_cast(
          target_b_msg.signals["timeSinceMeas"].value,
          test_target.b.time_since_meas);
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
        publisher_->publish(can_msg_target);
      }
    }
  }

  uint8_t get_defined_target_ids()
  {
    return defined_target_ids;
  }

protected:
  uint8_t defined_target_ids = 0;

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};  // TargetsPublisher

class TargetsSubscriber : public rclcpp::Node
{
public:
  TargetsSubscriber()
  : Node("off_highway_general_purpose_radar_targets_sub"), targets_updated_(false)
  {
    subscriber_ = this->create_subscription<off_highway_general_purpose_radar_msgs::msg::Targets>(
      "targets", 1,
      std::bind(&TargetsSubscriber::targetsCallback, this, std::placeholders::_1));
  }

  off_highway_general_purpose_radar_msgs::msg::Targets get_targets()
  {
    return received_targets_;
  }

  inline bool targetsUpdated()
  {
    return targets_updated_;
  }

  inline void resetTargetsIndicator()
  {
    targets_updated_ = false;
  }

private:
  void targetsCallback(const off_highway_general_purpose_radar_msgs::msg::Targets msg)
  {
    received_targets_ = msg;
    targets_updated_ = true;
  }
  rclcpp::Subscription<off_highway_general_purpose_radar_msgs::msg::Targets>::SharedPtr subscriber_;
  off_highway_general_purpose_radar_msgs::msg::Targets received_targets_;
  bool targets_updated_;
};  // TargetsSubscriber

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
    // Overwrite allowed age to avoid timing issues in unit tests
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("allowed_age", 1.0)
    };
    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);
    node_ = std::make_shared<off_highway_general_purpose_radar::Receiver>(node_options);

    ASSERT_EQ(node_->get_parameter("allowed_age").as_double(), 1.0);

    targets_subscriber_ = std::make_shared<TargetsSubscriber>();
  }

  void publish_targets(
    off_highway_general_purpose_radar_msgs::msg::Targets targets, bool send_a,
    bool send_b);
  off_highway_general_purpose_radar_msgs::msg::Targets get_targets();
  void verify_targets(
    off_highway_general_purpose_radar_msgs::msg::Targets test_targets,
    off_highway_general_purpose_radar_msgs::msg::Targets received_targets, bool send_a,
    bool send_b);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  std::shared_ptr<off_highway_general_purpose_radar::Receiver> node_;

  std::shared_ptr<TargetsPublisher> targets_publisher_;
  std::shared_ptr<TargetsSubscriber> targets_subscriber_;
};

void TestRadarReceiver::publish_targets(
  off_highway_general_purpose_radar_msgs::msg::Targets targets, bool send_a,
  bool send_b)
{
  auto msg_def = node_->get_messages();
  targets_publisher_ = std::make_shared<TargetsPublisher>(targets, send_a, send_b, msg_def);
  spin_receiver(100ms);
}

off_highway_general_purpose_radar_msgs::msg::Targets TestRadarReceiver::get_targets()
{
  spin_subscriber(500ms);
  off_highway_general_purpose_radar_msgs::msg::Targets subscribed_targets_ =
    targets_subscriber_->get_targets();
  return subscribed_targets_;
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
    rclcpp::spin_some(targets_subscriber_);
    rclcpp::sleep_for(100ms);
  }
}

void TestRadarReceiver::verify_targets(
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets,
  off_highway_general_purpose_radar_msgs::msg::Targets received_targets, bool send_a, bool send_b)
{
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("targets"), 1U);
  EXPECT_EQ(node_->count_subscribers("targets"), 1U);

  // Check targets
  uint8_t found_target_ids = 0;
  for (off_highway_general_purpose_radar_msgs::msg::Target received_target :
    received_targets.targets)
  {
    // Search received target in test targets
    off_highway_general_purpose_radar_msgs::msg::Target current_test_target;
    for (off_highway_general_purpose_radar_msgs::msg::Target test_target : test_targets.targets) {
      if (received_target.a.can_id == test_target.a.can_id) {
        current_test_target = test_target;
        break;
      }
    }

    // Target A and B is only sent if target A was sent and measured
    if (send_a && received_target.a.measured) {
      EXPECT_EQ(received_target.a.can_id, current_test_target.a.can_id);
      EXPECT_EQ(received_target.a.id, current_test_target.a.id);
      EXPECT_EQ(received_target.a.radial_distance, current_test_target.a.radial_distance);
      EXPECT_EQ(received_target.a.radial_velocity, current_test_target.a.radial_velocity);
      EXPECT_EQ(received_target.a.reflected_power, current_test_target.a.reflected_power);
      EXPECT_NEAR(received_target.a.azimuth_angle, current_test_target.a.azimuth_angle, 1e-7);
      EXPECT_EQ(received_target.a.measured, current_test_target.a.measured);
      if (send_b) {
        EXPECT_EQ(received_target.b.can_id, current_test_target.b.can_id);
        EXPECT_EQ(received_target.b.id, current_test_target.b.id);
        EXPECT_NEAR(
          received_target.b.time_since_meas,
          current_test_target.b.time_since_meas, 1e-7);
        EXPECT_NEAR(
          received_target.b.azimuth_angle_std,
          current_test_target.b.azimuth_angle_std, 1e-4);
        EXPECT_EQ(
          received_target.b.radial_velocity_std,
          current_test_target.b.radial_velocity_std);
        EXPECT_EQ(
          received_target.b.radial_distance_std,
          current_test_target.b.radial_distance_std);
        EXPECT_EQ(received_target.b.exist_probability, current_test_target.b.exist_probability);
      } else {
        EXPECT_EQ(received_target.b.can_id, 0u);
        EXPECT_EQ(received_target.b.id, 0u);
        EXPECT_EQ(received_target.b.time_since_meas, 0u);
        EXPECT_EQ(received_target.b.azimuth_angle_std, 0u);
        EXPECT_EQ(received_target.b.radial_velocity_std, 0u);
        EXPECT_EQ(received_target.b.radial_distance_std, 0u);
        EXPECT_EQ(received_target.b.exist_probability, 0u);
      }
      found_target_ids++;
    } else {
      // If target A was not sent or target is not measured, no target should be received
      EXPECT_TRUE(received_targets.targets.empty());
    }
  }
  EXPECT_EQ(targets_publisher_->get_defined_target_ids(), found_target_ids);
}

TEST_F(TestRadarReceiver, testTargetZero) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target_0;
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

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsValidValues) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target_rand;
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

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsMinValues) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target_min;
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

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsMaxValues) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target_max;
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

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsIncompleteValuesA) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target;
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

  bool send_a = true;
  bool send_b = false;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsIncompleteValuesB) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target;
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

  bool send_a = false;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testTargetsInvalidValues) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target_invalid;
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

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, test48RandomValidTargets) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  // Create vector with unique IDs
  std::vector<uint8_t> ids(48);
  std::iota(std::begin(ids), std::end(ids), 0);
  std::default_random_engine rng;
  std::shuffle(std::begin(ids), std::end(ids), rng);

  for (auto id : ids) {
    off_highway_general_purpose_radar_msgs::msg::Target test_target;
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

    test_targets.targets.push_back(test_target);
  }

  bool send_a = true;
  bool send_b = true;
  publish_targets(test_targets, send_a, send_b);
  verify_targets(test_targets, get_targets(), send_a, send_b);
}

TEST_F(TestRadarReceiver, testInvalidTargetIdA) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target;
  // Set values for target A message
  test_target.a.id = 48;
  test_target.a.can_id = 0x200;
  test_target.a.measured = true;

  test_targets.targets.push_back(test_target);

  bool send_a = true;
  bool send_b = false;

  EXPECT_THROW(publish_targets(test_targets, send_a, send_b), std::runtime_error);
}

TEST_F(TestRadarReceiver, testInvalidTargetIdB) {
  off_highway_general_purpose_radar_msgs::msg::Targets test_targets;
  off_highway_general_purpose_radar_msgs::msg::Target test_target;
  // Set values for target B message
  test_target.b.id = 48;
  test_target.b.can_id = 0x201;

  test_targets.targets.push_back(test_target);

  bool send_a = false;
  bool send_b = true;

  EXPECT_THROW(publish_targets(test_targets, send_a, send_b), std::runtime_error);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
