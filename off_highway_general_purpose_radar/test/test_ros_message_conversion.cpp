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

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "../src/converter/ros_message_conversion.hpp"
#include "helpers/random_generator.hpp"

class TestRosMessageConversion : public ::testing::Test
{
protected:
  off_highway_general_purpose_radar_msgs::msg::Target create_target_with_min_values()
  {
    off_highway_general_purpose_radar_msgs::msg::Target target;

    target.header.stamp = rclcpp::Time(0, 0);
    target.header.frame_id = "gpr_frame";

    // TargetA with min values based on CAN signal definitions
    target.a.can_id = 0;
    target.a.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
    target.a.id = 0;
    // RadialDistance: 12-bit unsigned, factor 0.0625, range [0, 255.9375m]
    target.a.radial_distance = 0.0;
    // RadialVelocity: 12-bit signed, factor 0.0625, range [-128, 127.9375 m/s]
    target.a.radial_velocity = -128.0;
    // ReflectedPower: 8-bit unsigned, factor 0.5, offset -50, range [-50, 77.5 dBm²]
    target.a.reflected_power = -50.0;
    // AzimuthAngle: 14-bit signed, factor 0.0002, range [-1.6384, 1.6382 rad]
    target.a.azimuth_angle = -1.6384;
    target.a.measured = false;

    // TargetB with min values
    target.b.can_id = 0;
    target.b.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
    target.b.id = 0;
    // AzimuthAngleStd: 12-bit unsigned, factor 0.0002, range [0, 0.8191 rad]
    target.b.azimuth_angle_std = 0.0;
    // RadialVelocityStd: 9-bit unsigned, factor 0.03125, range [0, 15.96875 m/s]
    target.b.radial_velocity_std = 0.0;
    // RadialDistanceStd: 10-bit unsigned, factor 0.0625, range [0, 63.9375 m]
    target.b.radial_distance_std = 0.0;
    // ExistProbability: 10-bit unsigned, factor 0.001, range [0, 1.023]
    target.b.exist_probability = 0.0;
    // TimeSinceMeas: 10-bit unsigned, factor 0.01, range [0, 10.23 s]
    target.b.time_since_meas = 0.0;

    return target;
  }

  off_highway_general_purpose_radar_msgs::msg::Target create_target_with_max_values()
  {
    off_highway_general_purpose_radar_msgs::msg::Target target;

    target.header.stamp = rclcpp::Time(100, 0);
    target.header.frame_id = "gpr_frame";

    // TargetA with max values
    target.a.can_id = 0xFFFFFFFF;
    target.a.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(100).nanosec(0);
    target.a.id = 63;  // 6-bit max
    // RadialDistance: 12-bit unsigned max: 4095 * 0.0625 = 255.9375m
    target.a.radial_distance = 255.9375;
    // RadialVelocity: 12-bit signed max: 2047 * 0.0625 = 127.9375 m/s
    target.a.radial_velocity = 127.9375;
    // ReflectedPower: 8-bit unsigned max: 255 * 0.5 - 50 = 77.5 dBm²
    target.a.reflected_power = 77.5;
    // AzimuthAngle: 14-bit signed max: 8191 * 0.0002 = 1.6382 rad
    target.a.azimuth_angle = 1.6382;
    target.a.measured = true;

    // TargetB with max values
    target.b.can_id = 0xFFFFFFFF;
    target.b.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(100).nanosec(0);
    target.b.id = 63;  // 6-bit max
    // AzimuthAngleStd: 12-bit unsigned max: 4095 * 0.0002 = 0.8191 rad
    target.b.azimuth_angle_std = 0.8191;
    // RadialVelocityStd: 9-bit unsigned max: 511 * 0.03125 = 15.96875 m/s
    target.b.radial_velocity_std = 15.96875;
    // RadialDistanceStd: 10-bit unsigned max: 1023 * 0.0625 = 63.9375 m
    target.b.radial_distance_std = 63.9375;
    // ExistProbability: 10-bit unsigned max: 1023 * 0.001 = 1.023
    target.b.exist_probability = 1.023;
    // TimeSinceMeas: 10-bit unsigned max: 1023 * 0.01 = 10.23 s
    target.b.time_since_meas = 10.23;

    return target;
  }

  off_highway_general_purpose_radar_msgs::msg::Target create_target_with_random_values()
  {
    off_highway_general_purpose_radar_msgs::msg::Target target;

    // Use RandomQuantizedGenerator with actual CAN signal ranges
    RandomQuantizedGenerator dr_gen(0.0625, 0.0, 255.9375);  // 12-bit unsigned
    RandomQuantizedGenerator vr_gen(0.0625, -128.0, 127.9375);  // 12-bit signed
    RandomQuantizedGenerator rp_gen(0.5, -50.0, 77.5);  // 8-bit unsigned with offset
    RandomQuantizedGenerator phi_gen(0.0002, -1.6384, 1.6382);  // 14-bit signed
    RandomQuantizedGenerator phi_std_gen(0.0002, 0.0, 0.8191);  // 12-bit unsigned
    RandomQuantizedGenerator vr_std_gen(0.03125, 0.0, 15.96875);  // 9-bit unsigned
    RandomQuantizedGenerator dr_std_gen(0.0625, 0.0, 63.9375);  // 10-bit unsigned
    RandomQuantizedGenerator prob_gen(0.001, 0.0, 1.023);  // 10-bit unsigned
    RandomQuantizedGenerator time_gen(0.01, 0.0, 10.23);  // 10-bit unsigned

    RandomQuantizedGenerator id_gen(1, 0, 63);  // 6-bit
    RandomQuantizedGenerator bool_gen(1, 0, 1);  // Boolean

    target.header.stamp = rclcpp::Time(1, 500000000);
    target.header.frame_id = "gpr_frame";

    // TargetA with random values
    target.a.can_id = 0x100;
    target.a.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(1).nanosec(0);
    target.a.id = id_gen();
    target.a.radial_distance = dr_gen();
    target.a.radial_velocity = vr_gen();
    target.a.reflected_power = rp_gen();
    target.a.azimuth_angle = phi_gen();
    target.a.measured = (bool_gen() == 1);

    // TargetB with random values
    target.b.can_id = 0x101;
    target.b.stamp = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(1).nanosec(0);
    target.b.id = target.a.id;  // Same ID
    target.b.azimuth_angle_std = phi_std_gen();
    target.b.radial_velocity_std = vr_std_gen();
    target.b.radial_distance_std = dr_std_gen();
    target.b.exist_probability = prob_gen();
    target.b.time_since_meas = time_gen();

    return target;
  }

  void verify_conversion(
    const off_highway_general_purpose_radar_msgs::msg::Target & input_target,
    const sensor_msgs::msg::PointCloud2 & pcl_msg)
  {
    ASSERT_EQ(pcl_msg.width, 1u);
    ASSERT_EQ(pcl_msg.height, 1u);
    ASSERT_TRUE(pcl_msg.is_dense);

    // Create iterators to read data
    sensor_msgs::PointCloud2ConstIterator<float> x(pcl_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(pcl_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(pcl_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> radial_velocity(pcl_msg, "radial_velocity");
    sensor_msgs::PointCloud2ConstIterator<float> reflected_power(pcl_msg, "reflected_power");
    sensor_msgs::PointCloud2ConstIterator<float> azimuth_angle_std(pcl_msg, "azimuth_angle_std");
    sensor_msgs::PointCloud2ConstIterator<float> radial_velocity_std(pcl_msg,
      "radial_velocity_std");
    sensor_msgs::PointCloud2ConstIterator<float> radial_distance_std(pcl_msg,
      "radial_distance_std");
    sensor_msgs::PointCloud2ConstIterator<float> exist_probability(pcl_msg, "exist_probability");
    sensor_msgs::PointCloud2ConstIterator<float> time_since_meas(pcl_msg, "time_since_meas");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> can_id_a(pcl_msg, "can_id_a");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> can_id_b(pcl_msg, "can_id_b");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> id_a(pcl_msg, "id_a");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> measured(pcl_msg, "measured");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> id_b(pcl_msg, "id_b");

    // Calculate expected x and y from polar coordinates (using TargetA fields)
    float expected_x = static_cast<float>(input_target.a.radial_distance *
      std::cos(input_target.a.azimuth_angle));
    float expected_y = static_cast<float>(input_target.a.radial_distance *
      std::sin(input_target.a.azimuth_angle));

    // Verify all fields
    EXPECT_FLOAT_EQ(*x, expected_x);
    EXPECT_FLOAT_EQ(*y, expected_y);
    EXPECT_FLOAT_EQ(*z, 0.0f);
    EXPECT_FLOAT_EQ(*radial_velocity, static_cast<float>(input_target.a.radial_velocity));
    EXPECT_FLOAT_EQ(*reflected_power, static_cast<float>(input_target.a.reflected_power));
    EXPECT_FLOAT_EQ(*azimuth_angle_std, static_cast<float>(input_target.b.azimuth_angle_std));
    EXPECT_FLOAT_EQ(*radial_velocity_std, static_cast<float>(input_target.b.radial_velocity_std));
    EXPECT_FLOAT_EQ(*radial_distance_std, static_cast<float>(input_target.b.radial_distance_std));
    EXPECT_FLOAT_EQ(*exist_probability, static_cast<float>(input_target.b.exist_probability));
    EXPECT_FLOAT_EQ(*time_since_meas, static_cast<float>(input_target.b.time_since_meas));
    EXPECT_EQ(*can_id_a, input_target.a.can_id);
    EXPECT_EQ(*can_id_b, input_target.b.can_id);
    EXPECT_EQ(*id_a, input_target.a.id);
    EXPECT_EQ(*measured, input_target.a.measured);
    EXPECT_EQ(*id_b, input_target.b.id);
  }
};

TEST_F(TestRosMessageConversion, testMinValues)
{
  auto target = create_target_with_min_values();

  off_highway_general_purpose_radar_msgs::msg::Targets targets_msg;
  targets_msg.header.stamp = rclcpp::Time(1, 0);
  targets_msg.header.frame_id = "gpr_frame";
  targets_msg.targets.push_back(target);

  auto pcl_msg = off_highway_general_purpose_radar::to_msg(
    targets_msg,
    rclcpp::Time(1, 0),
    "gpr_frame");

  verify_conversion(target, pcl_msg);
}

TEST_F(TestRosMessageConversion, testMaxValues)
{
  auto target = create_target_with_max_values();

  off_highway_general_purpose_radar_msgs::msg::Targets targets_msg;
  targets_msg.header.stamp = rclcpp::Time(1, 0);
  targets_msg.header.frame_id = "gpr_frame";
  targets_msg.targets.push_back(target);

  auto pcl_msg = off_highway_general_purpose_radar::to_msg(
    targets_msg,
    rclcpp::Time(1, 0),
    "gpr_frame");

  verify_conversion(target, pcl_msg);
}

TEST_F(TestRosMessageConversion, testRandomValues)
{
  auto target = create_target_with_random_values();

  off_highway_general_purpose_radar_msgs::msg::Targets targets_msg;
  targets_msg.header.stamp = rclcpp::Time(1, 0);
  targets_msg.header.frame_id = "gpr_frame";
  targets_msg.targets.push_back(target);

  auto pcl_msg = off_highway_general_purpose_radar::to_msg(
    targets_msg,
    rclcpp::Time(1, 0),
    "gpr_frame");

  verify_conversion(target, pcl_msg);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
