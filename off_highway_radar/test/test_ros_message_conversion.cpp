// Copyright 2024 Robert Bosch GmbH and its subsidiaries
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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "../src/converter/ros_message_conversion.hpp"
#include "helpers/random_generator.hpp"

class TestRosMessageConversion : public ::testing::Test
{
protected:
  off_highway_radar_msgs::msg::Object create_object_with_min_values()
  {
    off_highway_radar_msgs::msg::Object obj;

    obj.header.stamp = rclcpp::Time(0, 0);
    obj.header.frame_id = "radar_frame";

    // ObjectA with min values based on CAN signal definitions
    obj.a.stamp = rclcpp::Time(0, 0);
    obj.a.can_id = 0x200;
    obj.a.id = 0;  // 6-bit unsigned: 0-63
    obj.a.position.x = -256.0;  // dx: 13-bit, factor 0.0625, offset -256 -> range [-256, 255.9375]
    obj.a.position.y = -128.0;  // dy: 12-bit, factor 0.0625, offset -128 -> range [-128, 127.9375]
    obj.a.position.z = 0.0;
    obj.a.velocity.linear.x = -64.0;  // vx: 10-bit, factor 0.125, offset -64 -> range [-64, 63.875]
    obj.a.velocity.linear.y = -64.0;  // vy: 10-bit, factor 0.125, offset -64 -> range [-64, 63.875]
    obj.a.meas = 0;  // 1-bit: 0-1
    obj.a.valid = 0;  // 1-bit: 0-1
    obj.a.hist = 0;  // 1-bit: 0-1

    // ObjectB with min values based on CAN signal definitions
    obj.b.stamp = rclcpp::Time(0, 0);
    obj.b.can_id = 0x201;
    obj.b.id = 0;  // 6-bit unsigned: 0-63
    obj.b.time_since_meas = 0.0;  // 13-bit, factor 0.0001, offset 0 -> range [0, 0.8191]
    obj.b.zone = 0;  // 5-bit unsigned: 0-31
    obj.b.rcs = -64.0;  // 8-bit, factor 0.5, offset -64 -> range [-64, 63.5]
    obj.b.moving = 0;  // 1-bit: 0-1
    obj.b.near = 0;  // 1-bit: 0-1
    obj.b.exist_probability = 0.0;  // 5-bit, factor 0.03125, offset 0 -> range [0, 0.96875]

    return obj;
  }

  off_highway_radar_msgs::msg::Object create_object_with_max_values()
  {
    off_highway_radar_msgs::msg::Object obj;

    obj.header.stamp = rclcpp::Time(100, 0);
    obj.header.frame_id = "radar_frame";

    // ObjectA with max values based on CAN signal definitions
    obj.a.stamp = rclcpp::Time(100, 0);
    obj.a.can_id = 0x200;
    obj.a.id = 63;  // 6-bit unsigned: 0-63
    obj.a.position.x = 255.9375;  // dx max: 8191 * 0.0625 - 256
    obj.a.position.y = 127.9375;  // dy max: 4095 * 0.0625 - 128
    obj.a.position.z = 0.0;
    obj.a.velocity.linear.x = 63.875;  // vx max: 1023 * 0.125 - 64
    obj.a.velocity.linear.y = 63.875;  // vy max: 1023 * 0.125 - 64
    obj.a.meas = 1;
    obj.a.valid = 1;
    obj.a.hist = 1;

    // ObjectB with max values based on CAN signal definitions
    obj.b.stamp = rclcpp::Time(100, 0);
    obj.b.can_id = 0x201;
    obj.b.id = 63;  // 6-bit unsigned: 0-63
    obj.b.time_since_meas = 0.8191;  // 13-bit max: 8191 * 0.0001
    obj.b.zone = 31;  // 5-bit unsigned: 0-31
    obj.b.rcs = 63.5;  // 8-bit max: 255 * 0.5 - 64
    obj.b.moving = 1;
    obj.b.near = 1;
    obj.b.exist_probability = 0.96875;  // 5-bit max: 31 * 0.03125

    return obj;
  }

  off_highway_radar_msgs::msg::Object create_object_with_random_values()
  {
    off_highway_radar_msgs::msg::Object obj;

    // Use RandomQuantizedGenerator with actual CAN signal ranges
    RandomQuantizedGenerator pos_x_gen(0.0625, -256.0, 255.9375);  // dx range
    RandomQuantizedGenerator pos_y_gen(0.0625, -128.0, 127.9375);  // dy range
    RandomQuantizedGenerator vel_x_gen(0.125, -64.0, 63.875);  // vx range
    RandomQuantizedGenerator vel_y_gen(0.125, -64.0, 63.875);  // vy range
    RandomQuantizedGenerator rcs_gen(0.5, -64.0, 63.5);  // RCS range
    RandomQuantizedGenerator prob_gen(0.03125, 0.0, 0.96875);  // wExist range
    RandomQuantizedGenerator time_gen(0.0001, 0.0, 0.8191);  // timeSinceMeas range

    RandomQuantizedGenerator id_dist(0, 63, 1);  // 6-bit ID
    RandomQuantizedGenerator zone_dist(0, 31, 1);  // 5-bit zone
    RandomQuantizedGenerator bool_dist(0, 1, 1);  // 1-bit boolean

    obj.header.stamp = rclcpp::Time(1, 500000000);
    obj.header.frame_id = "radar_frame";

    // ObjectA with random values within valid ranges
    obj.a.stamp = rclcpp::Time(1, 500000000);
    obj.a.can_id = 0x200;
    obj.a.id = id_dist();
    obj.a.position.x = pos_x_gen();
    obj.a.position.y = pos_y_gen();
    obj.a.position.z = 0.0;
    obj.a.velocity.linear.x = vel_x_gen();
    obj.a.velocity.linear.y = vel_y_gen();
    obj.a.meas = bool_dist();
    obj.a.valid = bool_dist();
    obj.a.hist = bool_dist();

    // ObjectB with random values within valid ranges
    obj.b.stamp = rclcpp::Time(1, 500000000);
    obj.b.can_id = 0x201;
    obj.b.id = obj.a.id;  // Keep same id as ObjectA
    obj.b.time_since_meas = time_gen();
    obj.b.zone = zone_dist();
    obj.b.rcs = rcs_gen();
    obj.b.moving = bool_dist();
    obj.b.near = bool_dist();
    obj.b.exist_probability = prob_gen();

    return obj;
  }

  void verify_conversion(
    const off_highway_radar_msgs::msg::Object & input_obj,
    const sensor_msgs::msg::PointCloud2 & pcl_msg)
  {
    ASSERT_EQ(pcl_msg.width, 1u);
    ASSERT_EQ(pcl_msg.height, 1u);
    ASSERT_TRUE(pcl_msg.is_dense);

    // Create iterators to read data
    sensor_msgs::PointCloud2ConstIterator<float> x(pcl_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(pcl_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(pcl_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> vx(pcl_msg, "vx");
    sensor_msgs::PointCloud2ConstIterator<float> vy(pcl_msg, "vy");
    sensor_msgs::PointCloud2ConstIterator<float> time_since_meas(pcl_msg, "time_since_meas");
    sensor_msgs::PointCloud2ConstIterator<float> rcs(pcl_msg, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> exist_probability(pcl_msg, "exist_probability");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> can_id_a(pcl_msg, "can_id_a");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> can_id_b(pcl_msg, "can_id_b");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> id_a(pcl_msg, "id_a");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> id_b(pcl_msg, "id_b");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> zone(pcl_msg, "zone");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> meas(pcl_msg, "meas");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> valid(pcl_msg, "valid");

    // Verify converted values match input
    EXPECT_FLOAT_EQ(*x, static_cast<float>(input_obj.a.position.x));
    EXPECT_FLOAT_EQ(*y, static_cast<float>(input_obj.a.position.y));
    EXPECT_FLOAT_EQ(*z, static_cast<float>(input_obj.a.position.z));
    EXPECT_FLOAT_EQ(*vx, static_cast<float>(input_obj.a.velocity.linear.x));
    EXPECT_FLOAT_EQ(*vy, static_cast<float>(input_obj.a.velocity.linear.y));
    EXPECT_FLOAT_EQ(*time_since_meas, static_cast<float>(input_obj.b.time_since_meas));
    EXPECT_FLOAT_EQ(*rcs, static_cast<float>(input_obj.b.rcs));
    EXPECT_FLOAT_EQ(*exist_probability, static_cast<float>(input_obj.b.exist_probability));
    EXPECT_EQ(*can_id_a, input_obj.a.can_id);
    EXPECT_EQ(*can_id_b, input_obj.b.can_id);
    EXPECT_EQ(*id_a, input_obj.a.id);
    EXPECT_EQ(*id_b, input_obj.b.id);
    EXPECT_EQ(*zone, input_obj.b.zone);
    EXPECT_EQ(*meas, input_obj.a.meas);
    EXPECT_EQ(*valid, input_obj.a.valid);
  }
};

TEST_F(TestRosMessageConversion, testMinValues)
{
  auto obj = create_object_with_min_values();

  off_highway_radar_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "radar_frame";
  objects_msg.objects.push_back(obj);

  auto pcl_msg = off_highway_radar::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "radar_frame");

  verify_conversion(obj, pcl_msg);
}

TEST_F(TestRosMessageConversion, testMaxValues)
{
  auto obj = create_object_with_max_values();

  off_highway_radar_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "radar_frame";
  objects_msg.objects.push_back(obj);

  auto pcl_msg = off_highway_radar::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "radar_frame");

  verify_conversion(obj, pcl_msg);
}

TEST_F(TestRosMessageConversion, testRandomValues)
{
  auto obj = create_object_with_random_values();

  off_highway_radar_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "radar_frame";
  objects_msg.objects.push_back(obj);

  auto pcl_msg = off_highway_radar::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "radar_frame");

  verify_conversion(obj, pcl_msg);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
