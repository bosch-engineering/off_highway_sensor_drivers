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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "../src/converter/ros_message_conversion.hpp"
#include "helpers/random_generator.hpp"

class TestRosMessageConversion : public ::testing::Test
{
protected:
  static constexpr double kCentimeterToMeter = 0.01;

  off_highway_uss_msgs::msg::Object create_object_with_min_values()
  {
    off_highway_uss_msgs::msg::Object obj;

    obj.header.stamp = rclcpp::Time(0, 0);
    obj.header.frame_id = "uss_frame";
    obj.id = 0;

    // Object with min values based on CAN signal definitions
    // 1stPointX/Y and 2ndPointX/Y: 10-bit signed, factor 2*0.01 = 0.02
    // Range for 10-bit signed: -512 to 511 raw -> -10.24m to 10.22m
    obj.position_first.x = -10.24;  // -512 * 0.02
    obj.position_first.y = -10.24;
    obj.position_second.x = -10.24;
    obj.position_second.y = -10.24;
    obj.exist_probability = 0.0;  // 3-bit: mapped to discrete steps [0, 0.125, 0.25, ..., 0.875]
    obj.object_type = 0;  // 2-bit: 0-3

    return obj;
  }

  off_highway_uss_msgs::msg::Object create_object_with_max_values()
  {
    off_highway_uss_msgs::msg::Object obj;

    obj.header.stamp = rclcpp::Time(100, 0);
    obj.header.frame_id = "uss_frame";
    obj.id = 255;  // Max object ID (multiplexor * kCountObjectFrames + frame_id)

    // Object with max values based on CAN signal definitions
    // 10-bit signed max: 511 * 0.02 = 10.22m
    obj.position_first.x = 10.22;
    obj.position_first.y = 10.22;
    obj.position_second.x = 10.22;
    obj.position_second.y = 10.22;
    obj.exist_probability = 0.875;  // Max in discrete steps (3-bit: 7 steps)
    obj.object_type = 3;  // 2-bit max: 0-3

    return obj;
  }

  off_highway_uss_msgs::msg::Object create_object_with_random_values()
  {
    off_highway_uss_msgs::msg::Object obj;

    // Use RandomQuantizedGenerator with actual CAN signal ranges
    RandomQuantizedGenerator pos_gen(
      2 * kCentimeterToMeter, -10.24, 10.22);  // 10-bit signed

    RandomQuantizedGenerator type_gen(1, 0, 3);  // 2-bit: 0-3
    RandomQuantizedGenerator id_gen(1, 0, 255);  // Object ID

    // Discrete exist probability steps (3-bit)
    std::vector<double> prob_steps = {0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875};
    RandomQuantizedGenerator prob_index_gen(1, 0, prob_steps.size() - 1);

    obj.header.stamp = rclcpp::Time(1, 500000000);
    obj.header.frame_id = "uss_frame";

    // Object with random values within valid ranges
    obj.id = id_gen();
    obj.position_first.x = pos_gen();
    obj.position_first.y = pos_gen();
    obj.position_second.x = pos_gen();
    obj.position_second.y = pos_gen();
    obj.exist_probability = prob_steps[static_cast<size_t>(prob_index_gen())];
    obj.object_type = type_gen();

    return obj;
  }

  void verify_conversion(
    const off_highway_uss_msgs::msg::Object & input_obj,
    const sensor_msgs::msg::PointCloud2 & pcl_msg,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points)
  {
    ASSERT_EQ(pcl_msg.width, interpolated_points.size());
    ASSERT_EQ(pcl_msg.height, 1u);
    ASSERT_TRUE(pcl_msg.is_dense);

    // Create iterators to read data
    sensor_msgs::PointCloud2ConstIterator<float> x(pcl_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(pcl_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(pcl_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> exist_probability(pcl_msg, "exist_probability");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> object_type(pcl_msg, "object_type");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> id(pcl_msg, "id");

    // Verify all points
    for (const auto & point : interpolated_points) {
      EXPECT_FLOAT_EQ(*x, static_cast<float>(point.x));
      EXPECT_FLOAT_EQ(*y, static_cast<float>(point.y));
      EXPECT_FLOAT_EQ(*z, static_cast<float>(point.z));
      EXPECT_FLOAT_EQ(*exist_probability, static_cast<float>(input_obj.exist_probability));
      EXPECT_EQ(*object_type, input_obj.object_type);
      EXPECT_EQ(*id, input_obj.id);

      ++x; ++y; ++z; ++exist_probability; ++object_type; ++id;
    }
  }
};

TEST_F(TestRosMessageConversion, testMinValues)
{
  auto obj = create_object_with_min_values();

  off_highway_uss_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "uss_frame";
  objects_msg.objects.push_back(obj);

  // Create simple interpolated points for testing
  std::vector<std::vector<geometry_msgs::msg::Point>> interpolated_points(1);
  geometry_msgs::msg::Point p1, p2;
  p1.x = obj.position_first.x;
  p1.y = obj.position_first.y;
  p1.z = 0.0;
  p2.x = obj.position_second.x;
  p2.y = obj.position_second.y;
  p2.z = 0.0;
  interpolated_points[0] = {p1, p2};

  auto pcl_msg = off_highway_uss::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "uss_frame",
    interpolated_points);

  verify_conversion(obj, pcl_msg, interpolated_points[0]);
}

TEST_F(TestRosMessageConversion, testMaxValues)
{
  auto obj = create_object_with_max_values();

  off_highway_uss_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "uss_frame";
  objects_msg.objects.push_back(obj);

  // Create simple interpolated points for testing
  std::vector<std::vector<geometry_msgs::msg::Point>> interpolated_points(1);
  geometry_msgs::msg::Point p1, p2;
  p1.x = obj.position_first.x;
  p1.y = obj.position_first.y;
  p1.z = 0.0;
  p2.x = obj.position_second.x;
  p2.y = obj.position_second.y;
  p2.z = 0.0;
  interpolated_points[0] = {p1, p2};

  auto pcl_msg = off_highway_uss::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "uss_frame",
    interpolated_points);

  verify_conversion(obj, pcl_msg, interpolated_points[0]);
}

TEST_F(TestRosMessageConversion, testRandomValues)
{
  auto obj = create_object_with_random_values();

  off_highway_uss_msgs::msg::Objects objects_msg;
  objects_msg.header.stamp = rclcpp::Time(1, 0);
  objects_msg.header.frame_id = "uss_frame";
  objects_msg.objects.push_back(obj);

  // Create simple interpolated points for testing
  std::vector<std::vector<geometry_msgs::msg::Point>> interpolated_points(1);
  geometry_msgs::msg::Point p1, p2;
  p1.x = obj.position_first.x;
  p1.y = obj.position_first.y;
  p1.z = 0.0;
  p2.x = obj.position_second.x;
  p2.y = obj.position_second.y;
  p2.z = 0.0;
  interpolated_points[0] = {p1, p2};

  auto pcl_msg = off_highway_uss::to_msg(
    objects_msg,
    rclcpp::Time(1, 0),
    "uss_frame",
    interpolated_points);

  verify_conversion(obj, pcl_msg, interpolated_points[0]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
