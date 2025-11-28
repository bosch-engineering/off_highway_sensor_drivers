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

#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "off_highway_uss_msgs/msg/object.hpp"
#include "off_highway_uss_msgs/msg/objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

namespace off_highway_uss
{

inline
sensor_msgs::msg::PointCloud2 to_msg(
  const off_highway_uss_msgs::msg::Objects & objects,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & interpolated_points)
{
  using sensor_msgs::PointCloud2Iterator;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id);
  msg.is_dense = true;

  // Calculate total number of points
  size_t total_points = 0;
  for (size_t i = 0; i < objects.objects.size(); ++i) {
    if (i < interpolated_points.size()) {
      total_points += interpolated_points[i].size();
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "exist_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "object_type", 1, sensor_msgs::msg::PointField::UINT8,
    "id", 1, sensor_msgs::msg::PointField::UINT8
  );

  modifier.resize(total_points);

  PointCloud2Iterator<float> x(msg, "x");
  PointCloud2Iterator<float> y(msg, "y");
  PointCloud2Iterator<float> z(msg, "z");
  PointCloud2Iterator<float> exist_probability(msg, "exist_probability");
  PointCloud2Iterator<uint8_t> object_type(msg, "object_type");
  PointCloud2Iterator<uint8_t> id(msg, "id");

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects[i];

    if (i < interpolated_points.size()) {
      for (const auto & point : interpolated_points[i]) {
        *x = static_cast<float>(point.x);
        *y = static_cast<float>(point.y);
        *z = static_cast<float>(point.z);
        *exist_probability = static_cast<float>(object.exist_probability);
        *object_type = object.object_type;
        *id = object.id;

        // Increment all iterators
        ++x;
        ++y;
        ++z;
        ++exist_probability;
        ++object_type;
        ++id;
      }
    }
  }

  return msg;
}

}  // namespace off_highway_uss
