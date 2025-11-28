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

#include "off_highway_radar_msgs/msg/object.hpp"
#include "off_highway_radar_msgs/msg/objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

namespace off_highway_radar
{

inline
sensor_msgs::msg::PointCloud2 to_msg(
  const off_highway_radar_msgs::msg::Objects & objects, const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  using sensor_msgs::PointCloud2Iterator;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id);
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    15,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vy", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time_since_meas", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rcs", 1, sensor_msgs::msg::PointField::FLOAT32,
    "exist_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "can_id_a", 1, sensor_msgs::msg::PointField::UINT32,
    "can_id_b", 1, sensor_msgs::msg::PointField::UINT32,
    "id_a", 1, sensor_msgs::msg::PointField::UINT8,
    "id_b", 1, sensor_msgs::msg::PointField::UINT8,
    "zone", 1, sensor_msgs::msg::PointField::UINT8,
    "meas", 1, sensor_msgs::msg::PointField::UINT8,
    "valid", 1, sensor_msgs::msg::PointField::UINT8
  );

  modifier.resize(objects.objects.size());

  PointCloud2Iterator<float> x(msg, "x");
  PointCloud2Iterator<float> y(msg, "y");
  PointCloud2Iterator<float> z(msg, "z");
  PointCloud2Iterator<float> vx(msg, "vx");
  PointCloud2Iterator<float> vy(msg, "vy");
  PointCloud2Iterator<float> time_since_meas(msg, "time_since_meas");
  PointCloud2Iterator<float> rcs(msg, "rcs");
  PointCloud2Iterator<float> exist_probability(msg, "exist_probability");
  PointCloud2Iterator<uint32_t> can_id_a(msg, "can_id_a");
  PointCloud2Iterator<uint32_t> can_id_b(msg, "can_id_b");
  PointCloud2Iterator<uint8_t> id_a(msg, "id_a");
  PointCloud2Iterator<uint8_t> id_b(msg, "id_b");
  PointCloud2Iterator<uint8_t> zone(msg, "zone");
  PointCloud2Iterator<uint8_t> meas(msg, "meas");
  PointCloud2Iterator<uint8_t> valid(msg, "valid");

  for (const auto & object : objects.objects) {
    *x = static_cast<float>(object.a.position.x);
    *y = static_cast<float>(object.a.position.y);
    *z = static_cast<float>(object.a.position.z);
    *vx = static_cast<float>(object.a.velocity.linear.x);
    *vy = static_cast<float>(object.a.velocity.linear.y);
    *time_since_meas = static_cast<float>(object.b.time_since_meas);
    *rcs = static_cast<float>(object.b.rcs);
    *exist_probability = static_cast<float>(object.b.exist_probability);
    *can_id_a = object.a.can_id;
    *can_id_b = object.b.can_id;
    *id_a = object.a.id;
    *id_b = object.b.id;
    *zone = object.b.zone;
    *meas = static_cast<uint8_t>(object.a.meas);
    *valid = static_cast<uint8_t>(object.a.valid);

    // Increment all iterators
    ++x;
    ++y;
    ++z;
    ++vx;
    ++vy;
    ++time_since_meas;
    ++rcs;
    ++exist_probability;
    ++can_id_a;
    ++can_id_b;
    ++id_a;
    ++id_b;
    ++zone;
    ++meas;
    ++valid;
  }

  return msg;
}

}  // namespace off_highway_radar
