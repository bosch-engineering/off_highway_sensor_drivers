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

#include "off_highway_general_purpose_radar_msgs/msg/target.hpp"
#include "off_highway_general_purpose_radar_msgs/msg/targets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

namespace off_highway_general_purpose_radar
{

inline
sensor_msgs::msg::PointCloud2 to_msg(
  const off_highway_general_purpose_radar_msgs::msg::Targets & targets, const rclcpp::Time & stamp,
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
    // "padding", 1, sensor_msgs::msg::PointField::FLOAT32,  // TODO(rcp1-beg) Needed?
    "radial_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "reflected_power", 1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth_angle_std", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_velocity_std", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_distance_std", 1, sensor_msgs::msg::PointField::FLOAT32,
    "exist_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time_since_meas", 1, sensor_msgs::msg::PointField::FLOAT32,
    "can_id_a", 1, sensor_msgs::msg::PointField::UINT32,
    "can_id_b", 1, sensor_msgs::msg::PointField::UINT32,
    "id_a", 1, sensor_msgs::msg::PointField::UINT8,
    "measured", 1, sensor_msgs::msg::PointField::UINT8,
    "id_b", 1, sensor_msgs::msg::PointField::UINT8
  );

  modifier.resize(targets.targets.size());

  PointCloud2Iterator<float> x(msg, "x");
  PointCloud2Iterator<float> y(msg, "y");
  PointCloud2Iterator<float> z(msg, "z");
  PointCloud2Iterator<float> radial_velocity(msg, "radial_velocity");
  PointCloud2Iterator<float> reflected_power(msg, "reflected_power");
  PointCloud2Iterator<float> azimuth_angle_std(msg, "azimuth_angle_std");
  PointCloud2Iterator<float> radial_velocity_std(msg, "radial_velocity_std");
  PointCloud2Iterator<float> radial_distance_std(msg, "radial_distance_std");
  PointCloud2Iterator<float> exist_probability(msg, "exist_probability");
  PointCloud2Iterator<float> time_since_meas(msg, "time_since_meas");
  PointCloud2Iterator<uint32_t> can_id_a(msg, "can_id_a");
  PointCloud2Iterator<uint32_t> can_id_b(msg, "can_id_b");
  PointCloud2Iterator<uint8_t> id_a(msg, "id_a");
  PointCloud2Iterator<uint8_t> measured(msg, "measured");
  PointCloud2Iterator<uint8_t> id_b(msg, "id_b");

  for (const auto & target : targets.targets) {
    *x = static_cast<float>(target.a.radial_distance * cos(target.a.azimuth_angle));
    *y = static_cast<float>(target.a.radial_distance * sin(target.a.azimuth_angle));
    *z = 0.0f;
    *radial_velocity = static_cast<float>(target.a.radial_velocity);
    *reflected_power = static_cast<float>(target.a.reflected_power);
    *azimuth_angle_std = static_cast<float>(target.b.azimuth_angle_std);
    *radial_velocity_std = static_cast<float>(target.b.radial_velocity_std);
    *radial_distance_std = static_cast<float>(target.b.radial_distance_std);
    *exist_probability = static_cast<float>(target.b.exist_probability);
    *time_since_meas = static_cast<float>(target.b.time_since_meas);
    *can_id_a = target.a.can_id;
    *can_id_b = target.b.can_id;
    *id_a = target.a.id;
    *measured = target.a.measured;
    *id_b = target.b.id;

    // Increment all iterators
    ++x;
    ++y;
    ++z;
    ++radial_velocity;
    ++reflected_power;
    ++azimuth_angle_std;
    ++radial_velocity_std;
    ++radial_distance_std;
    ++exist_probability;
    ++time_since_meas;
    ++can_id_a;
    ++can_id_b;
    ++id_a;
    ++measured;
    ++id_b;
  }

  return msg;
}

}  // namespace off_highway_general_purpose_radar
