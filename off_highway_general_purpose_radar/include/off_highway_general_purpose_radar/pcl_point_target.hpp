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

#pragma once

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"
#include "pcl/register_point_struct.h"

#include "off_highway_general_purpose_radar_msgs/msg/target.hpp"

namespace off_highway_general_purpose_radar
{

/**
 * \brief Target as PCL point
 */
struct EIGEN_ALIGN16 PclPointTarget
{
  PCL_ADD_POINT4D;
  float radial_velocity{0.};
  float reflected_power{0.};
  float azimuth_angle_std{0.};
  float radial_velocity_std{0.};
  float radial_distance_std{0.};
  float exist_probability{0.};
  float time_since_meas{0.};
  uint32_t can_id_a{0};
  uint32_t can_id_b{0};
  uint8_t id_a{0};
  uint8_t measured{0};
  uint8_t id_b{0};

  /**
   * \brief Construct a new PclPointTarget object from a target
   *
   * \param l Target input
   */
  explicit PclPointTarget(const off_highway_general_purpose_radar_msgs::msg::Target & l)
  : x(static_cast<float>(l.a.radial_distance * cos(l.a.azimuth_angle))),
    y(static_cast<float>(l.a.radial_distance * sin(l.a.azimuth_angle))),
    z(0.),
    radial_velocity(static_cast<float>(l.a.radial_velocity)),
    reflected_power(static_cast<float>(l.a.reflected_power)),
    azimuth_angle_std(static_cast<float>(l.b.azimuth_angle_std)),
    radial_velocity_std(static_cast<float>(l.b.radial_velocity_std)),
    radial_distance_std(static_cast<float>(l.b.radial_distance_std)),
    exist_probability(static_cast<float>(l.b.exist_probability)),
    time_since_meas(static_cast<float>(l.b.time_since_meas)),
    can_id_a(l.a.can_id),
    can_id_b(l.b.can_id),
    id_a(l.a.id),
    measured(l.a.measured),
    id_b(l.b.id)
  {
  }
  PclPointTarget() {}
};  // SSE padding

}  // namespace off_highway_general_purpose_radar

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(off_highway_general_purpose_radar::PclPointTarget,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, radial_velocity, radial_velocity)
                                  (float, reflected_power, reflected_power)
                                  (float, azimuth_angle_std, azimuth_angle_std)
                                  (float, radial_velocity_std, radial_velocity_std)
                                  (float, radial_distance_std, radial_distance_std)
                                  (float, exist_probability, exist_probability)
                                  (float, time_since_meas, time_since_meas)
                                  (std::uint32_t, can_id_a, can_id_a)
                                  (std::uint32_t, can_id_b, can_id_b)
                                  (std::uint8_t, id_a, id_a)
                                  (std::uint8_t, measured, measured)
                                  (std::uint8_t, id_b, id_b))
/* *INDENT-ON* */
