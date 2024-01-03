// Copyright 2022 Robert Bosch GmbH and its subsidiaries
// Copyright 2023 digital workbench GmbH
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

#include "off_highway_radar_msgs/msg/object.hpp"

namespace off_highway_radar
{

/**
 * \brief Object as PCL point
 */
struct EIGEN_ALIGN16 PclPointObject
{
  PCL_ADD_POINT4D;
  float v_x{0.};
  float v_y{0.};
  float time_since_meas{0.};
  float rcs{0.};
  float exist_probability{0.};
  uint32_t can_id_a{0};
  uint32_t can_id_b{0};
  uint8_t id_a{0};
  uint8_t meas{0};
  uint8_t valid{0};
  uint8_t hist{0};
  uint8_t id_b{0};
  uint8_t zone{0};
  uint8_t moving{0};
  uint8_t near{0};

  /**
   * \brief Construct a new PclPointObject object from an object
   *
   * \param o Object input
   */
  explicit PclPointObject(const off_highway_radar_msgs::msg::Object & o)
  : x(static_cast<float>(o.a.position.x)),
    y(static_cast<float>(o.a.position.y)),
    z(0.),
    v_x(static_cast<float>(o.a.velocity.linear.x)),
    v_y(static_cast<float>(o.a.velocity.linear.y)),
    time_since_meas(static_cast<float>(o.b.time_since_meas)),
    rcs(static_cast<float>(o.b.rcs)),
    exist_probability(static_cast<float>(o.b.exist_probability)),
    can_id_a(o.a.can_id),
    can_id_b(o.b.can_id),
    id_a(o.a.id),
    meas(o.a.meas),
    valid(o.a.valid),
    hist(o.a.hist),
    id_b(o.b.id),
    zone(o.b.zone),
    moving(o.b.moving),
    near(o.b.near)
  {
  }
  PclPointObject() {}
};  // SSE padding

}  // namespace off_highway_radar

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(off_highway_radar::PclPointObject,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, v_x, v_x)
                                  (float, v_y, v_y)
                                  (float, time_since_meas, time_since_meas)
                                  (float, rcs, rcs)
                                  (float, exist_probability, exist_probability)
                                  (std::uint32_t, can_id_a, can_id_a)
                                  (std::uint32_t, can_id_b, can_id_b)
                                  (std::uint8_t, id_a, id_a)
                                  (std::uint8_t, meas, meas)
                                  (std::uint8_t, valid, valid)
                                  (std::uint8_t, hist, hist)
                                  (std::uint8_t, id_b, id_b)
                                  (std::uint8_t, zone, zone)
                                  (std::uint8_t, moving, moving)
                                  (std::uint8_t, near, near))
/* *INDENT-ON* */
