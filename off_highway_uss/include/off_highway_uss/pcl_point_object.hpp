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

#include "off_highway_uss_msgs/msg/object.hpp"

namespace off_highway_uss
{

/**
 * \brief Object as PCL point
 */
struct EIGEN_ALIGN16 PclPointObject
{
  PCL_ADD_POINT4D;
  float exist_probability{0.};
  uint8_t object_type{0};
  uint8_t id{0};

  /**
   * \brief Construct a new PclPointObject object
   *
   * \param o Object input
   * \param id Object id
   */
  PclPointObject(const off_highway_uss_msgs::msg::Object & o, uint8_t id)
  : x(static_cast<float>(o.position_first.x)),
    y(static_cast<float>(o.position_first.y)),
    z(static_cast<float>(o.position_first.z)),
    exist_probability(static_cast<float>(o.exist_probability)),
    object_type(o.object_type),
    id(id)
  {
  }

  /**
   * \brief Construct a new PclPointObject object
   *
   * \param position Point position
   * \param exist_probability Existence probability of object
   * \param object_type Type of object
   * \param id Object id
   */
  PclPointObject(
    const geometry_msgs::msg::Point & position, double exist_probability,
    uint8_t object_type, uint8_t id)
  : x(static_cast<float>(position.x)),
    y(static_cast<float>(position.y)),
    z(static_cast<float>(position.z)),
    exist_probability(static_cast<float>(exist_probability)),
    object_type(object_type),
    id(id)
  {
  }
  PclPointObject() {}
};  // SSE padding

}  // namespace off_highway_uss

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(off_highway_uss::PclPointObject,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, exist_probability, exist_probability)
                                  (std::uint8_t, object_type, object_type)
                                  (std::uint8_t, id, id))
/* *INDENT-ON* */
