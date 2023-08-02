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

#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace off_highway_uss
{

/**
 * \brief Generate equidistant points on segment between two points.
 *
 * \param first Start of segment
 * \param second End of segment
 * \param sample_distance Distance between consecutive samples of segment
 * \return Equidistant points on segment between given points if sample distance > 0, else first and
 * second.
 */
std::vector<geometry_msgs::msg::Point> interpolate_segment(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second,
  double sample_distance);

}  // namespace off_highway_uss
