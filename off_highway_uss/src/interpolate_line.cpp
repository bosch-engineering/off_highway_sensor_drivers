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

#include "off_highway_uss/interpolate_line.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>

namespace off_highway_uss
{

std::vector<geometry_msgs::msg::Point> interpolate_segment(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second,
  double sample_distance)
{
  if (sample_distance <= 0.) {
    return {first, second};
  }

  double dx = second.x - first.x;
  double dy = second.y - first.y;
  double d = std::sqrt(dx * dx + dy * dy);
  // Always generate both end points
  uint32_t sample_count = std::max(2u, 1 + static_cast<uint32_t>(d / sample_distance));

  std::vector<geometry_msgs::msg::Point> samples(sample_count);

  auto interpolate = [n = 0, N = sample_count, &first, &dx, &dy]() mutable {
      double t = n++ / static_cast<double>(N - 1);
      geometry_msgs::msg::Point sample = first;
      sample.x += t * dx;
      sample.y += t * dy;
      return sample;
    };
  std::generate(samples.begin(), samples.end(), interpolate);

  return samples;
}

}  // namespace off_highway_uss
