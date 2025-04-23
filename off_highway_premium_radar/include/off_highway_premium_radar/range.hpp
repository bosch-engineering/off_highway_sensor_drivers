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

#include <cmath>
#include <cstdint>
#include <limits>

namespace off_highway_premium_radar
{

/**
 * \brief Range of signal for range checks
 *
 * \tparam T Type of signal
 */
template<typename T>
struct Range
{
  //! Physical minimum of signal
  T physical_min;
  //! Physical maximum of signal
  T physical_max;
  //! Signal not available value for signal
  T sna;  // Signal Not Available

  /**
   * \brief Check signal for range
   *
   * \param value Signal value to check
   * \return Value if in range and not NaN, SNA otherwise
   */
  T check(const T & value) const
  {
    bool invalid = std::isnan(value) || value > physical_max || value < physical_min;

    return invalid ? sna : value;
  }
};

#define CHECK_SIGNAL(SIGNAL) (SIGNAL = r_ ## SIGNAL.check(SIGNAL))

}  // namespace off_highway_premium_radar
