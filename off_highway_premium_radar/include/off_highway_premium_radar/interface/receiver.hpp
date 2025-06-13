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

#include <memory>

#include "off_highway_premium_radar/pdu_definitions.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Receiver interface
 */
class Receiver
{
public:
  using SharedPtr = std::shared_ptr<Receiver>;

  /**
   * \brief Destructor
   */
  virtual ~Receiver() = default;

  /**
   * \brief Called from receiving thread on receiving a full location data measurement
   *
   * \param data Location data measurement (in host order)
   */
  virtual void on_location_data(const LocationData &) {}

  /**
   * \brief Called from receiving thread on receiving a sensor information PDU
   *
   * \param data Sensor information PDU (in host order)
   */
  virtual void on_sensor_state_information(const SensorStateInformation &) {}

  /**
   * \brief Called from receiving thread on receiving a location attributes PDU
   *
   * \param data Location attributes PDU (in host order)
   */
  virtual void on_location_attributes(const LocationAttributes &) {}
};

}  // namespace off_highway_premium_radar
