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

#include <memory>

#include "off_highway_premium_radar_sample/pdu_definitions.hpp"

namespace off_highway_premium_radar_sample
{

/**
 * \brief Sender interface
 */
class Sender
{
public:
  using SharedPtr = std::shared_ptr<Sender>;

  /**
   * \brief Destructor
   */
  virtual
  ~Sender() = default;

  /**
   * \brief Send ego vehicle data
   *
   * \param data Ego vehicle data
   */
  virtual bool send_ego_vehicle_data(EgoVehicleInput & data) = 0;

  /**
   * \brief Send measurement cycle synchronization
   *
   * \param data Measurement cycle synchronization
   */
  virtual bool send_measurement_cycle_sync(MeasurementCycleSynchronisation & data) = 0;

  /**
   * \brief Send sensor mode request
   *
   * \param data Sensor mode request
   */
  virtual bool send_sensor_mode_request(SensorModeRequest & data) = 0;

  /**
   * \brief Send measurement program
   *
   * \param data Measurement program
   */
  virtual bool send_measurement_program(MeasurementProgram & data) = 0;
};

}  // namespace off_highway_premium_radar_sample
