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

#include <array>
#include <cstdint>
#include <vector>

#include "off_highway_premium_radar/pdu_definitions.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Location data handler to assemble split Location packet stream into full measurement
 *
 * It will store each packet in binary form till all packets were received. Assemble will convert
 * all stored PDUs to a list of locations.
 */
class LocationDataHandler
{
public:
  using LocationDataPduRaw = std::array<uint8_t, LocationDataPdu::kPduSize>;
  /**
   * \brief Construct a new LocationDataHandler object
   */
  LocationDataHandler();

  /**
   * \brief Handle PDU
   *
   * If the PDU is a new measurement (based on different block counter), reset internal buffer and
   * set expected number of locations and PDUs.
   *
   * Either way append the PDU to the internal storage.
   *
   * \param buffer Location PDU as byte buffer
   */
  void handle_pdu(const LocationDataPduRaw & buffer);

  /**
   * \brief True if all location data packets from one measurement received, false otherwise
   */
  bool finished();

  /**
   * \brief Process and assemble received raw PDUs to location data
   *
   * \return Assembled location data containing header and all received locations
   */
  LocationData assemble();

  static constexpr uint8_t kMaxNumPdu{64U};
  static constexpr uint8_t kBlockCounterOffset{24U};
  static constexpr uint8_t kBlockNumberLocsOffset{35U};

private:
  //! Count of current PDU for this measurement
  int16_t block_counter{-1};
  //! Number of PDUs for this measurement
  uint16_t num_pdus{1};
  //! Number of locations in this measurement
  uint16_t num_locations{0};
  //! Raw binary measurement data
  std::vector<LocationDataPduRaw> pdus;
};

}  // namespace off_highway_premium_radar
