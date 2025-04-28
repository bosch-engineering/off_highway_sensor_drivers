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

#include "off_highway_premium_radar/location_data_handler.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "helper.hpp"

namespace off_highway_premium_radar
{

LocationDataHandler::LocationDataHandler()
{
  pdus.reserve(kMaxNumPdu);
}

void LocationDataHandler::handle_pdu(const LocationDataPduRaw & pdu)
{
  uint8_t received_block_counter = pdu.at(kBlockCounterOffset);

  // Check if new and / or first measurement
  bool new_measurement = received_block_counter != block_counter;
  bool first_measurement = block_counter == -1;

  if (new_measurement) {
    if (!first_measurement && !finished()) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("LocationDataHandler::handle_pdu"),
        "New measurement although old one not finished: " <<
          static_cast<int16_t>(received_block_counter) << " != " << block_counter);
    }

    // First PDU of measurement
    pdus.resize(0);
    block_counter = received_block_counter;
    num_locations = read_uint16_be(&pdu.at(kBlockNumberLocsOffset));
    num_pdus = num_locations ?
      std::ceil(
      static_cast<float>(num_locations) /
      LocationDataPdu::kMaxNumLocDataPacketsPerPdu) : 1;
  }

  pdus.emplace_back(pdu);
}

bool LocationDataHandler::finished()
{
  return pdus.size() == num_pdus;
}

LocationData LocationDataHandler::assemble()
{
  if (pdus.empty()) {
    throw std::runtime_error("Called assemble without any PDUs.");
  }

  LocationData data;

  // Take headers from first PDU, they are the same for all PDUs
  LocationDataPdu first_pdu{pdus.front()};
  data.e2e_header = first_pdu.e2e_header;
  data.header = first_pdu.loc_data_header;

  if (!num_locations || !data.header.LocData_DataMeas) {
    // Abort if no locations or measurement invalid
    return data;
  }

  uint16_t valid_locations{0};
  for (const auto & pdu : pdus) {
    LocationDataPdu data_pdu{pdu};

    for (auto & loc_data_packet : data_pdu.loc_data_packets) {
      if ((!loc_data_packet.LocData_MeasStat_i_j) & 1U) {
        // Measured and range check was not passed, skip this location
        continue;
      }

      data.locations.emplace_back(loc_data_packet);
      if (++valid_locations == num_locations) {
        return data;
      }
    }
  }

  return data;
}

}  // namespace off_highway_premium_radar
