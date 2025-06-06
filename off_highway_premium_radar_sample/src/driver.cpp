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

#include "off_highway_premium_radar_sample/driver.hpp"
#include "helper.hpp"

namespace off_highway_premium_radar_sample
{

Driver::Driver(
  const std::string & host_ip, const int host_port, const std::string & sensor_ip,
  const int sensor_port, bool connect)
: ctx_(1),  // Use single additional thread to sequential process
  udp_socket_(ctx_, sensor_ip, sensor_port, host_ip, host_port)
{
  // Initialize UDP socket
  try {
    udp_socket_.open();
    udp_socket_.bind();
    if (connect) {
      // Established-over-unconnected technique: Connect UDP socket to sensor IP to only receive
      // traffic from it and enable reusing of same port for multiple processes.
      udp_socket_.connect();
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("Driver::Driver"), "Error creating UDP socket: %s:%i - %s",
      udp_socket_.host_ip().c_str(), udp_socket_.host_port(), ex.what());
    throw ex;
  }
}

void Driver::start_receiving()
{
  udp_socket_.asyncReceive(
    std::bind(
      &Driver::callback_udp, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
}

void Driver::callback_udp(
  const std::vector<uint8_t> & buffer,
  const std::string & source_ip, const uint32_t /*source_port*/)
{
  size_t received_bytes = buffer.size();

  // Stop if received bytes smaller than any PDU header
  if (received_bytes < kPduHeaderLength) {
    return;
  }

  // Stop if packet not from sensor
  if (udp_socket_.remote_ip() != source_ip) {
    return;
  }

  switch (get_pdu_type(buffer)) {
    case PduType::kLocationData:
      {
        auto pdu_raw = move_to_array<uint8_t, LocationDataPdu::kPduSize>(buffer);
        location_data_handler_.handle_pdu(pdu_raw);
        if (location_data_handler_.finished()) {
          auto location_data = location_data_handler_.assemble();
          for (auto & r : receivers_) {
            r->on_location_data(location_data);
          }
        }
        break;
      }
    case PduType::kSensorFeedback:
      {
        auto msg = to_pdu<SensorFeedback>(buffer);
        for (auto & r : receivers_) {
          r->on_sensor_feedback(msg);
        }
        break;
      }
    case PduType::kStateInformation:
      {
        auto msg = to_pdu<SensorStateInformation>(buffer);
        for (auto & r : receivers_) {
          r->on_sensor_state_information(msg);
        }
        break;
      }
    case PduType::kSensorBroadcast:
      {
        auto msg = to_pdu<SensorBroadcast>(buffer);
        for (auto & r : receivers_) {
          r->on_sensor_broadcast(msg);
        }
        break;
      }
    case PduType::kLocationAttributes:
      {
        auto msg = to_pdu<LocationAttributes>(buffer);
        for (auto & r : receivers_) {
          r->on_location_attributes(msg);
        }
        break;
      }
    case PduType::kSensorDtcInformation:
      {
        auto msg = to_pdu<SensorDTCInformation>(buffer);
        for (auto & r : receivers_) {
          r->on_sensor_dtc_information(msg);
        }
        break;
      }
    case PduType::kUnknown:
      break;
  }
}

Driver::PduType Driver::get_pdu_type(std::vector<uint8_t> buffer)
{
  auto pdu_size = buffer.size();

  if (pdu_size < std::max(kPduIdOffset, kPduPayloadLengthOffset)) {
    return PduType::kUnknown;
  }

  const uint32_t id = read_uint32_be(&buffer.at(kPduIdOffset));
  const uint32_t length = read_uint32_be(&buffer.at(kPduPayloadLengthOffset));

  if (id >= LocationDataPdu::kPacketIdFirst &&
    id <= LocationDataPdu::kPacketIdLast &&
    length == LocationDataPdu::kPduPayloadLength)
  {
    return PduType::kLocationData;
  }
  if (id == SensorFeedback::kPduId && length == SensorFeedback::kPduPayloadLength) {
    return PduType::kSensorFeedback;
  }
  if (id == SensorStateInformation::kPduId && length == SensorStateInformation::kPduPayloadLength) {
    return PduType::kStateInformation;
  }
  if (id == SensorBroadcast::kPduId && length == SensorBroadcast::kPduPayloadLength) {
    return PduType::kSensorBroadcast;
  }
  if (id == LocationAttributes::kPduId && length == LocationAttributes::kPduPayloadLength) {
    return PduType::kLocationAttributes;
  }
  if (id == SensorDTCInformation::kPduId && length == SensorDTCInformation::kPduPayloadLength) {
    return PduType::kSensorDtcInformation;
  }

  return PduType::kUnknown;
}

}  // namespace off_highway_premium_radar_sample
