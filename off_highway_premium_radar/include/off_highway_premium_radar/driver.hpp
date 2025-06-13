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

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "io_context/io_context.hpp"

#include "off_highway_premium_radar/location_data_handler.hpp"
#include "off_highway_premium_radar/pdu_definitions.hpp"
#include "off_highway_premium_radar/udp_socket.hpp"
#include "off_highway_premium_radar/interface/receiver.hpp"
#include "off_highway_premium_radar/interface/sender.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Driver class to receive and send to / from sensor
 *
 * Provides sender interface to send data.
 */
class Driver : public Sender
{
public:
  using Receivers = std::vector<Receiver::SharedPtr>;

  /**
   * \brief Construct a new Driver object
   *
   * \param host_ip Host IP
   * \param host_port Host port
   * \param sensor_ip Sensor IP
   * \param sensor_port Sensor port
   * \param connect Connect UDP socket to receive only from sensor (won't receive broadcast msgs)
   */
  explicit Driver(
    const std::string & host_ip, const int host_port, const std::string & sensor_ip,
    const int sensor_port, bool connect);

  /**
   * \brief Destroy the Driver object
   */
  virtual ~Driver() = default;

  /**
   * \brief Start receiving packets
   */
  void start_receiving();

  /**
   * \brief Register new receiver to trigger on new packets
   *
   * \param receiver Receiver to register
   */
  void register_receiver(Receiver::SharedPtr receiver) {receivers_.emplace_back(receiver);}

  /**
   * \brief Send ego vehicle data
   *
   * \param data Ego vehicle data
   */
  bool send_ego_vehicle_data(EgoVehicleInput & data) override {return send(data);}

  /**
   * \brief Send measurement program
   *
   * \param data Measurement program
   */
  bool send_measurement_program(MeasurementProgram & data) override {return send(data);}

private:
  /**
   * \brief Callback that is called on receiving an UDP packet
   *
   * \param buffer
   * \param source_ip
   * \param source_port
   */
  void callback_udp(
    const std::vector<uint8_t> & buffer, const std::string & source_ip,
    const uint32_t source_port);

  /**
   * \brief Serialize data into byte buffer and send it to sensor
   *
   * \tparam Input Type of data to send
   * \param data Data to send
   * \return True if all bytes were sent, false otherwise
   */
  template<class Input>
  bool send(Input data)
  {
    auto buffer = data.serialize();
    // TODO(rcp1-beg) Really check if complete buffer was sent?
    return udp_socket_.send(buffer) == buffer.size();
  }

  /**
   * \brief PDU types of sensor messages
   */
  enum class PduType
  {
    kLocationData,
    kStateInformation,
    kLocationAttributes,
    kUnknown
  };

  /**
   * \brief Get the PDU type from received buffer
   *
   * \param buffer Buffer
   * \return PDU type from buffer
   */
  PduType get_pdu_type(std::vector<uint8_t> buffer);

  // Socket handling
  //! IO Context for threading
  IoContext ctx_;
  //! UDP socket
  UdpSocket udp_socket_;

  //! Handler to assemble location data PDUs
  LocationDataHandler location_data_handler_;

  //! Receivers process received data
  Receivers receivers_;
};

}  // namespace off_highway_premium_radar
