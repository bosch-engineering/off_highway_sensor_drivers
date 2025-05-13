// Copyright 2021 LeoDrive.
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

// Developed by LeoDrive, 2021

// Modifications:
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

#include <array>
#include <string>
#include <vector>

#include "io_context/io_context.hpp"
#include "msg_converters/converters.hpp"

using asio::ip::udp;
using asio::ip::address;
using drivers::common::IoContext;

namespace off_highway_premium_radar_sample
{

/**
 * \brief Wrapper around ASIO UDP socket
 */
class UdpSocket
{
public:
  using ReceiveCallback = std::function<void(const std::vector<uint8_t> &,
      const std::string & remote_ip, const uint32_t remote_port)>;

  /**
   * \brief Construct a new UdpSocket object
   *
   * \param ctx Threading context
   * \param remote_ip Remote IP to receive / send data from / to
   * \param remote_port Remote port
   * \param host_ip Host IP (empty to use any interface)
   * \param host_port Host port
   */
  UdpSocket(
    const IoContext & ctx,
    const std::string & remote_ip, uint16_t remote_port,
    const std::string & host_ip, uint16_t host_port);

  /**
   * \brief Destroy the UdpSocket object
   */
  ~UdpSocket();

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;

  std::string remote_ip() const;
  uint16_t remote_port() const;
  std::string host_ip() const;
  uint16_t host_port() const;
  std::string source_ip() const;
  uint16_t source_port() const;

  void open();
  void close();
  bool isOpen() const;
  void bind();
  void connect();

  /**
   * \brief Blocking send operation
   */
  std::size_t send(std::vector<uint8_t> & buff);

  /**
   * \brief Blocking receive operation
   */
  size_t receive(std::vector<uint8_t> & buff);

  /**
   * \brief Non-blocking send operation
   */
  void asyncSend(std::vector<uint8_t> & buff);

  /**
   * \brief Non-blocking receive operation
   */
  void asyncReceive(ReceiveCallback func);

private:
  /**
   * \brief Handler for asynchronous sending
   */
  void asyncSendHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred);

  /**
   * \brief Handler to recursively retrigger asynchronous receive, resize buffer and trigger
   * callback
   */
  void asyncReceiveHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred);

private:
  //! Actual UDP socket (ASIO wrapper around BSD socket)
  udp::socket udp_socket_;
  //! For sending packets
  udp::endpoint remote_endpoint_;
  //! For binding our socket
  udp::endpoint host_endpoint_;
  //! For receiving packets
  udp::endpoint source_endpoint_;
  //! Callback on receive of packets
  ReceiveCallback receive_callback_;
  static const size_t kReceiveBufferSize_{2048};
  //! Receive buffer
  std::vector<uint8_t> receive_buffer_;
};

}  // namespace off_highway_premium_radar_sample
