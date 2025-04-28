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
#include "off_highway_premium_radar/udp_socket.hpp"

#include <iostream>
#include <utility>
#include <string>
#include <system_error>
#include <vector>

#include "asio.hpp"
#include "rclcpp/logging.hpp"

namespace off_highway_premium_radar
{

UdpSocket::UdpSocket(
  const IoContext & ctx,
  const std::string & remote_ip,
  const uint16_t remote_port,
  const std::string & host_ip,
  const uint16_t host_port)
: udp_socket_(ctx.ios())
{
  remote_endpoint_ = remote_ip.empty() ?
    udp::endpoint{udp::v4(), remote_port} :
  udp::endpoint{address::from_string(remote_ip), remote_port};
  host_endpoint_ = host_ip.empty() ?
    udp::endpoint{udp::v4(), host_port} :
  udp::endpoint{address::from_string(host_ip), host_port};
  source_endpoint_ = udp::endpoint{udp::v4(), 0};
  receive_buffer_.resize(kReceiveBufferSize_);
}

UdpSocket::~UdpSocket()
{
  close();
}

std::size_t UdpSocket::send(std::vector<uint8_t> & buff)
{
  try {
    return udp_socket_.send_to(asio::buffer(buff), remote_endpoint_);
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::send"), error.what());
    return 0;
  }
}

size_t UdpSocket::receive(std::vector<uint8_t> & buff)
{
  asio::error_code error;
  asio::ip::udp::endpoint sender_endpoint;

  std::size_t len = udp_socket_.receive_from(
    asio::buffer(buff),
    host_endpoint_,
    0,
    error);

  if (error && error != asio::error::message_size) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::receive"), error.message());
    return 0;
  }
  return len;
}

void UdpSocket::asyncSend(std::vector<uint8_t> & buff)
{
  udp_socket_.async_send_to(
    asio::buffer(buff), remote_endpoint_,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncSendHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncReceive(UdpSocket::ReceiveCallback func)
{
  receive_callback_ = std::move(func);
  udp_socket_.async_receive_from(
    asio::buffer(receive_buffer_),
    source_endpoint_,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncReceiveHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncSendHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncSendHandler"), error.message());
  }
}

void UdpSocket::asyncReceiveHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncReceiveHandler"), error.message());
    return;
  }

  if (bytes_transferred > 0 && receive_callback_) {
    receive_buffer_.resize(bytes_transferred);
    receive_callback_(receive_buffer_, source_ip(), source_port());
    receive_buffer_.resize(kReceiveBufferSize_);
    udp_socket_.async_receive_from(
      asio::buffer(receive_buffer_),
      source_endpoint_,
      [this](std::error_code error, std::size_t bytes_tf)
      {
        receive_buffer_.resize(bytes_tf);
        asyncReceiveHandler(error, bytes_tf);
      });
  }
}

std::string UdpSocket::remote_ip() const
{
  return remote_endpoint_.address().to_string();
}

uint16_t UdpSocket::remote_port() const
{
  return remote_endpoint_.port();
}

std::string UdpSocket::host_ip() const
{
  return host_endpoint_.address().to_string();
}

uint16_t UdpSocket::host_port() const
{
  return host_endpoint_.port();
}

std::string UdpSocket::source_ip() const
{
  return source_endpoint_.address().to_string();
}

uint16_t UdpSocket::source_port() const
{
  return source_endpoint_.port();
}

void UdpSocket::open()
{
  udp_socket_.open(udp::v4());
  udp_socket_.set_option(udp::socket::reuse_address(true));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("UdpSocket::open"), "Socket is open.");
}

void UdpSocket::close()
{
  asio::error_code error;
  udp_socket_.close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::close"), error.message());
  }
}

bool UdpSocket::isOpen() const
{
  return udp_socket_.is_open();
}

void UdpSocket::bind()
{
  udp_socket_.bind(host_endpoint_);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("UdpSocket::bind"),
    "Socket is bound on " << host_ip() << ":" << host_port() << ".");
}

void UdpSocket::connect()
{
  udp_socket_.connect(remote_endpoint_);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("UdpSocket::connect"),
    "Socket is connected to " << remote_ip() << ":" << remote_port() << ".");
}

}  // namespace off_highway_premium_radar
