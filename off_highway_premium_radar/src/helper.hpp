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

#include <endian.h>

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <stdexcept>

namespace off_highway_premium_radar
{

/**
 * \brief Memcpy from raw byte array into desired type
 *
 * \tparam T Type to read from byte buffer
 * \param buffer Byte buffer, needs to hold at least T size (not checked!!!)
 * \return Value of type T copied from byte buffer
 */
template<typename T>
T memcpy_from_bytes(const uint8_t * buffer)
{
  if (!buffer) {
    throw std::runtime_error("Empty buffer in memcpy.");
  }

  T value;
  std::memcpy(&value, buffer, sizeof(T));
  return value;
}

/**
 * \brief Read uint32 from big-endian ordered buffer
 *
 * \param buffer Big-endian ordered buffer, at least four bytes size (not checked!!!)
 * \return Host-ordered uint32
 */
inline uint32_t read_uint32_be(const uint8_t * buffer)
{
  return be32toh(memcpy_from_bytes<uint32_t>(buffer));
}

/**
 * \brief Read uint16 from big-endian ordered buffer
 *
 * \param buffer Big-endian ordered buffer, at least two bytes size (not checked!!!)
 * \return Host-ordered uint16
 */
inline uint16_t read_uint16_be(const uint8_t * buffer)
{
  return be16toh(memcpy_from_bytes<uint16_t>(buffer));
}

// Require standard 32-bit float representation
static_assert(std::numeric_limits<float>::is_iec559, "IEC559 / IEEE754 floating point required!");
static_assert(sizeof(float) == sizeof(uint32_t), "32-bit floating point required!");

/**
 * \brief Convert big-endian 32-bit float to host-ordered 32-bit float
 * \note Requires / assumes IEEE754 floating point representation
 * \param in Big-endian ordered 32-bit float
 * \return Host-ordered 32-bit float
 */
inline float be32tohf(float in)
{
  return std::bit_cast<float>(be32toh(std::bit_cast<uint32_t>(in)));
}

/**
 * \brief Convert host-ordered 32-bit float to big-endian 32-bit float
 * \note Requires / assumes IEEE754 floating point representation
 * \param in Host-ordered 32-bit float
 * \return Big-endian ordered 32-bit float
 */
inline float htobe32f(float in)
{
  return std::bit_cast<float>(htobe32(std::bit_cast<uint32_t>(in)));
}

/**
 * \brief Move vector into array
 *
 * \tparam Size Size of buffer to convert
 * \param buffer Buffer to convert from
 * \return Fixed size array containing raw PDU bytes
 */
template<typename T, size_t N>
std::array<T, N> move_to_array(const std::vector<T> & v)
{
  std::array<T, N> a;

  if (v.size() <= N) {
    std::move(v.begin(), v.end(), a.begin());
  }

  return a;
}

/**
 * \brief Convert buffer to PDU
 *
 * \tparam Pdu PDU to convert to
 * \param buffer Buffer to convert from
 * \return PDU
 */
template<class Pdu>
Pdu to_pdu(const std::vector<uint8_t> & buffer)
{
  return Pdu(move_to_array<uint8_t, Pdu::kPduSize>(std::move(buffer)));
}

}  // namespace off_highway_premium_radar
