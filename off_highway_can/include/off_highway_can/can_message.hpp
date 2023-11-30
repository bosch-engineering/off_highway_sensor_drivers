// Copyright 2022 Robert Bosch GmbH and its subsidiaries
// Copyright 2023 digital workbench GmbH
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
#include <unordered_map>

namespace off_highway_can
{

/**
 * \brief CAN signal storing necessary information for en-/decoding
 */
struct Signal
{
  uint16_t start_bit{0};
  uint16_t length{0};
  bool is_big_endian{false};
  bool is_signed{false};
  double factor{1.};
  double offset{0.};
  double min{0.};
  double max{0.};
  double value{0.};

  bool set(double value, const std::string & signal_name);

  template<typename FrameData>
  void decode(const FrameData & frame);

  template<typename FrameData>
  void encode(FrameData & frame);
};

/**
 * \brief Message counter CAN signal to allow modular arithmetic with check
 */
struct MessageCounter : Signal
{
  bool first{true};

  /**
   * \brief Increase message counter with modular arithmetic (wrap-around at signal length)
   */
  void increase();

  /**
   * \brief Decode and check with last value before decoding (alters value!)
   * \param frame Frame byte array.
   */
  template<typename FrameData>
  bool decode_and_check(const FrameData & frame);
};

/**
 * \brief CAN message with message counter, CRC byte and signals
 */
struct Message
{
  std::string name;
  uint8_t length{8};
  uint8_t crc_index{7};
  MessageCounter message_counter;
  std::unordered_map<std::string, Signal> signals;

  /**
   * \brief Encode signals into frame bits.
   * \param frame Frame byte array
   */
  template<typename FrameData>
  void encode(FrameData & frame);

  /**
   * \brief Validate frame by setting CRC and increasing message counter.
   * \param frame Frame byte array
   */
  template<typename FrameData>
  void validate(FrameData & frame);

  /**
   * \brief Decode frame bits into signals.
   * \param frame Frame byte array
   */
  template<typename FrameData>
  bool decode(const FrameData & frame);

  /**
   * \brief Check frame by comparing CRC and message counter difference.
   * \param frame Frame byte array
   */
  template<typename FrameData>
  bool valid(const FrameData & frame);
};

}  // namespace off_highway_can

#include "impl/can_message_impl.hpp"
