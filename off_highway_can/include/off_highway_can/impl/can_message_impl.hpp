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

#include <type_traits>

#include "off_highway_can/crc.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"

// Disable warnings from external header, we won't fix FOSS
// TODO(rcp1-beg) Replace CAN encoding / decoding with own implementation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "off_highway_can/external/can_encode_decode_inl.h"
#pragma GCC diagnostic pop

namespace off_highway_can
{

inline uint64_t round_from_physical_value(float physical_value, float factor, float offset)
{
  return (int64_t)std::round((physical_value - offset) / factor);
}

inline void encode_by_round(
  uint8_t * frame, const float value, const uint16_t startbit, const uint16_t length,
  bool is_big_endian, bool is_signed, float factor, float offset)
{
  ::storeSignal(
    frame, round_from_physical_value(value, factor, offset), startbit, length, is_big_endian,
    is_signed);
}

template<typename FrameData>
void Signal::decode(const FrameData & frame)
{
  value = ::decode(frame.data(), start_bit, length, is_big_endian, is_signed, factor, offset);
}

template<>
void Signal::decode(const ros2_socketcan_msgs::msg::FdFrame::_data_type & frame);

template<typename FrameData>
void Signal::encode(FrameData & frame)
{
  encode_by_round(frame.data(), value, start_bit, length, is_big_endian, is_signed, factor, offset);
}

template<>
void Signal::encode(ros2_socketcan_msgs::msg::FdFrame::_data_type & frame);

template<typename FrameData>
bool MessageCounter::decode_and_check(const FrameData & frame_data)
{
  auto last = value;
  decode(frame_data);

  if (first) {
    first = false;
    return true;
  }

  return value > last ? value - last < 2 : last - value > 2;
}

template<typename FrameData>
void Message::encode(FrameData & frame)
{
  if constexpr (std::is_same_v<FrameData, ros2_socketcan_msgs::msg::FdFrame::_data_type>) {
    frame.resize(length);
  }

  for (auto & [_, signal] : signals) {
    signal.encode(frame);
  }
  validate(frame);
}


template<typename FrameData>
void Message::validate(FrameData & frame)
{
  if (message_counter) {
    message_counter->increase();
    message_counter->encode(frame);
  }
  if (crc_index) {
    frame[*crc_index] = calculate_crc(frame);
  }
}


template<typename FrameData>
bool Message::decode(const FrameData & frame)
{
  if (!valid(frame)) {
    return false;
  }

  for (auto & [_, signal] : signals) {
    signal.decode(frame);
  }

  return true;
}

template<typename FrameData>
bool Message::valid(const FrameData & frame)
{
  if (frame.size() != this->length) {
    return false;
  }

  if (crc_index && frame[*crc_index] != calculate_crc(frame)) {
    if (message_counter) {
      message_counter->first = true;
    }
    return false;
  }

  return !message_counter || message_counter->decode_and_check(frame);
}

template<typename FrameData>
uint8_t Message::calculate_crc(const FrameData & frame)
{
  return calculateCRC(frame.data(), *crc_index, frame.size());
}

template<>
uint8_t Message::calculate_crc(const ros2_socketcan_msgs::msg::FdFrame::_data_type & frame);

}  // namespace off_highway_can
