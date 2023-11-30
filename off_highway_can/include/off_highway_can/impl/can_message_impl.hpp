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

#include "off_highway_can/crc.hpp"

// Disable warnings from external header, we won't fix FOSS
// TODO(rcp1-beg) Replace CAN encoding / decoding with own implementation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "off_highway_can/external/can_encode_decode_inl.h"
#pragma GCC diagnostic pop

namespace off_highway_can
{
template<typename FrameData>
void Signal::decode(const FrameData & frame)
{
  value = ::decode(frame.data(), start_bit, length, is_big_endian, is_signed, factor, offset);
}

template<typename FrameData>
void Signal::encode(FrameData & frame)
{
  ::encode(frame.data(), value, start_bit, length, is_big_endian, is_signed, factor, offset);
}

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
  for (auto & [_, signal] : signals) {
    signal.encode(frame);
  }
  validate(frame);
}


template<typename FrameData>
void Message::validate(FrameData & frame)
{
  message_counter.increase();
  message_counter.encode(frame);
  frame[crc_index] = calculateCRC(frame.data(), crc_index, frame.size());
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
  if (frame[crc_index] != calculateCRC(frame.data(), crc_index, frame.size())) {
    message_counter.first = true;
    return false;
  }

  return message_counter.decode_and_check(frame);
}
}  // namespace off_highway_can
