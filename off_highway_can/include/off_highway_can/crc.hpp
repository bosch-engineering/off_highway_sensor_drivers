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

namespace off_highway_can
{
/**
 * \brief Compute J1850 standard 8-bit cyclic-redundancy checksum for given frame.
 *
 * \param data Frame data payload
 * \param pos_crc Byte position / offset of crc byte
 * \param data_length Frame data length
 * \return J1850 standard 8-bit CRC (initial value 0xFF)
 */
uint8_t calculateCRC(const uint8_t * data, uint8_t pos_crc, uint8_t data_length);
}  // namespace off_highway_can
