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

#include <array>
#include <cstdint>
#include <limits>
#include <utility>

#include "off_highway_can/can_message.hpp"

#include "gtest/gtest.h"

using off_highway_can::Signal;

class SignalTest : public testing::TestWithParam<Signal> {};

TEST_P(SignalTest, testEncodeDecode)
{
  constexpr size_t kCanFdMaxLength = 64;
  std::array<uint8_t, kCanFdMaxLength> frame;

  Signal s = GetParam();
  auto expected = s.value;
  s.encode(frame);

  s.value = std::numeric_limits<double>::max();
  s.decode(frame);

  EXPECT_NEAR(expected, s.value, 0.5 * s.factor);
}

INSTANTIATE_TEST_SUITE_P(
  SignalTestSuite,
  SignalTest,
  testing::Values(
    Signal{0, 7, false, true, 1.0, 0.0, 0.0, 0.0, -1.0},
    Signal{0, 7, false, true, 100, 0.0, 0.0, 0.0, -100.0},
    Signal{56, 3, false, false, 1.000000, 0.0, 0.0, 0.0, 3.0},
    Signal{2, 6, true, false, 1.0, 0, 0.0, 0.0, 35},
    Signal{0, 2, true, false, 1.0, 0, 0.0, 0.0, 1.0},
    Signal{62, 24, true, true, 0.1, 20, 0.0, 0.0, 1.0},
    Signal{0, 12, false, true, 0.15, -102.3, 0.0, 0.0, 70.6},
    Signal{0, 11, false, true, 0.2, -102.4, 0.0, 0.0, 70},
    Signal{0, 16, false, false, 0.01, -163.84, 0.0, 0.0, 55},
    Signal{0, 12, false, false, 0.1, -102.4, 0.0, 0.0, 90},
    Signal{0, 12, false, true, 0.0001, -0.1024, 0.0, 0.0, 0.1023},
    Signal{0, 10, false, false, 0.02, -5.12, 0.0, 0.0, 5.1},
    Signal{0, 12, false, true, 0.15, -102.3, 0.0, 0.0, -70.6},
    Signal{0, 11, false, true, 0.2, -102.4, 0.0, 0.0, -50},
    Signal{11, 16, false, true, 0.01, -163.84, 0.0, 0.0, -110},
    Signal{0, 12, false, true, 0.1, -102.4, 0.0, 0.0, -90},
    Signal{0, 12, false, true, 0.0001, -0.1024, 0.0, 0.0, -0.1024},
    Signal{5, 10, false, true, 0.02, -5.12, 0.0, 0.0, -5.12}
  )
);

class EncodeByRoundTest : public testing::TestWithParam<std::pair<Signal, uint64_t>> {};

TEST_P(EncodeByRoundTest, testEncodeByRound)
{
  auto [signal, expected] = GetParam();

  uint64_t value = off_highway_can::round_from_physical_value(
    signal.value, signal.factor, signal.offset);

  EXPECT_EQ(expected, value);
}

INSTANTIATE_TEST_SUITE_P(
  SignalRoundingTestSuite,
  EncodeByRoundTest,
  testing::Values(
    std::pair(Signal{0, 12, false, true, 1.0, 0.0, 0.0, 0.0, -50.6}, -51.0),
    std::pair(Signal{0, 12, false, true, 1.0, 0.0, 0.0, 0.0, -50.5}, -51.0),
    std::pair(Signal{0, 12, false, true, 1.0, 0.0, 0.0, 0.0, -50.4}, -50.0),
    std::pair(Signal{0, 12, false, true, 1.5, 0.0, 0.0, 0.0, -15.0}, -10.0),
    std::pair(Signal{0, 12, false, true, 1.5, 0.0, 0.0, 0.0, -20.1}, -13.0),
    std::pair(Signal{0, 12, false, true, 1.5, 0.0, 0.0, 0.0, -10.7}, -7.0),
    std::pair(Signal{0, 12, false, true, 0.1, 0.0, 0.0, 0.0, 2.94}, 29.0),
    std::pair(Signal{0, 12, false, true, 0.1, 0.0, 0.0, 0.0, 2.95}, 30.0),
    std::pair(Signal{0, 12, false, true, 0.1, 0.0, 0.0, 0.0, 2.96}, 30.0),
    std::pair(Signal{0, 12, false, false, 2.0, 0.0, 0.0, 0.0, 105.0}, 53.0),
    std::pair(Signal{0, 12, false, false, 2.0, 0.0, 0.0, 0.0, 106.0}, 53.0),
    std::pair(Signal{0, 12, false, false, 2.0, 0.0, 0.0, 0.0, 107.0}, 54.0),
    std::pair(Signal{0, 32, false, false, 0.0001, 0.0, 0.0, 0.0, 105.00004}, 1050000.0),
    std::pair(Signal{0, 32, false, false, 0.0001, 0.0, 0.0, 0.0, 106.00005}, 1060001.0),
    std::pair(Signal{0, 32, false, false, 0.0001, 0.0, 0.0, 0.0, 107.00006}, 1070001.0),
    std::pair(Signal{0, 32, false, false, 0.1, 0.0, 0.0, 0.0, 0.94}, 9.0),
    std::pair(Signal{0, 32, false, false, 0.1, 0.0, 0.0, 0.0, 0.96}, 10.0)
  )
);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
