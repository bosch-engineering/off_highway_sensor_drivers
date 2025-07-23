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

#include <future>
#include <random>

#include "helpers/random_generator.hpp"
#include "helpers/service_test_class.hpp"

using namespace std::chrono_literals;

class TestMeasurementProgram : public ServiceTestClass<
    off_highway_premium_radar_msgs::srv::MeasurementProgram,
    off_highway_premium_radar::MeasurementProgram>
{
public:
  TestMeasurementProgram()
  : ServiceTestClass(
      /*sensor_port=*/ 0x76C6,
      /*host_port=*/ 0x76C0,
      /*loopback_ip=*/ "127.0.0.1",
      /*spin_timeout=*/ std::chrono::seconds(1))
  {}

protected:
  void SetUp() override
  {
    ServiceTestClass::SetUp("measurement_program_client", "/driver/set_measurement_program");
  }

  void send_and_verify_measurement_program(uint16_t program_id)
  {
    send_service_request(program_id);
    auto received_data = receive_pdu_data();
    verify_measurement_program_data(program_id, received_data);
  }

  void verify_measurement_program_data(
    uint16_t expected_program_id,
    const off_highway_premium_radar::MeasurementProgram & received_data)
  {
    EXPECT_EQ(
      received_data.pdu_id,
      off_highway_premium_radar::MeasurementProgram::kPduId);
    EXPECT_EQ(
      received_data.pdu_payload_length,
      off_highway_premium_radar::MeasurementProgram::kPduPayloadLength);
    EXPECT_EQ(
      received_data.measurement_program_data.MeasPgm_ID,
      expected_program_id);
  }
};

TEST_F(TestMeasurementProgram, testMeasurementProgramMinValues)
{
  send_and_verify_measurement_program(0);
}

TEST_F(TestMeasurementProgram, testMeasurementProgramAnyValues)
{
  send_and_verify_measurement_program(3);
}

TEST_F(TestMeasurementProgram, testMeasurementProgramMaxValues)
{
  send_and_verify_measurement_program(4);
}

TEST_F(TestMeasurementProgram, testMeasurementProgramRandomValues)
{
  send_and_verify_measurement_program(
    RandomQuantizedGenerator{1, 0, 4}());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
