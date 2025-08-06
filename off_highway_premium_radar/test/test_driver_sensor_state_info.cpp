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

#include "helpers/output_test_class.hpp"
#include "helpers/random_generator.hpp"

class TestRadarDriver
  : public OutputTestClass<
    off_highway_premium_radar_msgs::msg::SensorStateInformation,
    off_highway_premium_radar::SensorStateInformation>
{
public:
  TestRadarDriver()
  : OutputTestClass(
      /*sensor_port=*/ 0x76C6,
      /*host_port=*/ 0x76C0,
      /*loopback_ip=*/ "127.0.0.1",
      /*spin_timeout=*/ std::chrono::seconds(1))
  {}

protected:
  void SetUp() override
  {
    OutputTestClass::SetUp("sensor_state_info_subscriber", "/driver/sensor_state_information");
  }

  void TearDown() override
  {
    OutputTestClass::TearDown();
  }

  void verify_sensor_state_info(
    off_highway_premium_radar::SensorStateInformation ref_sensor_state_info,
    off_highway_premium_radar_msgs::msg::SensorStateInformation sub_sensor_state_info);
};

void TestRadarDriver::verify_sensor_state_info(
  off_highway_premium_radar::SensorStateInformation ref_sensor_state_info,
  off_highway_premium_radar_msgs::msg::SensorStateInformation sub_sensor_state_info)
{
  EXPECT_EQ(
    sub_sensor_state_info.sensor_state,
    ref_sensor_state_info.sensor_state_data.SenStInfo_SenSt);
}

TEST_F(TestRadarDriver, testSensorStateInformationAnyValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);

  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;

  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt = 101;

  send_pdu_data(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorStateInformationMinValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);

  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;

  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt = 1;

  send_pdu_data(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorStateInformationMaxValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);

  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;

  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt = 255;

  send_pdu_data(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorStateInformationRandomValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::SensorStateInformation::kPduSize);
  off_highway_premium_radar::SensorStateInformation test_sensor_state_information_ =
    to_pdu<off_highway_premium_radar::SensorStateInformation>(buffer);

  test_sensor_state_information_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_state_information_.e2e_header.E2E_length = 0xFF;

  test_sensor_state_information_.sensor_state_data.SenStInfo_SenSt =
    RandomQuantizedGenerator{1, 1, 255}();

  send_pdu_data(test_sensor_state_information_);
  verify_sensor_state_info(test_sensor_state_information_, get_pdu_data());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
