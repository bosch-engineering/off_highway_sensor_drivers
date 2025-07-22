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

#include "helpers/input_test_class.hpp"
#include "helpers/random_generator.hpp"

class TestRadarDriver
  : public InputTestClass<
    off_highway_premium_radar_msgs::msg::EgoVehicleInput,
    off_highway_premium_radar::EgoVehicleInput>
{
public:
  TestRadarDriver()
  : InputTestClass(
      /*sensor_port=*/ 0x76C6,
      /*host_port=*/ 0x76C0,
      /*loopback_ip=*/ "127.0.0.1",
      /*spin_timeout=*/ std::chrono::seconds(1))
  {}

protected:
  void SetUp() override
  {
    InputTestClass::SetUp("ego_vehicle_data_publisher", "/driver/ego_vehicle_data");
  }

  void TearDown() override
  {
    InputTestClass::TearDown();
  }

  void publish_pdu_data(
    const off_highway_premium_radar_msgs::msg::EgoVehicleInput & msg_data) override;

  void verify_ego_vehicle_data(
    off_highway_premium_radar::VehicleData ref_ego_vehicle_data_,
    off_highway_premium_radar::EgoVehicleInput rec_ref_ego_vehicle_data_);
  auto to_msg(const off_highway_premium_radar::VehicleData & d);

  static constexpr double kDegToRad = std::numbers::pi / 180.0;

  off_highway_premium_radar::VehicleData ref_ego_vehicle_data_;
  off_highway_premium_radar::EgoVehicleInput rec_ego_vehicle_data_;
};

inline
auto TestRadarDriver::to_msg(const off_highway_premium_radar::VehicleData & d)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleData test_data;
  geometry_msgs::msg::TwistWithCovariance velocity;
  velocity.twist.linear.x = d.EgoData_VehSpd;
  // Sensor uses deg/s
  velocity.twist.angular.z = d.EgoData_RelYawRate * kDegToRad;

  velocity.covariance[0] = d.EgoData_VehSpdStdDev * d.EgoData_VehSpdStdDev;

  geometry_msgs::msg::Accel acceleration;
  acceleration.linear.x = d.EgoData_LogAcc;

  test_data.velocity = velocity;
  test_data.acceleration = acceleration;
  return test_data;
}

void TestRadarDriver::publish_pdu_data(
  const off_highway_premium_radar_msgs::msg::EgoVehicleInput & msg_data)
{
  sensor_publisher_->publish(msg_data);
  executor_.spin_some(spin_timeout_);

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::EgoVehicleInput::kPduSize);

  udp_socket_.receive(buffer);

  rec_ego_vehicle_data_ =
    off_highway_premium_radar::to_pdu<off_highway_premium_radar::EgoVehicleInput>(buffer);
}

void TestRadarDriver::verify_ego_vehicle_data(
  off_highway_premium_radar::VehicleData ref_ego_vehicle_data_,
  off_highway_premium_radar::EgoVehicleInput rec_ref_ego_vehicle_data_)
{
  EXPECT_FLOAT_EQ(
    rec_ref_ego_vehicle_data_.vehicle_data.EgoData_RelYawRate,
    ref_ego_vehicle_data_.EgoData_RelYawRate);
  EXPECT_FLOAT_EQ(
    rec_ref_ego_vehicle_data_.vehicle_data.EgoData_VehSpd,
    ref_ego_vehicle_data_.EgoData_VehSpd);
  EXPECT_FLOAT_EQ(
    rec_ref_ego_vehicle_data_.vehicle_data.EgoData_VehSpdStdDev,
    ref_ego_vehicle_data_.EgoData_VehSpdStdDev);
  EXPECT_FLOAT_EQ(
    rec_ref_ego_vehicle_data_.vehicle_data.EgoData_LogAcc,
    ref_ego_vehicle_data_.EgoData_LogAcc);
}

TEST_F(TestRadarDriver, testEgoVehicleInputZeroValues)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleInput test_data;

  ref_ego_vehicle_data_.EgoData_RelYawRate = 0.0f;
  ref_ego_vehicle_data_.EgoData_VehSpd = 0.0f;
  ref_ego_vehicle_data_.EgoData_VehSpdStdDev = 0.0f;
  ref_ego_vehicle_data_.EgoData_LogAcc = 0.0f;

  test_data.vehicle_data = TestRadarDriver::to_msg(ref_ego_vehicle_data_);

  publish_pdu_data(test_data);
  verify_ego_vehicle_data(ref_ego_vehicle_data_, rec_ego_vehicle_data_);
}

TEST_F(TestRadarDriver, testEgoVehicleInputAnyValues)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleInput test_data;

  ref_ego_vehicle_data_.EgoData_RelYawRate = 5.0f;
  ref_ego_vehicle_data_.EgoData_VehSpd = 10.0f;
  ref_ego_vehicle_data_.EgoData_VehSpdStdDev = 15.0f;
  ref_ego_vehicle_data_.EgoData_LogAcc = 20.0f;

  test_data.vehicle_data = TestRadarDriver::to_msg(ref_ego_vehicle_data_);

  publish_pdu_data(test_data);
  verify_ego_vehicle_data(ref_ego_vehicle_data_, rec_ego_vehicle_data_);
}

TEST_F(TestRadarDriver, testEgoVehicleInputMinValues)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleInput test_data;

  ref_ego_vehicle_data_.EgoData_RelYawRate = std::numeric_limits<float>::lowest();
  ref_ego_vehicle_data_.EgoData_VehSpd = std::numeric_limits<float>::lowest();
  ref_ego_vehicle_data_.EgoData_VehSpdStdDev = 0;
  ref_ego_vehicle_data_.EgoData_LogAcc = std::numeric_limits<float>::lowest();

  test_data.vehicle_data = TestRadarDriver::to_msg(ref_ego_vehicle_data_);

  publish_pdu_data(test_data);
  verify_ego_vehicle_data(ref_ego_vehicle_data_, rec_ego_vehicle_data_);
}

TEST_F(TestRadarDriver, testEgoVehicleInputMaxValues)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleInput test_data;

  ref_ego_vehicle_data_.EgoData_RelYawRate = std::numeric_limits<float>::max();
  ref_ego_vehicle_data_.EgoData_VehSpd = std::numeric_limits<float>::max();
  ref_ego_vehicle_data_.EgoData_VehSpdStdDev = std::sqrt(std::numeric_limits<float>::max());
  ref_ego_vehicle_data_.EgoData_LogAcc = std::numeric_limits<float>::max();

  test_data.vehicle_data = TestRadarDriver::to_msg(ref_ego_vehicle_data_);

  publish_pdu_data(test_data);
  verify_ego_vehicle_data(ref_ego_vehicle_data_, rec_ego_vehicle_data_);
}

TEST_F(TestRadarDriver, testEgoVehicleInputRandomValues)
{
  off_highway_premium_radar_msgs::msg::EgoVehicleInput test_data;

  ref_ego_vehicle_data_.EgoData_RelYawRate =
    RandomQuantizedGenerator{
    0.001,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  ref_ego_vehicle_data_.EgoData_VehSpd =
    RandomQuantizedGenerator{
    0.001,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  ref_ego_vehicle_data_.EgoData_VehSpdStdDev =
    RandomQuantizedGenerator{
    0.001,
    std::numeric_limits<float>::min(),
    std::sqrt(std::numeric_limits<float>::max())}();
  ref_ego_vehicle_data_.EgoData_LogAcc =
    RandomQuantizedGenerator{
    0.001,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();

  test_data.vehicle_data = TestRadarDriver::to_msg(ref_ego_vehicle_data_);

  publish_pdu_data(test_data);
  verify_ego_vehicle_data(ref_ego_vehicle_data_, rec_ego_vehicle_data_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
