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

#include "pcl_conversions/pcl_conversions.h"
#include "off_highway_premium_radar/converters/pcl_radar_point_type.hpp"

class TestRadarDriver
  : public OutputTestClass<
    off_highway_premium_radar_msgs::msg::LocationDataHeader,
    off_highway_premium_radar::LocData_Header_i>
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
    OutputTestClass::SetUp("sensor_locations_subscriber", "/driver/locations_header");

    // Separate promise/future for location data
    location_future_ = location_promise_.get_future();

    sensor_locations_subscription_ =
      sensor_sub_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/driver/locations", 10,
      [&](const sensor_msgs::msg::PointCloud2 msg) {
        pcl::fromROSMsg(msg, received_location_data_);
        location_promise_.set_value(true);
      });
  }

  void TearDown() override
  {
    OutputTestClass::TearDown();
  }

  // Override get_pdu_data to wait for both messages
  void get_pdu_data();
  void send_pdu_data(std::vector<off_highway_premium_radar::LocationDataPdu> pdus);
  void verify_locations(
    std::vector<off_highway_premium_radar::LocationDataPdu> ref_locations,
    bool check_sna = false);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_locations_subscription_;
  pcl::PointCloud<off_highway_premium_radar::PclPointLocation> received_location_data_{};

private:
  // Separate promise/future for location data
  std::promise<bool> location_promise_;
  std::shared_future<bool> location_future_;
};

void TestRadarDriver::get_pdu_data()
{
  // Wait for header message (base class functionality)
  auto header_ret = executor_.spin_until_future_complete(future_, spin_timeout_);
  EXPECT_EQ(header_ret, rclcpp::FutureReturnCode::SUCCESS);

  // Wait for location data message
  auto location_ret = executor_.spin_until_future_complete(location_future_, spin_timeout_);
  EXPECT_EQ(location_ret, rclcpp::FutureReturnCode::SUCCESS);

  // Reset both promises for next test
  promise_ = std::promise<bool>();
  future_ = promise_.get_future();
  location_promise_ = std::promise<bool>();
  location_future_ = location_promise_.get_future();
}

void TestRadarDriver::send_pdu_data(
  std::vector<off_highway_premium_radar::LocationDataPdu> pdus)
{
  for (auto pdu : pdus) {
    auto data = pdu.serialize();
    udp_socket_.send(data);
  }
}

void compareFields(
  const off_highway_premium_radar::PclPointLocation & received,
  const off_highway_premium_radar::LocData_Packet_i_j & ref)
{
  EXPECT_EQ(received.radial_distance, ref.LocData_RadDist_i_j);
  EXPECT_EQ(received.radial_velocity, ref.LocData_RadRelVel_i_j);
  EXPECT_EQ(received.azimuth_angle, ref.LocData_AziAng_i_j);
  EXPECT_EQ(received.elevation_angle, ref.LocData_EleAng_i_j);
  EXPECT_EQ(received.radar_cross_section, ref.LocData_Rcs_i_j);
  EXPECT_EQ(received.signal_noise_ratio, ref.LocData_Snr_i_j);
  EXPECT_EQ(received.radial_distance_variance, ref.LocData_RadDistVar_i_j);
  EXPECT_EQ(received.radial_velocity_variance, ref.LocData_RadRelVelVar_i_j);
  EXPECT_EQ(received.azimuth_angle_variance, ref.LocData_VarAzi_i_j);
  EXPECT_EQ(received.elevation_angle_variance, ref.LocData_VarEle_i_j);
  EXPECT_EQ(received.radial_distance_velocity_covariance, ref.LocData_DistVelCov_i_j);
  EXPECT_EQ(received.velocity_resolution_processing_probability, ref.LocData_ProVelRes_i_j);
  EXPECT_EQ(received.azimuth_angle_probability, ref.LocData_ProAziAng_i_j);
  EXPECT_EQ(received.elevation_angle_probability, ref.LocData_ProEleAng_i_j);
  EXPECT_EQ(received.idx_azimuth_ambiguity_peer, ref.LocData_IdAngAmb_i_j);
  EXPECT_EQ(received.measurement_status, ref.LocData_MeasStat_i_j);
}

void checkNaNFields(
  const off_highway_premium_radar::PclPointLocation & received,
  const off_highway_premium_radar::LocData_Packet_i_j & ref)
{
  EXPECT_TRUE(std::isnan(received.radial_distance));
  EXPECT_TRUE(std::isnan(received.radial_velocity));
  EXPECT_TRUE(std::isnan(received.azimuth_angle));
  EXPECT_TRUE(std::isnan(received.elevation_angle));
  EXPECT_TRUE(std::isnan(received.radar_cross_section));
  EXPECT_TRUE(std::isnan(received.signal_noise_ratio));
  EXPECT_TRUE(std::isnan(received.radial_distance_variance));
  EXPECT_TRUE(std::isnan(received.radial_velocity_variance));
  EXPECT_TRUE(std::isnan(received.azimuth_angle_variance));
  EXPECT_TRUE(std::isnan(received.elevation_angle_variance));
  EXPECT_TRUE(std::isnan(received.radial_distance_velocity_covariance));
  EXPECT_TRUE(std::isnan(received.velocity_resolution_processing_probability));
  EXPECT_TRUE(std::isnan(received.azimuth_angle_probability));
  EXPECT_TRUE(std::isnan(received.elevation_angle_probability));
  EXPECT_EQ(received.idx_azimuth_ambiguity_peer, 0xFFFF);
  EXPECT_EQ(received.measurement_status, ref.LocData_MeasStat_i_j);
}

void TestRadarDriver::verify_locations(
  std::vector<off_highway_premium_radar::LocationDataPdu> ref_locations,
  bool check_sna)
{
  EXPECT_EQ(
    received_msg_.lgp_version,
    ref_locations[0].loc_data_header.LocData_LgpVer_i);
  EXPECT_EQ(
    received_msg_.block_counter,
    ref_locations[0].loc_data_header.LocData_BlockCounter_i);
  EXPECT_EQ(
    received_msg_.operation_mode.operation_mode,
    ref_locations[0].loc_data_header.LocData_OpMode);
  EXPECT_EQ(
    received_msg_.data_measured,
    ref_locations[0].loc_data_header.LocData_DataMeas);
  EXPECT_EQ(
    received_msg_.num_locations,
    ref_locations[0].loc_data_header.LocData_NumLoc);
  EXPECT_EQ(
    received_msg_.start_measurement.sec,
    ref_locations[0].loc_data_header.LocData_TimeSts_i);
  EXPECT_EQ(
    received_msg_.start_measurement.nanosec,
    ref_locations[0].loc_data_header.LocData_TimeStns_i);

  // Check if measurement data is completely invalid
  if (ref_locations[0].loc_data_header.LocData_DataMeas == 0) {
    EXPECT_EQ(received_location_data_.points.size(), 0);
    return;
  }

  // Count expected valid locations (only those with LocData_MeasStat_i_j != 0)
  uint32_t expected_valid_locations = 0;
  for (const auto & ref_location : ref_locations) {
    for (const auto & loc_data_packet : ref_location.loc_data_packets) {
      if (loc_data_packet.LocData_MeasStat_i_j != 0) {
        expected_valid_locations++;
      }
    }
  }
  EXPECT_EQ(received_location_data_.points.size(), expected_valid_locations);

  uint32_t rec_loc_index = 0;
  if (!check_sna) {
    for (const auto & ref_location : ref_locations) {
      for (const auto & loc_data_packet : ref_location.loc_data_packets) {
        // Check if this reference location should be present in received data
        if (loc_data_packet.LocData_MeasStat_i_j != 0) {
          compareFields(received_location_data_[rec_loc_index], loc_data_packet);
          rec_loc_index++;
        }
      }
    }
  } else {
    for (const auto & ref_location : ref_locations) {
      for (const auto & loc_data_packet : ref_location.loc_data_packets) {
        checkNaNFields(received_location_data_[rec_loc_index], loc_data_packet);
        rec_loc_index++;
      }
    }
  }
}

TEST_F(TestRadarDriver, testSensorLocationMinValues)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 16;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst;

    test_location_.e2e_header.E2E_Counter = 0xFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 0;
    test_location_.loc_data_header.LocData_BlockCounter_i = 0;
    test_location_.loc_data_header.LocData_TimeSts_i = 0;
    test_location_.loc_data_header.LocData_TimeStns_i = 0;
    test_location_.loc_data_header.LocData_OpMode = 0;
    test_location_.loc_data_header.LocData_DataMeas = 1;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 0;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_header.LocData_NumLoc; i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_Snr_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j =
        std::numeric_limits<float>::lowest();
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 0;
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 0;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationZeroLocData_DataMeas)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 16;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst;

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 0;
    test_location_.loc_data_header.LocData_BlockCounter_i = 0;
    test_location_.loc_data_header.LocData_TimeSts_i = 0;
    test_location_.loc_data_header.LocData_TimeStns_i = 0;
    test_location_.loc_data_header.LocData_OpMode = 0;
    test_location_.loc_data_header.LocData_DataMeas = 0;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 0;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_header.LocData_NumLoc; i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j = -110.0F;
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = -1.5708F;
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = -0.785398F;
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = -50.0F;
      test_location_.loc_data_packets[i].LocData_Snr_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j = 0;
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = 0;
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = 0;
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j = -0.1F;
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j = 0;
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j = 0;
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j = 0;
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 0;
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 1;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationZeroLocData_MeasStat_i_j)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 16;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst;

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 0;
    test_location_.loc_data_header.LocData_BlockCounter_i = 0;
    test_location_.loc_data_header.LocData_TimeSts_i = 0;
    test_location_.loc_data_header.LocData_TimeStns_i = 0;
    test_location_.loc_data_header.LocData_OpMode = 0;
    test_location_.loc_data_header.LocData_DataMeas = 1;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 0;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_header.LocData_NumLoc; i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j = -110.0F;
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = -1.5708F;
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = -0.785398F;
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = -50.0F;
      test_location_.loc_data_packets[i].LocData_Snr_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j = 0;
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j = 0;
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = 0;
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = 0;
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j = -0.1F;
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j = 0;
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j = 0;
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j = 0;
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 0;
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 0;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationMaxValues)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 1024;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst + i;

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 65535;
    test_location_.loc_data_header.LocData_BlockCounter_i = 255;
    test_location_.loc_data_header.LocData_TimeSts_i = 4294967295;
    test_location_.loc_data_header.LocData_TimeStns_i = 4294967295;
    test_location_.loc_data_header.LocData_OpMode = 255;
    test_location_.loc_data_header.LocData_DataMeas = 1;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 255;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_packets.size(); i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_Snr_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j =
        std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j = std::numeric_limits<float>::max();
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 255u;
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 1023u;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  TestRadarDriver::send_pdu_data(test_locations_);
  TestRadarDriver::get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationValidValues)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 32;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst + i;

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 65530;
    test_location_.loc_data_header.LocData_BlockCounter_i = 250;
    test_location_.loc_data_header.LocData_TimeSts_i = 0xFFFFFFFE;
    test_location_.loc_data_header.LocData_TimeStns_i = 0xFFFFFFFE;
    test_location_.loc_data_header.LocData_OpMode = 240;
    test_location_.loc_data_header.LocData_DataMeas = 1;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 250;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_packets.size(); i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = 301.0F;
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j = 50.0F;
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = 1.3008F;
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = 0.705398F;
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = 50.0F;
      test_location_.loc_data_packets[i].LocData_Snr_i_j = 15.0F;
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j = 0.01F;
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j = 0.1F;
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = 0.0104533F;
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = 0.0104533F;
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j = 0.1F;
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j = 250.0F;
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j = 250.0F;
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j = 250.0F;
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 249u;
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 1020u;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationNANValues)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 32;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id = off_highway_premium_radar::LocationDataPdu::kPacketIdFirst + i;

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    test_location_.loc_data_header.LocData_LgpVer_i = 65535;
    test_location_.loc_data_header.LocData_BlockCounter_i = 255;
    test_location_.loc_data_header.LocData_TimeSts_i = 0xFFFFFFFF;
    test_location_.loc_data_header.LocData_TimeStns_i = 0xFFFFFFFF;
    test_location_.loc_data_header.LocData_OpMode = 20;
    test_location_.loc_data_header.LocData_DataMeas = 1;
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu = 255;
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_packets.size(); i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_AziAng_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_EleAng_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_Rcs_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_Snr_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_VarEle_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j = NAN;
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 0xFFFF;
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j = 0xFFFF;
      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  TestRadarDriver::send_pdu_data(test_locations_);
  TestRadarDriver::get_pdu_data();
  verify_locations(test_locations_, /*check_nan=*/ true);
}

TEST_F(TestRadarDriver, testSensorLocationRandomValidValues)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 16;
  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id =
      static_cast<uint64_t>(
      RandomQuantizedGenerator{
      1.0,
      off_highway_premium_radar::LocationDataPdu::kPacketIdFirst,
      off_highway_premium_radar::LocationDataPdu::kPacketIdLast}());

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    // RandomQuantizedGenerator(resolution, min, max)
    test_location_.loc_data_header.LocData_LgpVer_i =
      RandomQuantizedGenerator{1.0, 0.0, 65535.0}();
    test_location_.loc_data_header.LocData_BlockCounter_i =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_TimeSts_i =
      RandomQuantizedGenerator{1.0, 0.0, 0xFFFFFFFF}();
    test_location_.loc_data_header.LocData_TimeStns_i =
      RandomQuantizedGenerator{1.0, 0.0, 0xFFFFFFFF}();
    test_location_.loc_data_header.LocData_OpMode =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_DataMeas = RandomQuantizedGenerator{1.0, 0.0, 1.0}();
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_header.LocData_NumLoc; i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j =
        RandomQuantizedGenerator{
        0.01,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j =
        RandomQuantizedGenerator{
        0.01,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_AziAng_i_j =
        RandomQuantizedGenerator{
        0.00174533,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_EleAng_i_j =
        RandomQuantizedGenerator{
        0.00174533,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_Rcs_i_j =
        RandomQuantizedGenerator{
        0.2,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_Snr_i_j =
        RandomQuantizedGenerator{
        0.1,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j =
        RandomQuantizedGenerator{
        0.00005,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j =
        RandomQuantizedGenerator{
        0.0001,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j =
        RandomQuantizedGenerator{
        1.745329e-5,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_VarEle_i_j =
        RandomQuantizedGenerator{
        1.7453293e-5,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j =
        RandomQuantizedGenerator{
        0.0001,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();
      test_location_.loc_data_packets[i].LocData_MeasStat_i_j =
        static_cast<uint16_t>(RandomQuantizedGenerator{1.0, 0.0F, 255.0F}());
      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j =
        static_cast<uint16_t>(RandomQuantizedGenerator{1.0, 0.0F, 1023.0F}());

      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

TEST_F(TestRadarDriver, testSensorLocationRandomZeroLocData_MeasStat_i_j)
{
  std::vector<off_highway_premium_radar::LocationDataPdu> test_locations_ = {};

  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationDataPdu::kPduSize);
  off_highway_premium_radar::LocationDataPdu test_location_ =
    to_pdu<off_highway_premium_radar::LocationDataPdu>(buffer);

  uint16_t num_locations = 16;

  // Generate random number of locations that should have LocData_MeasStat_i_j = 0
  RandomQuantizedGenerator zero_count_gen{1.0, 0.0, static_cast<double>(num_locations)};
  size_t num_zero_locations = static_cast<size_t>(zero_count_gen());

  // Vector to track which locations should have LocData_MeasStat_i_j = 0
  std::vector<bool> should_be_zero(num_locations, false);

  // Randomly select which locations should have LocData_MeasStat_i_j = 0
  if (num_zero_locations > 0) {
    RandomQuantizedGenerator location_selector{1.0, 0.0, static_cast<double>(num_locations - 1)};

    // Use a set to ensure unique indices
    std::set<size_t> zero_indices;
    while (zero_indices.size() < num_zero_locations) {
      zero_indices.insert(static_cast<size_t>(location_selector()));
    }

    for (size_t idx : zero_indices) {
      should_be_zero[idx] = true;
    }
  }

  for (size_t i = 0; i < (num_locations / test_location_.kMaxNumLocDataPacketsPerPdu); i++) {
    test_location_.pdu_id =
      static_cast<uint64_t>(
      RandomQuantizedGenerator{
      1.0,
      off_highway_premium_radar::LocationDataPdu::kPacketIdFirst,
      off_highway_premium_radar::LocationDataPdu::kPacketIdLast}());

    test_location_.e2e_header.E2E_Counter = 0xFFFF;
    test_location_.e2e_header.E2E_Crc = 0xFFFF;
    test_location_.e2e_header.E2E_DataId = 0xFFFFFFFF;
    test_location_.e2e_header.E2E_length = 0xFF;

    // RandomQuantizedGenerator(resolution, min, max)
    test_location_.loc_data_header.LocData_LgpVer_i =
      RandomQuantizedGenerator{1.0, 0.0, 65535.0}();
    test_location_.loc_data_header.LocData_BlockCounter_i =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_TimeSts_i =
      RandomQuantizedGenerator{1.0, 0.0, 0xFFFFFFFF}();
    test_location_.loc_data_header.LocData_TimeStns_i =
      RandomQuantizedGenerator{1.0, 0.0, 0xFFFFFFFF}();
    test_location_.loc_data_header.LocData_OpMode =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_DataMeas = RandomQuantizedGenerator{1.0, 0.0, 1.0}();
    test_location_.loc_data_header.LocData_NumLoc = num_locations;
    test_location_.loc_data_header.LocData_MaxLocPerPdu =
      RandomQuantizedGenerator{1.0, 0.0, 255.0}();
    test_location_.loc_data_header.LocData_Reserved_i =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    for (size_t i = 0; i < test_location_.loc_data_header.LocData_NumLoc; i++) {
      test_location_.loc_data_packets[i].LocData_RadDist_i_j =
        RandomQuantizedGenerator{
        0.01,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadRelVel_i_j =
        RandomQuantizedGenerator{
        0.01,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_AziAng_i_j =
        RandomQuantizedGenerator{
        0.00174533,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_EleAng_i_j =
        RandomQuantizedGenerator{
        0.00174533,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_Rcs_i_j =
        RandomQuantizedGenerator{
        0.2,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_Snr_i_j =
        RandomQuantizedGenerator{
        0.1,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadDistVar_i_j =
        RandomQuantizedGenerator{
        0.00005,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_RadRelVelVar_i_j =
        RandomQuantizedGenerator{
        0.0001,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_VarAzi_i_j =
        RandomQuantizedGenerator{
        1.745329e-5,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_VarEle_i_j =
        RandomQuantizedGenerator{
        1.7453293e-5,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_DistVelCov_i_j =
        RandomQuantizedGenerator{
        0.0001,
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max()}();
      test_location_.loc_data_packets[i].LocData_ProVelRes_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();
      test_location_.loc_data_packets[i].LocData_ProAziAng_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();
      test_location_.loc_data_packets[i].LocData_ProEleAng_i_j =
        RandomQuantizedGenerator{1.0, 0.0F, 255.0F}();

      // Set LocData_MeasStat_i_j based on random selection
      if (should_be_zero[i]) {
        test_location_.loc_data_packets[i].LocData_MeasStat_i_j = 0;
      } else {
        test_location_.loc_data_packets[i].LocData_MeasStat_i_j =
          static_cast<uint16_t>(RandomQuantizedGenerator{1.0, 1.0F, 255.0F}());
      }

      test_location_.loc_data_packets[i].LocData_IdAngAmb_i_j =
        static_cast<uint16_t>(RandomQuantizedGenerator{1.0, 0.0F, 1023.0F}());

      test_location_.loc_data_packets[i].LocData_Reserved_i_j =
      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    }
    test_locations_.push_back(test_location_);
  }

  send_pdu_data(test_locations_);
  get_pdu_data();
  verify_locations(test_locations_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
