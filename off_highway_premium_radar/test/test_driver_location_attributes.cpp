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
    off_highway_premium_radar_msgs::msg::LocationAttributes,
    off_highway_premium_radar::LocationAttributes>
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
    OutputTestClass::SetUp("sensor_location_attributes_subscriber", "/driver/location_attributes");
  }

  void TearDown() override
  {
    OutputTestClass::TearDown();
  }

  template<typename T>
  ::testing::AssertionResult expect_eq_or_sna(const T & sub, const T & ref, bool check_sna = false);

  void verify_sensor_location_attributes(
    off_highway_premium_radar::LocationAttributes ref_sensor_location_attributes,
    off_highway_premium_radar_msgs::msg::LocationAttributes sub_sensor_location_attributes,
    bool check_sna = false);
};

template<typename T>
::testing::AssertionResult TestRadarDriver::expect_eq_or_sna(
  const T & sub, const T & ref, bool check_sna)
{
  if (check_sna && std::isnan(ref)) {
    EXPECT_TRUE(std::isnan(sub));
    if (!std::isnan(sub)) {
      return ::testing::AssertionFailure() << "Expected NaN, got value: " << sub;
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    EXPECT_FLOAT_EQ(sub, ref);
    if (sub != ref) {
      return ::testing::AssertionFailure() << "Expected: " << ref << ", got: " << sub;
    }
  } else {
    EXPECT_EQ(sub, ref);
    if (sub != ref) {
      return ::testing::AssertionFailure() << "Expected: " << ref << ", got: " << sub;
    }
  }
  return ::testing::AssertionSuccess();
}

void TestRadarDriver::verify_sensor_location_attributes(
  off_highway_premium_radar::LocationAttributes ref_sensor_location_attributes,
  off_highway_premium_radar_msgs::msg::LocationAttributes sub_sensor_location_attributes,
  bool check_sna)
{
  // Location Attributes Header
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.lgp_version,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_LgpVer);
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.block_counter,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_BlockCounter);
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.start_measurement.sec,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_TimeSts);
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.start_measurement.nanosec,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_TimeStns);
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.operation_mode.operation_mode,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_OpMode);
  EXPECT_EQ(
    sub_sensor_location_attributes.location_attributes_header.data_measured,
    ref_sensor_location_attributes.loc_atr_header.LocAtr_DataMeas);

  // Location Attributes Packet - Sensor Modulation Performance
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance
      .detection_of_measurement_program,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_DmpID,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance
      .modulation_id,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_ModID,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.distance_range_scaling,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_DistRangScalFact,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.separability_distance,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_SepRadDist,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.separability_relative_velocity,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_SepRadVelo,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.precision_distance,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_PrecRadDist,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.precision_relative_velocity,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_PrecRadVelo,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance
      .covariance_of_distance_and_relative_velocity,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_RadDistVeloCovVar,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.minimum_measurable_distance,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_MinRadDist,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance.maximum_measurable_distance,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_MaxRadDist,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance
      .minimum_measurable_relative_velocity,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_MinRadVelo,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.sensor_modulation_performance
      .maximum_measurable_relative_velocity,
      ref_sensor_location_attributes.loc_atr_packet.sensor_modulation_performance
      .LocAtr_MaxRadVelo,
      check_sna));

  // Location Attributes Packet - Misalignment
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.theta,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_ThetaMalAng,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.theta_variance,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_PhiMalAng,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_variance,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_PhiMalAngVar,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_eme,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_PhiMalAngEme,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_eme_variance,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.status,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MalStatus,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.status_eme,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MalStatusEme,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.percent_negative_theta,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_PercNegativeTheta,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.min_theta_sos,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.max_theta_sos,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.theta_sos_variance,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.theta_sos_mean,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.min_phi_sos,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.max_phi_sos,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_sos_variance,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_sos_mean,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.phi_sos_spread,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.num_sos,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_NumSOs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.num_eme,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_NumEmeLocs,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.misalignment.mal_est_quality,
      ref_sensor_location_attributes.loc_atr_packet.misalignment.LocAtr_MalEstQuality,
      check_sna));

  // Location Attributes Packet - Interference Indicator
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.interference_indicator.fov_reduction_due_to_interfence,
      ref_sensor_location_attributes.loc_atr_packet.interference_indicator.LocAtr_FovRedInt,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes
      .location_attributes_packet.interference_indicator.interference_indicator,
      ref_sensor_location_attributes.loc_atr_packet.interference_indicator.LocAtr_IntStat,
      check_sna));

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i <
    sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view.fov.size();
    ++i)
  {
    EXPECT_TRUE(
      expect_eq_or_sna(
        sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view.fov[i],
        ref_sensor_location_attributes.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i],
        check_sna));
  }
  for (size_t i = 0;
    i <
    sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view
    .azimuth
    .size(); ++i)
  {
    EXPECT_TRUE(
      expect_eq_or_sna(
        sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view.azimuth[i],
        ref_sensor_location_attributes.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i],
        check_sna));
  }
  for (size_t i = 0;
    i <
    sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view
    .elevation_range_scaling
    .size(); ++i)
  {
    EXPECT_TRUE(
      expect_eq_or_sna(
        sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view
        .elevation_range_scaling[i],
        ref_sensor_location_attributes.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i],
        check_sna));
  }
  for (size_t i = 0;
    i <
    sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view.elevation
    .size(); ++i)
  {
    EXPECT_TRUE(
      expect_eq_or_sna(
        sub_sensor_location_attributes.location_attributes_packet.sensor_field_of_view.elevation[i],
        ref_sensor_location_attributes.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i],
        check_sna));
  }

  // Location Attributes Sensor Coating
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .theta_indicator_mimo,
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      static_cast<uint8_t>(sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .theta_indicator_mimo_valid),
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .phi_indicator,
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.mdPhiIndcr,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      static_cast<uint8_t>(sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .phi_indicator_valid),
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .reflections_indicator,
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.nRefIndcr,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      static_cast<uint8_t>(sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .reflections_indicator_valid),
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.nRefIndcrVldFlg,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .theta_mimo_rate,
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.thetaMIMORate,
      check_sna));
  EXPECT_TRUE(
    expect_eq_or_sna(
      static_cast<uint8_t>(sub_sensor_location_attributes.location_attributes_packet.sensor_coating
      .theta_mimo_rate_valid),
      ref_sensor_location_attributes.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag,
      check_sna));

  // Location Attributes Mounting Position
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.x,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenPosX);
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.y,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenPosY);
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.z,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenPosZ);
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.azimuth,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenPosAzi);
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.elevation,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenPosEle);
  EXPECT_FLOAT_EQ(
    sub_sensor_location_attributes.mounting_position.orientation,
    ref_sensor_location_attributes.loc_atr_mounting_position.LocAtr_SenOrient);
}

TEST_F(TestRadarDriver, testSensorLocationAttributesAnyValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  // Location Attributes Header
  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer = 65;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter = 100;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts = 2;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns = 30;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode = 20;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas = 1;

  // Location Attributes Packet - Sensor Modulation Performance
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID = 1;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID = 2;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact = 1.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist = 4.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo = 5.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist = 6.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo = 7.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar = 0.05;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist = 8.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist = 9.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo = -10.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo = 11.0;

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAng = 3.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar = 4.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAng = -0.7;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngVar = 6.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEme = -0.5;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar = 8.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatus = 9;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatusEme = 10;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PercNegativeTheta = 0.1;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs = 0.1;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs = 0.2;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs = 0.3;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs = 0.4;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs = 0.5;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs = 0.6;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs = 0.7;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs = 0.5;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs = -0.3;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumSOs = 11;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumEmeLocs = 12;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalEstQuality = 1;

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt = 0.1;
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat = 2;

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] =
      RandomQuantizedGenerator{
      1.0F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] =
      RandomQuantizedGenerator{
      0.0174533F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] =
      RandomQuantizedGenerator{
      0.01,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] =
      RandomQuantizedGenerator{
      0.0174533F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = 0.1;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = 0.1;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = 0.2;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = 0.3;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 1;

  // Location Attributes Mounting Position
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosX = 0.1;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosY = 0.2;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosZ = 0.3;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosAzi = 0.4;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosEle = 0.5;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenOrient = 1;

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorLocationAttributesSNAValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  // Location Attributes Header
  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer = 65;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter = 100;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts = 2;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns = 30;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode = 20;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas = 1;

  // Location Attributes Packet - Sensor Modulation Performance
  // uint8_t will overflow to 0 when using NAN as SNA (value stated in the TD)
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID = 0xFF;
  // uint16_t will overflow to 0 when using NAN as SNA (value stated in the TD)
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID = 0xFFFF;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo = NAN;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo = NAN;

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAng = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAng = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngVar = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEme = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatus = 0xFFFF;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatusEme = 0xFFFF;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PercNegativeTheta = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs = NAN;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumSOs = 0xFFFF;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumEmeLocs = 0xFFFF;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalEstQuality = NAN;

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt = NAN;
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat = 0xFF;

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] = NAN;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] = NAN;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] = NAN;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] = NAN;
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = NAN;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = NAN;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = NAN;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = NAN;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 1;

  // Location Attributes Mounting Position
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosX = 0.1;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosY = 0.2;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosZ = 0.3;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosAzi = 0.4;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosEle = 0.5;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenOrient = 1;

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data(),
    /*check_sna=*/ true);
}

TEST_F(TestRadarDriver, testSensorLocationAttributesZeroValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas = 0;

  // Location Attributes Packet - Sensor Modulation Performance
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo = 0.0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo = 0.0;

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAng = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAng = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngVar = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEme = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatus = 0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalStatusEme = 0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_PercNegativeTheta = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumSOs = 0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_NumEmeLocs = 0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalEstQuality = 0;

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt = 0.0;
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat = 0;

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] = 0;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] = 0;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] = 0;
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] = 0;
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 0;

  // Location Attributes Mounting Position
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosX = 0.0;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosY = 0.0;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosZ = 0.0;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosAzi = 0.0;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenPosEle = 0.0;
  test_sensor_location_attributes_.loc_atr_mounting_position.LocAtr_SenOrient = 0;

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorLocationAttributesMinValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode = 0;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas = 0;

  // Location Attributes Packet - Sensor Modulation Performance
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo =
    std::numeric_limits<float>::lowest();

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAng = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAng = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngVar = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEme = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatus = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatusEme = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PercNegativeTheta = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumSOs = 0;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumEmeLocs = 0;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalEstQuality =
    std::numeric_limits<float>::lowest();

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt =
    std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat = 0;

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] =
      std::numeric_limits<float>::lowest();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] =
      std::numeric_limits<float>::lowest();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] =
      std::numeric_limits<float>::lowest();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] =
      std::numeric_limits<float>::lowest();
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 0;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate = 0;
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 0;

  // Location Attributes Mounting Position
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosX = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosY = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosZ = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosAzi = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosEle = std::numeric_limits<float>::lowest();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenOrient = -1;

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorLocationAttributesMaxValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer = 65535;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter = 255;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts = 4294967295;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns = 4294967295;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode = 255;
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas = 1;

  // Location Attributes Packet - Sensor Modulation Performance
  // NOTE: Some values below are based on pdu_definitions.hpp instead of TD
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID = 254;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID = 1023;
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo =
    std::numeric_limits<float>::max();

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAng = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAng = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngVar = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEme = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatus = 255;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatusEme = 255;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PercNegativeTheta = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumSOs = 1023;
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumEmeLocs = 1023;
  test_sensor_location_attributes_.loc_atr_packet.misalignment.LocAtr_MalEstQuality =
    std::numeric_limits<float>::max();

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt =
    std::numeric_limits<float>::max();
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat = 2;

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] =
      std::numeric_limits<float>::max();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] =
      std::numeric_limits<float>::max();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] =
      std::numeric_limits<float>::max();
  }
  for (size_t i = 0;
    i < test_sensor_location_attributes_
    .loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    i++)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] =
      std::numeric_limits<float>::max();
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO =
    std::numeric_limits<float>::max();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr =
    std::numeric_limits<float>::max();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr =
    std::numeric_limits<float>::max();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg = 1;
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate =
    std::numeric_limits<float>::max();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag = 1;

  // Location Attributes Mounting Position
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosX = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosY = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosZ = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosAzi = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosEle = std::numeric_limits<float>::max();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenOrient = 1;

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data());
}

TEST_F(TestRadarDriver, testSensorLocationAttributesRandomValues)
{
  std::vector<uint8_t> buffer;
  buffer.resize(off_highway_premium_radar::LocationAttributes::kPduSize);
  off_highway_premium_radar::LocationAttributes test_sensor_location_attributes_ =
    to_pdu<off_highway_premium_radar::LocationAttributes>(buffer);

  test_sensor_location_attributes_.e2e_header.E2E_Counter = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_Crc = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_DataId = 0xFF;
  test_sensor_location_attributes_.e2e_header.E2E_length = 0xFF;

  test_sensor_location_attributes_.loc_atr_header.LocAtr_LgpVer =
    RandomQuantizedGenerator{1, 0, 65535}();
  test_sensor_location_attributes_.loc_atr_header.LocAtr_BlockCounter =
    RandomQuantizedGenerator{1, 0, 255}();
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeSts =
    RandomQuantizedGenerator{1, 0, 4294967295}();
  test_sensor_location_attributes_.loc_atr_header.LocAtr_TimeStns =
    RandomQuantizedGenerator{1, 0, 4294967295}();
  test_sensor_location_attributes_.loc_atr_header.LocAtr_OpMode =
    RandomQuantizedGenerator{1, 0, 255}();
  test_sensor_location_attributes_.loc_atr_header.LocAtr_DataMeas =
    RandomQuantizedGenerator{1, 0, 1}();

  // Location Attributes Packet - Sensor Modulation Performance
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DmpID =
    RandomQuantizedGenerator{1, 0, 254}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_ModID =
    RandomQuantizedGenerator{1, 0, 1023}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_DistRangScalFact =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadDist =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_SepRadVelo =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadDist =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_PrecRadVelo =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_RadDistVeloCovVar =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadDist =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadDist =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MinRadVelo =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.sensor_modulation_performance.LocAtr_MaxRadVelo =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();

  // Location Attributes Packet - Misalignment
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAng =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_ThetaMalAngVar =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAng =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngVar =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEme =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PhiMalAngEmeVar =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatus =
    RandomQuantizedGenerator{1, 0, 255}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalStatusEme =
    RandomQuantizedGenerator{1, 0, 255}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_PercNegativeTheta =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinThetaMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxThetaMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarThetaMalSOs =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanThetaMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MinPhiMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MaxPhiMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_VarPhiMalSOs =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MeanPhiMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_SpreadPhiMalSOs =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumSOs =
    RandomQuantizedGenerator{1, 0, 1023}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_NumEmeLocs =
    RandomQuantizedGenerator{1, 0, 1023}();
  test_sensor_location_attributes_
  .loc_atr_packet.misalignment.LocAtr_MalEstQuality =
    RandomQuantizedGenerator{
    0.000174533F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();

  // Location Attributes Packet - Interference Indicator
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_FovRedInt =
    RandomQuantizedGenerator{
    0.01F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_.loc_atr_packet.interference_indicator.LocAtr_IntStat =
    RandomQuantizedGenerator{1, 0, 2}();

  // Location Attributes Packet - Sensor Field Of View
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_FoVRange[i] =
      RandomQuantizedGenerator{
      1.0F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_AziAngArr[i] =
      RandomQuantizedGenerator{
      0.0174533F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_RangScaEle[i] =
      RandomQuantizedGenerator{
      0.01,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }
  for (size_t i = 0;
    i <
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr.size();
    ++i)
  {
    test_sensor_location_attributes_.loc_atr_packet.sensor_field_of_view.LocAtr_EleAngArr[i] =
      RandomQuantizedGenerator{
      0.0174533F,
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::max()}();
  }

  // Location Attributes Sensor Coating
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMO =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdThetaIndcrMIMOVldFlg =
    RandomQuantizedGenerator{1, 0, 1}();
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcr =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.mdPhiIndcrVldFlg =
    RandomQuantizedGenerator{1, 0, 1}();
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcr =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.nRefIndcrVldFlg =
    RandomQuantizedGenerator{1, 0, 1}();
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORate =
    RandomQuantizedGenerator{
    0.01,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  // can be 0 or 1 due to SensorCoatingPacket.msg deffinition
  test_sensor_location_attributes_.loc_atr_packet.sensor_coating.thetaMIMORteVldFlag =
    RandomQuantizedGenerator{1, 0, 1}();

  // Location Attributes Mounting Position
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosX =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosY =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosZ =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosAzi =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenPosEle =
    RandomQuantizedGenerator{
    0.1F,
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::max()}();
  test_sensor_location_attributes_
  .loc_atr_mounting_position.LocAtr_SenOrient =
    RandomQuantizedGenerator{1, -1, 1}();

  send_pdu_data(test_sensor_location_attributes_);
  verify_sensor_location_attributes(
    test_sensor_location_attributes_,
    get_pdu_data());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
