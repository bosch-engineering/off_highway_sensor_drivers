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

#pragma once

#include <numbers>
#include <cmath>
#include <limits>
#include <string>

#include "asio.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

#include "off_highway_premium_radar/pdu_definitions.hpp"

#include "off_highway_premium_radar_msgs/msg/ego_vehicle_data.hpp"
#include "off_highway_premium_radar_msgs/msg/interference_indicator.hpp"
#include "off_highway_premium_radar_msgs/msg/location_attributes_header.hpp"
#include "off_highway_premium_radar_msgs/msg/location_attributes_packet.hpp"
#include "off_highway_premium_radar_msgs/msg/location_attributes.hpp"
#include "off_highway_premium_radar_msgs/msg/location_data_header.hpp"
#include "off_highway_premium_radar_msgs/msg/misalignment_packet.hpp"
#include "off_highway_premium_radar_msgs/msg/operation_mode.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_coating_packet.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_field_of_view.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_modulation_performance.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_mounting.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_state_information.hpp"
#include "off_highway_premium_radar_msgs/msg/time.hpp"

#include "off_highway_premium_radar_msgs/msg/ego_vehicle_input.hpp"
#include "off_highway_premium_radar_msgs/srv/measurement_program.hpp"

namespace off_highway_premium_radar
{

using namespace off_highway_premium_radar_msgs;  // NOLINT

static constexpr double kDegToRad = std::numbers::pi / 180.0;
static constexpr double kRadToDeg = 1.0 / kDegToRad;

// To ROS message
inline
auto to_msg(uint32_t sec, uint32_t nanosec)
{
  return build<msg::Time>()
         .sec(sec)
         .nanosec(nanosec);
}

inline
auto to_msg(const LocData_Header_i & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  return build<msg::LocationDataHeader>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .start_measurement(to_msg(d.LocData_TimeSts_i, d.LocData_TimeStns_i))
         .lgp_version(d.LocData_LgpVer_i)
         .block_counter(d.LocData_BlockCounter_i)
         .operation_mode(build<msg::OperationMode>().operation_mode(d.LocData_OpMode))
         .data_measured(d.LocData_DataMeas)
         .num_locations(d.LocData_NumLoc);
  // LocData_MaxLocPerPdu is fixed value and thus irrelevant
}

inline
auto to_msg(
  const SensorStateInformation & d, const rclcpp::Time stamp,
  const std::string & frame_id)
{
  return build<msg::SensorStateInformation>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .sensor_state(d.sensor_state_data.SenStInfo_SenSt);
}

inline
auto to_msg(const LocAtr_Header_i & d)
{
  return build<msg::LocationAttributesHeader>()
         .lgp_version(d.LocAtr_LgpVer)
         .block_counter(d.LocAtr_BlockCounter)
         .start_measurement(to_msg(d.LocAtr_TimeSts, d.LocAtr_TimeStns))
         .operation_mode(build<msg::OperationMode>().operation_mode(d.LocAtr_OpMode))
         .data_measured(d.LocAtr_DataMeas);
}

inline
auto to_msg(const SensorModulationPerformance & d)
{
  return build<msg::SensorModulationPerformance>()
         .detection_of_measurement_program(d.LocAtr_DmpID)
         .modulation_id(d.LocAtr_ModID)
         .distance_range_scaling(d.LocAtr_DistRangScalFact)
         .separability_distance(d.LocAtr_SepRadDist)
         .separability_relative_velocity(d.LocAtr_SepRadVelo)
         .precision_distance(d.LocAtr_PrecRadDist)
         .precision_relative_velocity(d.LocAtr_PrecRadVelo)
         .covariance_of_distance_and_relative_velocity(d.LocAtr_RadDistVeloCovVar)
         .minimum_measurable_distance(d.LocAtr_MinRadDist)
         .maximum_measurable_distance(d.LocAtr_MaxRadDist)
         .minimum_measurable_relative_velocity(d.LocAtr_MinRadVelo)
         .maximum_measurable_relative_velocity(d.LocAtr_MaxRadVelo);
}

inline
auto to_msg(const Misalignment & d)
{
  return build<msg::MisalignmentPacket>()
         .theta(d.LocAtr_ThetaMalAng)
         .theta_variance(d.LocAtr_ThetaMalAngVar)
         .phi(d.LocAtr_PhiMalAng)
         .phi_variance(d.LocAtr_PhiMalAngVar)
         .phi_eme(d.LocAtr_PhiMalAngEme)
         .phi_eme_variance(d.LocAtr_PhiMalAngEmeVar)
         .status(d.LocAtr_MalStatus)
         .status_eme(d.LocAtr_MalStatusEme)
         .percent_negative_theta(d.LocAtr_PercNegativeTheta)
         .min_theta_sos(d.LocAtr_MinThetaMalSOs)
         .max_theta_sos(d.LocAtr_MaxThetaMalSOs)
         .theta_sos_variance(d.LocAtr_VarThetaMalSOs)
         .theta_sos_mean(d.LocAtr_MeanThetaMalSOs)
         .min_phi_sos(d.LocAtr_MinPhiMalSOs)
         .max_phi_sos(d.LocAtr_MaxPhiMalSOs)
         .phi_sos_variance(d.LocAtr_VarPhiMalSOs)
         .phi_sos_mean(d.LocAtr_MeanPhiMalSOs)
         .phi_sos_spread(d.LocAtr_SpreadPhiMalSOs)
         .num_sos(d.LocAtr_NumSOs)
         .num_eme(d.LocAtr_NumEmeLocs)
         .mal_est_quality(d.LocAtr_MalEstQuality);
}

inline
auto to_msg(const InterferenceIndicator & d)
{
  return build<msg::InterferenceIndicator>()
         .fov_reduction_due_to_interfence(d.LocAtr_FovRedInt)
         .interference_indicator(d.LocAtr_IntStat);
}

inline
auto to_msg(const SensorFieldOfView & d)
{
  return build<msg::SensorFieldOfView>()
         .fov(d.LocAtr_FoVRange)
         .azimuth(d.LocAtr_AziAngArr)
         .elevation_range_scaling(d.LocAtr_RangScaEle)
         .elevation(d.LocAtr_EleAngArr);
}

inline
auto to_msg(const SensorCoating & d)
{
  return build<msg::SensorCoatingPacket>()
         .theta_indicator_mimo(d.mdThetaIndcrMIMO)
         .theta_indicator_mimo_valid(d.mdThetaIndcrMIMOVldFlg)
         .phi_indicator(d.mdPhiIndcr)
         .phi_indicator_valid(d.mdPhiIndcrVldFlg)
         .reflections_indicator(d.nRefIndcr)
         .reflections_indicator_valid(d.nRefIndcrVldFlg)
         .theta_mimo_rate(d.thetaMIMORate)
         .theta_mimo_rate_valid(d.thetaMIMORteVldFlag);
}

inline
auto to_msg(const LocAttributes_Packet & d)
{
  return build<msg::LocationAttributesPacket>()
         .sensor_modulation_performance(to_msg(d.sensor_modulation_performance))
         .misalignment(to_msg(d.misalignment))
         .interference_indicator(to_msg(d.interference_indicator))
         .sensor_field_of_view(to_msg(d.sensor_field_of_view))
         .sensor_coating(to_msg(d.sensor_coating));
}

inline
auto to_msg(const LocAtr_MountingPosition & d)
{
  return build<msg::SensorMounting>()
         .x(d.LocAtr_SenPosX)
         .y(d.LocAtr_SenPosY)
         .z(d.LocAtr_SenPosZ)
         .azimuth(d.LocAtr_SenPosAzi)
         .elevation(d.LocAtr_SenPosEle)
         .orientation(d.LocAtr_SenOrient);
}

inline
auto to_msg(const LocationAttributes & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  return build<msg::LocationAttributes>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .location_attributes_header(to_msg(d.loc_atr_header))
         .location_attributes_packet(to_msg(d.loc_atr_packet))
         .mounting_position(to_msg(d.loc_atr_mounting_position));
}

inline
sensor_msgs::msg::PointCloud2 to_msg(
  const Locations & locations, const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  using sensor_msgs::PointCloud2Iterator;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id);
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    19,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    // "padding", 1, sensor_msgs::msg::PointField::FLOAT32,  // TODO(rcp1-beg) Needed?
    "radial_distance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_velocity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth_angle", 1, sensor_msgs::msg::PointField::FLOAT32,
    "elevation_angle", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radar_cross_section", 1, sensor_msgs::msg::PointField::FLOAT32,
    "signal_noise_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_distance_variance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_velocity_variance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth_angle_variance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "elevation_angle_variance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radial_distance_velocity_covariance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity_resolution_processing_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "azimuth_angle_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "elevation_angle_probability", 1, sensor_msgs::msg::PointField::FLOAT32,
    "measurement_status", 1, sensor_msgs::msg::PointField::FLOAT32,
    "idx_azimuth_ambiguity_peer", 1, sensor_msgs::msg::PointField::FLOAT32
  );

  modifier.resize(locations.size());

  PointCloud2Iterator<float> x(msg, "x");
  PointCloud2Iterator<float> y(msg, "y");
  PointCloud2Iterator<float> z(msg, "z");
  PointCloud2Iterator<float> radial_distance(msg, "radial_distance");
  PointCloud2Iterator<float> radial_velocity(msg, "radial_velocity");
  PointCloud2Iterator<float> azimuth_angle(msg, "azimuth_angle");
  PointCloud2Iterator<float> elevation_angle(msg, "elevation_angle");
  PointCloud2Iterator<float> radar_cross_section(msg, "radar_cross_section");
  PointCloud2Iterator<float> signal_noise_ratio(msg, "signal_noise_ratio");
  PointCloud2Iterator<float> radial_distance_variance(msg, "radial_distance_variance");
  PointCloud2Iterator<float> radial_velocity_variance(msg, "radial_velocity_variance");
  PointCloud2Iterator<float> azimuth_angle_variance(msg, "azimuth_angle_variance");
  PointCloud2Iterator<float> elevation_angle_variance(msg, "elevation_angle_variance");
  PointCloud2Iterator<float> radial_distance_velocity_covariance(msg,
    "radial_distance_velocity_covariance");
  PointCloud2Iterator<float> velocity_resolution_processing_probability(msg,
    "velocity_resolution_processing_probability");
  PointCloud2Iterator<float> azimuth_angle_probability(msg, "azimuth_angle_probability");
  PointCloud2Iterator<float> elevation_angle_probability(msg, "elevation_angle_probability");
  PointCloud2Iterator<float> measurement_status(msg, "measurement_status");
  PointCloud2Iterator<float> idx_azimuth_ambiguity_peer(msg, "idx_azimuth_ambiguity_peer");

  for (const auto & l : locations) {
    const float & phi = l.LocData_EleAng_i_j;
    const float & theta = l.LocData_AziAng_i_j;
    float cos_phi = std::cos(phi);
    float sin_theta = std::sin(theta);

    *x = l.LocData_RadDist_i_j * std::sqrt(cos_phi * cos_phi - sin_theta * sin_theta);
    *y = l.LocData_RadDist_i_j * sin_theta;
    *z = l.LocData_RadDist_i_j * std::sin(phi);
    *radial_distance = l.LocData_RadDist_i_j;
    *radial_velocity = l.LocData_RadRelVel_i_j;
    *azimuth_angle = theta;
    *elevation_angle = phi;
    *radar_cross_section = l.LocData_Rcs_i_j;
    *signal_noise_ratio = l.LocData_Snr_i_j;
    *radial_distance_variance = l.LocData_RadDistVar_i_j;
    *radial_velocity_variance = l.LocData_RadRelVelVar_i_j;
    *azimuth_angle_variance = l.LocData_VarAzi_i_j;
    *elevation_angle_variance = l.LocData_VarEle_i_j;
    *radial_distance_velocity_covariance = l.LocData_DistVelCov_i_j;
    *velocity_resolution_processing_probability = l.LocData_ProVelRes_i_j;
    *azimuth_angle_probability = l.LocData_ProAziAng_i_j;
    *elevation_angle_probability = l.LocData_ProEleAng_i_j;
    *measurement_status = static_cast<float>(l.LocData_MeasStat_i_j);
    *idx_azimuth_ambiguity_peer = static_cast<float>(l.LocData_IdAngAmb_i_j);

    ++x;
    ++y;
    ++z;
    ++radial_distance;
    ++radial_velocity;
    ++azimuth_angle;
    ++elevation_angle;
    ++radar_cross_section;
    ++signal_noise_ratio;
    ++radial_distance_variance;
    ++radial_velocity_variance;
    ++azimuth_angle_variance;
    ++elevation_angle_variance;
    ++radial_distance_velocity_covariance;
    ++velocity_resolution_processing_probability;
    ++azimuth_angle_probability;
    ++elevation_angle_probability;
    ++measurement_status;
    ++idx_azimuth_ambiguity_peer;
  }

  return msg;
}

// From ROS message
inline
auto from_msg(const msg::EgoVehicleInput::ConstSharedPtr & msg)
{
  const auto & x_velocity_variance = msg->vehicle_data.velocity.covariance[0];
  if (x_velocity_variance < 0.0) {
    throw std::out_of_range{"X velocity variance must be non-negative."};
  }

  EgoVehicleInput d;
  d.vehicle_data.EgoData_VehSpd = msg->vehicle_data.velocity.twist.linear.x;
  // Sensor uses deg/s
  d.vehicle_data.EgoData_RelYawRate = msg->vehicle_data.velocity.twist.angular.z * kRadToDeg;
  d.vehicle_data.EgoData_VehSpdStdDev = std::sqrt(x_velocity_variance);
  d.vehicle_data.EgoData_LogAcc = msg->vehicle_data.acceleration.linear.x;
  return d;
}

inline
auto from_srv(const srv::MeasurementProgram::Request::SharedPtr msg)
{
  MeasurementProgram d;
  d.measurement_program_data.MeasPgm_ID = msg->id;
  return d;
}

}  // namespace off_highway_premium_radar
