// Copyright 2023 Robert Bosch GmbH and its subsidiaries
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
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "off_highway_premium_radar_sample/pdu_definitions.hpp"

#include \
  "off_highway_premium_radar_sample_msgs/msg/diagnostics_ethernet_configuration_information.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/do_ip_information.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/ego_vehicle_data.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/interference_indicator.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/location_attributes_header.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/location_attributes_packet.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/location_attributes.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/location_data_header.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/location_data_packet.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/measurement_cycle_sync_data.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/misalignment_packet.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/operation_mode.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_broadcast_data.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_broadcast.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_dtc_information.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_ethernet_configuration_information.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_feedback.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_field_of_view.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_modulation_performance.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/sensor_state_information.hpp"
#include "off_highway_premium_radar_sample_msgs/msg/time.hpp"

#include "off_highway_premium_radar_sample_msgs/msg/ego_vehicle_input.hpp"
#include "off_highway_premium_radar_sample_msgs/srv/measurement_program.hpp"
#include "off_highway_premium_radar_sample_msgs/srv/sensor_mode_request.hpp"

namespace off_highway_premium_radar_sample
{

using namespace off_highway_premium_radar_sample_msgs;  // NOLINT

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
auto to_msg(const MeasurementCycleSyncData & d)
{
  return build<msg::MeasurementCycleSyncData>()
         .sync(d.FeedBack_SyncType)
         .sensor_time_offset(rclcpp::Time(d.FeedBack_SenTimeOff));
}

inline
auto to_msg(const EgoVehicleData & d)
{
  geometry_msgs::msg::TwistWithCovariance velocity;
  velocity.twist.linear.x = d.FeedBack_VehSpd;
  // Sensor uses deg/s
  velocity.twist.angular.z = d.FeedBack_RelYawRate * kDegToRad;
  velocity.covariance[0] = d.FeedBack_VehSpdStdDev * d.FeedBack_VehSpdStdDev;

  geometry_msgs::msg::Accel acceleration;
  acceleration.linear.x = d.FeedBack_LogAcc;

  return build<msg::EgoVehicleData>()
         .velocity(velocity)
         .acceleration(acceleration);
}

inline
auto to_msg(const SensorFeedback & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  return build<msg::SensorFeedback>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .lgp_version(d.FeedBack_LgpVer)
         .vehicle_time(to_msg(d.FeedBack_TimeS, d.FeedBack_TimeNs))
         .measurement_cycle_sync_data(to_msg(d.measurement_cycle_sync_data))
         .time_sync_status(d.FeedBack_TimeSynSta)
         .ego_vehicle_data(to_msg(d.ego_vehicle_data));
}

inline
auto to_msg(
  const SensorStateInformation & d, const rclcpp::Time stamp,
  const std::string & frame_id)
{
  return build<msg::SensorStateInformation>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .lgp_version(d.SenStInfo_LgpVer)
         .sensor_state(d.sensor_state_data.SenStInfo_SenSt)
         .customer_version(d.sensor_state_data.SenStInfo_SwNu_Cust)
         .internal_version(d.sensor_state_data.sen_st_info_sw_nu_int.CommitId);
}

inline
auto to_msg(const SensorEthernetConfigurationInformation & d)
{
  return build<msg::SensorEthernetConfigurationInformation>()
         .sensor_ip_address(asio::ip::address_v4(d.BroadCast_SenIpAdd).to_string())
         .destination_ip_address(asio::ip::address_v4(d.BroadCast_DestIpAdd).to_string())
         .netmask(asio::ip::address_v4(d.BroadCast_SenNetmask).to_string())
         .vlan(d.BroadCast_SenVlan)
         .source_port(d.BroadCast_SouPort)
         .destination_port(d.BroadCast_DestPort);
}

inline
msg::DiagnosticsEthernetConfigurationInformation to_msg(
  const DignosticsEthernetConfigurationInformation & d)
{
  return build<msg::DiagnosticsEthernetConfigurationInformation>()
         .ip_address(asio::ip::address_v4(d.BroadCast_DiagSouIpAdd).to_string())
         .netmask(asio::ip::address_v4(d.BroadCast_DiagNetmask).to_string())
         .vlan(d.BroadCast_DiagVlan)
         .port(d.BroadCast_DiagPort);
}

inline
auto to_msg(const DoIPInformation & d)
{
  return build<msg::DoIpInformation>()
         .physical_address(d.BroadCast_SenDoIPPhyAdd)
         .functional_address(d.BroadCast_SenDoIPFuncAdd)
         .target_address(d.BroadCast_DoIPTarAdd);
}

inline
auto to_msg(const SensorBroadcastData & d)
{
  return build<msg::SensorBroadcastData>()
         .customer_version(d.BroadCast_SwCust)
         .sensor_ethernet_configuration_information(
    to_msg(d.sensor_ethernet_configuration_information))
         .diagnostics_ethernet_configuration_information(
    to_msg(d.dignostics_ethernet_configuration_information))
         .sensor_mac_address(d.BroadCast_SenMacAd)
         .doip_information(to_msg(d.doip_information));
}

inline
auto to_msg(const SensorBroadcast & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  return build<msg::SensorBroadcast>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .lgp_version(d.BroadCast_LgpVer)
         .sensor_broadcast_data(to_msg(d.sensor_broadcast_data));
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
         .num_eme(d.LocAtr_NumEmeLocs);
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
auto to_msg(const LocAttributes_Packet & d)
{
  return build<msg::LocationAttributesPacket>()
         .sensor_modulation_performance(to_msg(d.sensor_modulation_performance))
         .misalignment(to_msg(d.misalignment))
         .interference_indicator(to_msg(d.interference_indicator))
         .sensor_field_of_view(to_msg(d.sensor_field_of_view));
}

inline
auto to_msg(const LocationAttributes & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  geometry_msgs::msg::Pose mounting_pose;
  mounting_pose.position.x = d.loc_atr_mounting_position.LocAtr_SenPosX;
  mounting_pose.position.y = d.loc_atr_mounting_position.LocAtr_SenPosY;
  mounting_pose.position.z = d.loc_atr_mounting_position.LocAtr_SenPosZ;

  tf2::Quaternion q_tf2;
  double roll = d.loc_atr_mounting_position.LocAtr_SenOrient == -1 ? std::numbers::pi : 0.;
  q_tf2.setRPY(
    roll, d.loc_atr_mounting_position.LocAtr_SenPosEle,
    d.loc_atr_mounting_position.LocAtr_SenPosAzi);
  mounting_pose.orientation = tf2::toMsg(q_tf2);


  return build<msg::LocationAttributes>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .location_attributes_header(to_msg(d.loc_atr_header))
         .location_attributes_packet(to_msg(d.loc_atr_packet))
         .mounting_pose(mounting_pose);
}

inline
auto to_msg(const SensorDTCInformation & d, const rclcpp::Time stamp, const std::string & frame_id)
{
  return build<msg::SensorDtcInformation>()
         .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(frame_id))
         .lgp_version(d.SensorDtc_LgpVer)
         .dtcs(d.dtc_information_data.SensorDtc_Dtc);
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
auto from_srv(const srv::SensorModeRequest::Request::SharedPtr msg)
{
  SensorModeRequest d;
  d.sensor_mode_data.SenModReq_RadMod = msg->radar_mode;
  return d;
}

inline
auto from_srv(const srv::MeasurementProgram::Request::SharedPtr msg)
{
  MeasurementProgram d;
  d.measurement_program_data.MeasPgm_ID = msg->id;
  return d;
}

}  // namespace off_highway_premium_radar_sample
