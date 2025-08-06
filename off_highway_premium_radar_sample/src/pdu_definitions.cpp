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

#include "off_highway_premium_radar_sample/pdu_definitions.hpp"

#include <endian.h>

#include <bit>
#include <cstring>

#include "helper.hpp"
namespace off_highway_premium_radar_sample
{

void LocData_Header_i::betoh()
{
  LocData_LgpVer_i = be32toh(LocData_LgpVer_i);
  LocData_TimeSts_i = be32toh(LocData_TimeSts_i);
  LocData_TimeStns_i = be32toh(LocData_TimeStns_i);
  LocData_NumLoc = be16toh(LocData_NumLoc);
}

void LocData_Packet_i_j::betoh()
{
  LocData_RadDist_i_j = be32tohf(LocData_RadDist_i_j);
  LocData_RadRelVel_i_j = be32tohf(LocData_RadRelVel_i_j);
  LocData_AziAng_i_j = be32tohf(LocData_AziAng_i_j);
  LocData_EleAng_i_j = be32tohf(LocData_EleAng_i_j);
  LocData_Rcs_i_j = be32tohf(LocData_Rcs_i_j);
  LocData_Snr_i_j = be32tohf(LocData_Snr_i_j);
  LocData_RadDistVar_i_j = be32tohf(LocData_RadDistVar_i_j);
  LocData_RadRelVelVar_i_j = be32tohf(LocData_RadRelVelVar_i_j);
  LocData_VarAzi_i_j = be32tohf(LocData_VarAzi_i_j);
  LocData_VarEle_i_j = be32tohf(LocData_VarEle_i_j);
  LocData_DistVelCov_i_j = be32tohf(LocData_DistVelCov_i_j);
  LocData_ProVelRes_i_j = be32tohf(LocData_ProVelRes_i_j);
  LocData_ProAziAng_i_j = be32tohf(LocData_ProAziAng_i_j);
  LocData_ProEleAng_i_j = be32tohf(LocData_ProEleAng_i_j);
  LocData_MeasStat_i_j = be16toh(LocData_MeasStat_i_j);
  LocData_IdAngAmb_i_j = be16toh(LocData_IdAngAmb_i_j);
}

void LocData_Packet_i_j::check()
{
  CHECK_SIGNAL(LocData_RadDist_i_j);
  CHECK_SIGNAL(LocData_RadRelVel_i_j);
  CHECK_SIGNAL(LocData_AziAng_i_j);
  CHECK_SIGNAL(LocData_EleAng_i_j);
  CHECK_SIGNAL(LocData_Rcs_i_j);
  CHECK_SIGNAL(LocData_Snr_i_j);
  CHECK_SIGNAL(LocData_RadDistVar_i_j);
  CHECK_SIGNAL(LocData_RadRelVelVar_i_j);
  CHECK_SIGNAL(LocData_VarAzi_i_j);
  CHECK_SIGNAL(LocData_VarEle_i_j);
  CHECK_SIGNAL(LocData_DistVelCov_i_j);
  CHECK_SIGNAL(LocData_ProVelRes_i_j);
  CHECK_SIGNAL(LocData_ProAziAng_i_j);
  CHECK_SIGNAL(LocData_ProEleAng_i_j);
  CHECK_SIGNAL(LocData_MeasStat_i_j);
  CHECK_SIGNAL(LocData_IdAngAmb_i_j);
}

LocationDataPdu::LocationDataPdu(const std::array<uint8_t, kPduSize> & buffer)
: LocationDataPdu(std::bit_cast<LocationDataPdu>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  loc_data_header.betoh();

  for (auto & loc_data_packet : loc_data_packets) {
    loc_data_packet.betoh();
    loc_data_packet.check();
  }
}

void EgoVehicleData::betoh()
{
  FeedBack_RelYawRate = be32tohf(FeedBack_RelYawRate);
  FeedBack_VehSpd = be32tohf(FeedBack_VehSpd);
  FeedBack_VehSpdStdDev = be32tohf(FeedBack_VehSpdStdDev);
  FeedBack_LogAcc = be32tohf(FeedBack_LogAcc);
}

SensorFeedback::SensorFeedback(const std::array<uint8_t, kPduSize> & buffer)
: SensorFeedback(std::bit_cast<SensorFeedback>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  FeedBack_LgpVer = be32toh(FeedBack_LgpVer);
  FeedBack_TimeS = be32toh(FeedBack_TimeS);
  FeedBack_TimeNs = be32toh(FeedBack_TimeNs);
  measurement_cycle_sync_data.FeedBack_SenTimeOff = be32toh(
    measurement_cycle_sync_data.FeedBack_SenTimeOff);
  ego_vehicle_data.betoh();
}

SensorStateInformation::SensorStateInformation(const std::array<uint8_t, kPduSize> & buffer)
: SensorStateInformation(std::bit_cast<SensorStateInformation>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  SenStInfo_LgpVer = be32toh(SenStInfo_LgpVer);
  sensor_state_data.SenStInfo_SwNu_Cust = be32toh(sensor_state_data.SenStInfo_SwNu_Cust);
}

void SensorEthernetConfigurationInformation::betoh()
{
  BroadCast_SenIpAdd = be32toh(BroadCast_SenIpAdd);
  BroadCast_DestIpAdd = be32toh(BroadCast_DestIpAdd);
  BroadCast_SenNetmask = be32toh(BroadCast_SenNetmask);
  BroadCast_SenVlan = be16toh(BroadCast_SenVlan);
  BroadCast_SouPort = be16toh(BroadCast_SouPort);
  // BroadCast_SouPortUnassigned
  BroadCast_DestPort = be16toh(BroadCast_DestPort);
  // BroadCast_DestPortUnassigned
}

void DignosticsEthernetConfigurationInformation::betoh()
{
  BroadCast_DiagSouIpAdd = be32toh(BroadCast_DiagSouIpAdd);
  BroadCast_DiagNetmask = be32toh(BroadCast_DiagNetmask);
  BroadCast_DiagVlan = be16toh(BroadCast_DiagVlan);
  BroadCast_DiagPort = be16toh(BroadCast_DiagPort);
}

void DoIPInformation::betoh()
{
  BroadCast_SenDoIPPhyAdd = be16toh(BroadCast_SenDoIPPhyAdd);
  BroadCast_SenDoIPFuncAdd = be16toh(BroadCast_SenDoIPFuncAdd);
  BroadCast_DoIPTarAdd = be16toh(BroadCast_DoIPTarAdd);
}

void SensorBroadcastData::betoh()
{
  BroadCast_SwCust = be32toh(BroadCast_SwCust);
  sensor_ethernet_configuration_information.betoh();
  dignostics_ethernet_configuration_information.betoh();
  BroadCast_SenMacAd = be64toh(BroadCast_SenMacAd);
  doip_information.betoh();
}

SensorBroadcast::SensorBroadcast(const std::array<uint8_t, kPduSize> & buffer)
: SensorBroadcast(std::bit_cast<SensorBroadcast>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  BroadCast_LgpVer = be32toh(BroadCast_LgpVer);
  sensor_broadcast_data.betoh();
}

void LocAtr_Header_i::betoh()
{
  LocAtr_LgpVer = be32toh(LocAtr_LgpVer);
  LocAtr_TimeSts = be32toh(LocAtr_TimeSts);
  LocAtr_TimeStns = be32toh(LocAtr_TimeStns);
}

void SensorModulationPerformance::betoh()
{
  LocAtr_ModID = be16toh(LocAtr_ModID);
  LocAtr_DistRangScalFact = be32tohf(LocAtr_DistRangScalFact);
  LocAtr_SepRadDist = be32tohf(LocAtr_SepRadDist);
  LocAtr_SepRadVelo = be32tohf(LocAtr_SepRadVelo);
  LocAtr_PrecRadDist = be32tohf(LocAtr_PrecRadDist);
  LocAtr_PrecRadVelo = be32tohf(LocAtr_PrecRadVelo);
  LocAtr_RadDistVeloCovVar = be32tohf(LocAtr_RadDistVeloCovVar);
  LocAtr_MinRadDist = be32tohf(LocAtr_MinRadDist);
  LocAtr_MaxRadDist = be32tohf(LocAtr_MaxRadDist);
  LocAtr_MinRadVelo = be32tohf(LocAtr_MinRadVelo);
  LocAtr_MaxRadVelo = be32tohf(LocAtr_MaxRadVelo);
}

void SensorModulationPerformance::check()
{
  CHECK_SIGNAL(LocAtr_DmpID);
  CHECK_SIGNAL(LocAtr_ModID);
  CHECK_SIGNAL(LocAtr_DistRangScalFact);
  CHECK_SIGNAL(LocAtr_SepRadDist);
  CHECK_SIGNAL(LocAtr_SepRadVelo);
  CHECK_SIGNAL(LocAtr_PrecRadDist);
  CHECK_SIGNAL(LocAtr_PrecRadVelo);
  CHECK_SIGNAL(LocAtr_RadDistVeloCovVar);
  CHECK_SIGNAL(LocAtr_MinRadDist);
  CHECK_SIGNAL(LocAtr_MaxRadDist);
  CHECK_SIGNAL(LocAtr_MinRadVelo);
  CHECK_SIGNAL(LocAtr_MaxRadVelo);
}

void Misalignment::betoh()
{
  LocAtr_ThetaMalAng = be32tohf(LocAtr_ThetaMalAng);
  LocAtr_ThetaMalAngVar = be32tohf(LocAtr_ThetaMalAngVar);
  LocAtr_PhiMalAng = be32tohf(LocAtr_PhiMalAng);
  LocAtr_PhiMalAngVar = be32tohf(LocAtr_PhiMalAngVar);
  LocAtr_PhiMalAngEme = be32tohf(LocAtr_PhiMalAngEme);
  LocAtr_PhiMalAngEmeVar = be32tohf(LocAtr_PhiMalAngEmeVar);
  LocAtr_MalStatus = be16toh(LocAtr_MalStatus);
  LocAtr_MalStatusEme = be16toh(LocAtr_MalStatusEme);
  LocAtr_PercNegativeTheta = be32tohf(LocAtr_PercNegativeTheta);
  LocAtr_MinThetaMalSOs = be32tohf(LocAtr_MinThetaMalSOs);
  LocAtr_MaxThetaMalSOs = be32tohf(LocAtr_MaxThetaMalSOs);
  LocAtr_VarThetaMalSOs = be32tohf(LocAtr_VarThetaMalSOs);
  LocAtr_MeanThetaMalSOs = be32tohf(LocAtr_MeanThetaMalSOs);
  LocAtr_MinPhiMalSOs = be32tohf(LocAtr_MinPhiMalSOs);
  LocAtr_MaxPhiMalSOs = be32tohf(LocAtr_MaxPhiMalSOs);
  LocAtr_VarPhiMalSOs = be32tohf(LocAtr_VarPhiMalSOs);
  LocAtr_MeanPhiMalSOs = be32tohf(LocAtr_MeanPhiMalSOs);
  LocAtr_SpreadPhiMalSOs = be32tohf(LocAtr_SpreadPhiMalSOs);
  LocAtr_NumSOs = be16toh(LocAtr_NumSOs);
  LocAtr_NumEmeLocs = be16toh(LocAtr_NumEmeLocs);
}

void Misalignment::check()
{
  CHECK_SIGNAL(LocAtr_ThetaMalAng);
  CHECK_SIGNAL(LocAtr_ThetaMalAngVar);
  CHECK_SIGNAL(LocAtr_PhiMalAng);
  CHECK_SIGNAL(LocAtr_PhiMalAngVar);
  CHECK_SIGNAL(LocAtr_PhiMalAngEme);
  CHECK_SIGNAL(LocAtr_PhiMalAngEmeVar);
  CHECK_SIGNAL(LocAtr_MalStatus);
  CHECK_SIGNAL(LocAtr_MalStatusEme);
  CHECK_SIGNAL(LocAtr_PercNegativeTheta);
  CHECK_SIGNAL(LocAtr_MinThetaMalSOs);
  CHECK_SIGNAL(LocAtr_MaxThetaMalSOs);
  CHECK_SIGNAL(LocAtr_VarThetaMalSOs);
  CHECK_SIGNAL(LocAtr_MeanThetaMalSOs);
  CHECK_SIGNAL(LocAtr_MinPhiMalSOs);
  CHECK_SIGNAL(LocAtr_MaxPhiMalSOs);
  CHECK_SIGNAL(LocAtr_VarPhiMalSOs);
  CHECK_SIGNAL(LocAtr_MeanPhiMalSOs);
  CHECK_SIGNAL(LocAtr_SpreadPhiMalSOs);
  CHECK_SIGNAL(LocAtr_NumSOs);
  CHECK_SIGNAL(LocAtr_NumEmeLocs);
}

void InterferenceIndicator::betoh()
{
  LocAtr_FovRedInt = be32tohf(LocAtr_FovRedInt);
}

void InterferenceIndicator::check()
{
  CHECK_SIGNAL(LocAtr_FovRedInt);
  CHECK_SIGNAL(LocAtr_IntStat);
}

void SensorFieldOfView::betoh()
{
  for (size_t i = 0; i < LocAtr_FoVRange.size(); ++i) {
    LocAtr_FoVRange[i] = be32tohf(LocAtr_FoVRange[i]);
  }
  for (size_t i = 0; i < LocAtr_AziAngArr.size(); ++i) {
    LocAtr_AziAngArr[i] = be32tohf(LocAtr_AziAngArr[i]);
  }
  for (size_t i = 0; i < LocAtr_RangScaEle.size(); ++i) {
    LocAtr_RangScaEle[i] = be32tohf(LocAtr_RangScaEle[i]);
  }
  for (size_t i = 0; i < LocAtr_EleAngArr.size(); ++i) {
    LocAtr_EleAngArr[i] = be32tohf(LocAtr_EleAngArr[i]);
  }
}

void SensorFieldOfView::check()
{
  // Need to use counter-based loop due to packed struct
  for (size_t i = 0; i < LocAtr_FoVRange.size(); ++i) {
    LocAtr_FoVRange[i] = r_LocAtr_FoVRange.check(LocAtr_FoVRange[i]);
  }
  for (size_t i = 0; i < LocAtr_AziAngArr.size(); ++i) {
    LocAtr_AziAngArr[i] = r_LocAtr_AziAngArr.check(LocAtr_AziAngArr[i]);
  }
  for (size_t i = 0; i < LocAtr_RangScaEle.size(); ++i) {
    LocAtr_RangScaEle[i] = r_LocAtr_RangScaEle.check(LocAtr_RangScaEle[i]);
  }
  for (size_t i = 0; i < LocAtr_EleAngArr.size(); ++i) {
    LocAtr_EleAngArr[i] = r_LocAtr_EleAngArr.check(LocAtr_EleAngArr[i]);
  }
}

void LocAttributes_Packet::betoh()
{
  sensor_modulation_performance.betoh();
  misalignment.betoh();
  interference_indicator.betoh();
  sensor_field_of_view.betoh();
}

void LocAttributes_Packet::check()
{
  sensor_modulation_performance.check();
  misalignment.check();
  interference_indicator.check();
  sensor_field_of_view.check();
}

void LocAtr_MountingPosition::betoh()
{
  LocAtr_SenPosX = be32tohf(LocAtr_SenPosX);
  LocAtr_SenPosY = be32tohf(LocAtr_SenPosY);
  LocAtr_SenPosZ = be32tohf(LocAtr_SenPosZ);
  LocAtr_SenPosAzi = be32tohf(LocAtr_SenPosAzi);
  LocAtr_SenPosEle = be32tohf(LocAtr_SenPosEle);
}

LocationAttributes::LocationAttributes(const std::array<uint8_t, kPduSize> & buffer)
: LocationAttributes(std::bit_cast<LocationAttributes>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  loc_atr_header.betoh();
  loc_atr_packet.betoh();
  loc_atr_mounting_position.betoh();
  loc_atr_packet.check();
}


SensorDTCInformation::SensorDTCInformation(const std::array<uint8_t, kPduSize> & buffer)
: SensorDTCInformation(std::bit_cast<SensorDTCInformation>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  SensorDtc_LgpVer = be32toh(SensorDtc_LgpVer);
  for (size_t i = 0; i < dtc_information_data.SensorDtc_Dtc.size(); ++i) {
    dtc_information_data.SensorDtc_Dtc[i] = be32toh(dtc_information_data.SensorDtc_Dtc[i]);
  }
}

std::vector<uint8_t> EgoVehicleInput::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  vehicle_data.EgoData_RelYawRate = htobe32f(vehicle_data.EgoData_RelYawRate);
  vehicle_data.EgoData_VehSpd = htobe32f(vehicle_data.EgoData_VehSpd);
  vehicle_data.EgoData_VehSpdStdDev = htobe32f(vehicle_data.EgoData_VehSpdStdDev);
  vehicle_data.EgoData_LogAcc = htobe32f(vehicle_data.EgoData_LogAcc);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> MeasurementCycleSynchronisation::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  mcs_data.MCS_SenTimeOff = htobe32(mcs_data.MCS_SenTimeOff);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> SensorModeRequest::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  sensor_mode_data.SenModReq_Unassigned.fill(0);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> MeasurementProgram::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  measurement_program_data.MeasPgm_ID = htobe16(measurement_program_data.MeasPgm_ID);
  measurement_program_data.MeasPgm_Unassigned.fill(0);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> SensorStateInformation::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  SenStInfo_LgpVer = htobe32(SenStInfo_LgpVer);
  sensor_state_data.SenStInfo_SwNu_Cust = htobe32(sensor_state_data.SenStInfo_SwNu_Cust);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> LocationDataPdu::serialize()
{
  pdu_id = htobe32(pdu_id);
  pdu_payload_length = htobe32(kPduPayloadLength);

  loc_data_header.LocData_LgpVer_i = htobe32(loc_data_header.LocData_LgpVer_i);
  loc_data_header.LocData_TimeSts_i = htobe32(loc_data_header.LocData_TimeSts_i);
  loc_data_header.LocData_TimeStns_i = htobe32(loc_data_header.LocData_TimeStns_i);
  loc_data_header.LocData_NumLoc = htobe16(loc_data_header.LocData_NumLoc);

  for (size_t i = 0; i < loc_data_packets.size(); ++i) {
    loc_data_packets[i].LocData_RadDist_i_j = htobe32f(loc_data_packets[i].LocData_RadDist_i_j);
    loc_data_packets[i].LocData_RadRelVel_i_j = htobe32f(loc_data_packets[i].LocData_RadRelVel_i_j);
    loc_data_packets[i].LocData_AziAng_i_j = htobe32f(loc_data_packets[i].LocData_AziAng_i_j);
    loc_data_packets[i].LocData_EleAng_i_j = htobe32f(loc_data_packets[i].LocData_EleAng_i_j);
    loc_data_packets[i].LocData_Rcs_i_j = htobe32f(loc_data_packets[i].LocData_Rcs_i_j);
    loc_data_packets[i].LocData_Snr_i_j = htobe32f(loc_data_packets[i].LocData_Snr_i_j);
    loc_data_packets[i].LocData_RadDistVar_i_j =
      htobe32f(loc_data_packets[i].LocData_RadDistVar_i_j);
    loc_data_packets[i].LocData_RadRelVelVar_i_j =
      htobe32f(loc_data_packets[i].LocData_RadRelVelVar_i_j);
    loc_data_packets[i].LocData_VarAzi_i_j = htobe32f(loc_data_packets[i].LocData_VarAzi_i_j);
    loc_data_packets[i].LocData_VarEle_i_j = htobe32f(loc_data_packets[i].LocData_VarEle_i_j);
    loc_data_packets[i].LocData_DistVelCov_i_j =
      htobe32f(loc_data_packets[i].LocData_DistVelCov_i_j);
    loc_data_packets[i].LocData_ProVelRes_i_j = htobe32f(loc_data_packets[i].LocData_ProVelRes_i_j);
    loc_data_packets[i].LocData_ProAziAng_i_j = htobe32f(loc_data_packets[i].LocData_ProAziAng_i_j);
    loc_data_packets[i].LocData_ProEleAng_i_j = htobe32f(loc_data_packets[i].LocData_ProEleAng_i_j);
    loc_data_packets[i].LocData_MeasStat_i_j = htobe16(loc_data_packets[i].LocData_MeasStat_i_j);
    loc_data_packets[i].LocData_IdAngAmb_i_j = htobe16(loc_data_packets[i].LocData_IdAngAmb_i_j);
  }

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

}  // namespace off_highway_premium_radar_sample
