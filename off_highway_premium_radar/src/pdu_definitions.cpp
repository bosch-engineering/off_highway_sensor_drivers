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

#include "off_highway_premium_radar/pdu_definitions.hpp"

#include <endian.h>

#include <bit>
#include <cstring>

#include "helper.hpp"
namespace off_highway_premium_radar
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

LocationDataPdu::LocationDataPdu(const std::array<uint8_t, kPduSize> & buffer)
: LocationDataPdu(std::bit_cast<LocationDataPdu>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
  loc_data_header.betoh();

  for (auto & loc_data_packet : loc_data_packets) {
    loc_data_packet.betoh();
  }
}

SensorStateInformation::SensorStateInformation(const std::array<uint8_t, kPduSize> & buffer)
: SensorStateInformation(std::bit_cast<SensorStateInformation>(buffer))
{
  pdu_id = be32toh(pdu_id);
  pdu_payload_length = be32toh(pdu_payload_length);
  // e2e_header is always FF
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
  LocAtr_MalEstQuality = be32tohf(LocAtr_MalEstQuality);
}

void InterferenceIndicator::betoh()
{
  LocAtr_FovRedInt = be32tohf(LocAtr_FovRedInt);
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

void SensorCoating::betoh()
{
  mdThetaIndcrMIMO = be32tohf(mdThetaIndcrMIMO);
  mdPhiIndcr = be32tohf(mdPhiIndcr);
  nRefIndcr = be32tohf(nRefIndcr);
  thetaMIMORate = be32tohf(thetaMIMORate);
}

void LocAttributes_Packet::betoh()
{
  sensor_modulation_performance.betoh();
  misalignment.betoh();
  interference_indicator.betoh();
  sensor_field_of_view.betoh();
  sensor_coating.betoh();
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

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

std::vector<uint8_t> LocationAttributes::serialize()
{
  pdu_id = htobe32(kPduId);
  pdu_payload_length = htobe32(kPduPayloadLength);
  loc_atr_packet.sensor_coating.mdThetaIndcrMIMO = htobe32f(
    loc_atr_packet.sensor_coating.mdThetaIndcrMIMO);
  loc_atr_packet.sensor_coating.mdPhiIndcr = htobe32f(
    loc_atr_packet.sensor_coating.mdPhiIndcr);
  loc_atr_packet.sensor_coating.nRefIndcr = htobe32f(
    loc_atr_packet.sensor_coating.nRefIndcr);
  loc_atr_packet.sensor_coating.thetaMIMORate = htobe32f(
    loc_atr_packet.sensor_coating.thetaMIMORate);

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(*this));
  std::memcpy(buffer.data(), this, sizeof(*this));
  return buffer;
}

}  // namespace off_highway_premium_radar
