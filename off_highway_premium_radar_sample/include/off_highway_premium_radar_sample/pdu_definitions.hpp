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

#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "off_highway_premium_radar_sample/range.hpp"

namespace off_highway_premium_radar_sample
{

static constexpr uint8_t kPduHeaderLength{8U};
static constexpr uint8_t kPduIdOffset{0U};
static constexpr uint8_t kPduPayloadLengthOffset{4U};

/**
 * \brief E2E header
 * \note As specified in Technical Customer Information - Location Gateway Protocol
 *       Version LGP_V11.0 | 2023-03-23 | Version: LGP_V11.0 | Document Version 11.3
 * \note Reference for E2E - E2E Protocol Specification: AUTOSAR FO R20-11
 *       For LRR_LGU_PF_V10.0.0 and LRR_LGU_PF_V11.0.0 E2E Header signals are fixed as below
 *         E2E_length: 0xFFFF
 *         E2E_Counter: 0xFFFF
 *         E2E_DataId: 0xFFFFFFFF
 *         E2E_Crc: 0xFFFFFFFF
 */
struct E2E_Header
{
  /**
   * \brief Length of PDU Payload
   */
  uint16_t E2E_length;
  /**
   * \brief Length of PDU Payload
   * \note On the sender side, for the first transmission request of a data element the counter
   *       shall be initialized with 0 and shall be incremented by 1 for every subsequent send
   *       request.
   *       When the counter reaches the maximum value (0xFFFF), then it shall restart with 0
   *       for the next send request The receiver expects the counter to be starting from 0 and
   *       increments by 1 till 0xFFFF, the sender shall reset to 0 when max value 0xFFFF is
   *       reached.
   */
  uint16_t E2E_Counter;
  /**
   * \brief PDU ID of the PDU
   */
  uint32_t E2E_DataId;
  /**
   * \brief CRC of the PDU
   * \note CRC32P4 is used in the CRC calculation using polynomial 0xF4ACFB13 and Table based
   *       calculation CRC is calculated using all PDU payload data excluding the CRC bytes E2E_Crc
   */
  uint32_t E2E_Crc;
} __attribute__((packed));

/**
 * \brief Location Data Header
 * \note i indicates the PDU number
 */
struct LocData_Header_i
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief LGP Version
   */
  uint32_t LocData_LgpVer_i;
  /**
   * \brief Block Counter
   * \note Shall increment for every transmission cycle, the PDU’s with the same signal value
   *       indicates they are measured at the same modulation time, when signal reaches maximum
   *       value counter shall reset
   */
  uint8_t LocData_BlockCounter_i;
  /**
   * \brief Start of the measurement Time Stamp (seconds)
   * \note Total time is the sum of LocData_TimeSts_i and LocData_TimeStns_i
   * \note Unit: s
   */
  uint32_t LocData_TimeSts_i;
  /**
   * \brief Start of the measurement Time Stamp (nano seconds)
   * \note Total time is the sum of LocData_TimeSts_i and LocData_TimeStns_i
   * \note Unit: ns
   */
  uint32_t LocData_TimeStns_i;
  /**
   * \brief Operation Mode
   * \note 13 - Initialization/Re-Initialize
   *       20 - Normal Mode
   *       22 - Modulation Off (high temperature)
   *       40 - Alignment Mode
   *       51 - Radar Off (Failure, over temperature)
   *       61 - Drive test
   *       100 - Error
   */
  uint8_t LocData_OpMode;
  /**
   * \brief Validity of the data measured
   * \note 0 - values of measurement in the current cycle invalid
   *       1 - values of measurement in the current cycle valid
   */
  uint8_t LocData_DataMeas;
  /**
   * \brief Number of measured locations/point objects
   * \endverbatim
   */
  uint16_t LocData_NumLoc;
  /**
   * \brief Maximum number of Location data packets
   * \note The signal is always fixed to 16
   */
  uint8_t LocData_MaxLocPerPdu;
  /**
   * \brief Reserved
   * \note Fixed value 0xFFFFFFFFFFFFFFFF
   */
  std::array<uint8_t, 8> LocData_Reserved_i;
} __attribute__((packed));

/**
 * \brief Location Data
 * \note i indicates the PDU number
 *       j indicates Location data packet number in a PDU
 *       PDU ID for the first PDU is 0x13370001 and increments as the PDU number increases
 *       like LocData_0: 0x13370001, LocData_1: 0x13370002 ... LocData_63: 0x13370040
 */
struct LocData_Packet_i_j
{
  static constexpr Range<float> r_LocData_RadDist_i_j{0.0F, 302.0F, NAN};
  static constexpr Range<float> r_LocData_RadRelVel_i_j{-110.0F, 55.0F, NAN};
  static constexpr Range<float> r_LocData_AziAng_i_j{-1.5708F, 1.5708F, NAN};
  static constexpr Range<float> r_LocData_EleAng_i_j{-0.785398F, 0.785398F, NAN};
  static constexpr Range<float> r_LocData_Rcs_i_j{-50.0F, 70.0F, NAN};
  static constexpr Range<float> r_LocData_Snr_i_j{0.0F, 31.0F, NAN};
  static constexpr Range<float> r_LocData_RadDistVar_i_j{0.0F, 0.05F, NAN};
  static constexpr Range<float> r_LocData_RadRelVelVar_i_j{0.0F, 0.1F, NAN};
  static constexpr Range<float> r_LocData_VarAzi_i_j{0.0F, 0.0174533F, NAN};
  static constexpr Range<float> r_LocData_VarEle_i_j{0.0F, 0.0174533F, NAN};
  static constexpr Range<float> r_LocData_DistVelCov_i_j{-0.1F, 0.1F, NAN};
  // Probability ranges are wrongly stated in interface documentation V11.0 as [0, 1]...
  static constexpr Range<float> r_LocData_ProVelRes_i_j{0.0F, 255.0F, NAN};
  static constexpr Range<float> r_LocData_ProAziAng_i_j{0.0F, 255.0F, NAN};
  static constexpr Range<float> r_LocData_ProEleAng_i_j{0.0F, 255.0F, NAN};
  static constexpr Range<uint16_t> r_LocData_MeasStat_i_j{0u, 255u, 0xFFFF};
  static constexpr Range<uint16_t> r_LocData_IdAngAmb_i_j{0u, 1023u, 0xFFFF};

  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  /**
   * \brief Radial Distance
   * \note Unit: m
   */
  float LocData_RadDist_i_j;
  /**
   * \brief Radial Relative Velocity
   * \note Unit: m/s
   */
  float LocData_RadRelVel_i_j;
  /**
   * \brief Azimuth Angle
   * \note Unit: rad
   */
  float LocData_AziAng_i_j;
  /**
   * \brief Elevation Angle
   * \note Unit: rad
   */
  float LocData_EleAng_i_j;
  /**
   * \brief Radar Cross Section
   * \note Unit: dBm^2
   */
  float LocData_Rcs_i_j;
  /**
   * \brief SNR
   * \note Unit: dB
   */
  float LocData_Snr_i_j;
  /**
   * \brief Variance of Radial Distance measured
   * \note Unit: m^2
   */
  float LocData_RadDistVar_i_j;
  /**
   * \brief Variance of Radial Relative velocity measured
   * \note Unit: m^2/s^2
   */
  float LocData_RadRelVelVar_i_j;
  /**
   * \brief Variance of Azimuth Angle measured
   * \note Unit: rad^2
   */
  float LocData_VarAzi_i_j;
  /**
   * \brief Variance of Elevation Angle measured
   * \note Unit: rad^2
   */
  float LocData_VarEle_i_j;
  /**
   * \brief Covariance of Radial Distance and Velocity measured
   * \note Unit: m^2/s
   */
  float LocData_DistVelCov_i_j;
  /**
   * \brief Probability of velocity resolution processing
   */
  float LocData_ProVelRes_i_j;
  /**
   * \brief Probability for correct signal model for azimuth angle
   */
  float LocData_ProAziAng_i_j;
  /**
   * \brief Probability for correct signal model for elevation angle
   */
  float LocData_ProEleAng_i_j;
  /**
   * \brief Measurement Status
   * \note Bit 0 - Measured and Range check passed
   *       Bit 1 - Two target estimator azimuth
   *       Bit 2 - Two target estimator elevation
   *       Bit 3 - Location standing
   *       Bit 4 - Two target VAR estimator active
   *       Bit 5 - Low power location
   *       Bit 6 - Azimuth ambiguity location
   *       Bit 7 - Elevation ambiguity location
   */
  uint16_t LocData_MeasStat_i_j;
  /**
   * \brief Index Angle Ambiguity
   * \note The signal indicates index of Location Peer of Angle Ambiguity
   *       If an azimuth\elevation ambiguity is detected the index of the location which is the
   *       azimuth or elevation ambiguity peer is set. For unambiguous location the signal value
   *       will be set to 0xFFFF
   *       Example: If the two Location with Location Index (10 and 11) are ambiguous then
   *                  Location 10: LocData_IdAziAmb_i_j = 11
   *                  Location 11: LocData_IdAziAmb_i_j = 10
   *       Currently, the signal LocData_IdAngAmb_i_j refers only to Elevation Ambiguity and
   *       not Azimuth Ambiguity
   */
  uint16_t LocData_IdAngAmb_i_j;
  /**
   * \brief Reserved
   * \note Fixed value 0xFF (all 12 Bytes)
   */
  std::array<uint8_t, 12> LocData_Reserved_i_j;
} __attribute__((packed));

/**
 * \brief Single location Data Pdu
 */
struct LocationDataPdu
{
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPacketIdFirst = 0x13370001UL;
  static constexpr uint32_t kPacketIdLast = 0x13370040UL;
  static constexpr uint32_t kPduPayloadLength{1190u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};
  static constexpr uint8_t kMaxNumLocDataPacketsPerPdu{16U};

  explicit LocationDataPdu(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct E2E_Header e2e_header;
  struct LocData_Header_i loc_data_header;
  std::array<LocData_Packet_i_j, kMaxNumLocDataPacketsPerPdu> loc_data_packets;
} __attribute__((packed));

static_assert(
  sizeof(LocationDataPdu) == LocationDataPdu::kPduSize,
  "Wrong LocationDataPdu struct size!");

struct LocationData
{
  E2E_Header e2e_header;
  LocData_Header_i header;
  std::vector<LocData_Packet_i_j> locations;
};

struct MeasurementCycleSyncData
{
  /**
   * \brief Synchronisation Type
   * \note A maximum of 3 Radar Sensor can be Synchronized
   *       0 No Synchronization
   *       1 Timeslot Synchronization
   */
  uint8_t FeedBack_SyncType;
  /**
   * \brief Sensor Time offset
   * \note Unit: ns
   */
  uint32_t FeedBack_SenTimeOff;
} __attribute__((packed));

struct EgoVehicleData
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Relative Yaw Rate
   * \note Has the same value as received by the signal EgoData_RelYawRate
   * * \note Unit: deg/s
   */
  float FeedBack_RelYawRate;
  /**
   * \brief Vehicle speed
   * \note Has the same value as received by the signal EgoData_VehSpd
   * \note Unit: m/s
   */
  float FeedBack_VehSpd;
  /**
   * \brief Vehicle speed standard deviation
   * \note Has the same value as received by the signal EgoData_VehSpdStdDev
   * \note Unit: m/s
   */
  float FeedBack_VehSpdStdDev;
  /**
   * \brief Longitudinal Acceleration
   * \note Has the same value as received by the signal EgoData_LogAcc
   * \note Unit: m/s^2
   */
  float FeedBack_LogAcc;
} __attribute__((packed));

struct SensorFeedback
{
  static constexpr uint32_t kPduId{0x133ADDCF};
  static constexpr uint32_t kPduPayloadLength{100u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  explicit SensorFeedback(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct E2E_Header e2e_header;
  /**
   * \brief LGP Version
   */
  uint32_t FeedBack_LgpVer;
  /**
   * \brief Central Vehicle Time
   * \note Received by the Sensor from the Central Master EC
   * \note FeedBack_TimeS and FeedBack_TimeNs together represents the total Cental Vehicle Time
   * \note Unit: s
   */
  uint32_t FeedBack_TimeS;
  /**
   * \brief Central Vehicle Time
   * \note Received by the Sensor from the Central Master EC
   * \note FeedBack_TimeS and FeedBack_TimeNs together represents the total Cental Vehicle Time
   * \note Unit: ns
   */
  uint32_t FeedBack_TimeNs;
  struct MeasurementCycleSyncData measurement_cycle_sync_data;
  /**
   * \brief Time Synchronization Status
   * \note Bit 0  0: No Timeout on receiving Synchronisation Messages
   *              1: Timeout on receiving Synchronisation Messages
   *       Bit 1  Reserved always set to 0
   *       Bit 2  Sync to gateway
   *              0: Local Time Base is synchronous to Global Time Master
   *              1: Local Time Base updates are based on a Time Gateway below the Global Time
   *                 Master
   *       Bit 3  Global time base
   *              0: Local Time Base is based on Local Time Base reference clock only (never
   *                 synchronized with Global Time Base)
   *              1: Local Time Base was at least synchronized with Global Time Base one time
   *       Bit 4  Time leap future
   *              0: No leap into the future within the received time for Time Base
   *              1: Leap into the future within the received time for Time Base exceeds a
   *                 configured
   *       Bit 5  Time leap past
   *              0: No leap into the past within the received time for Time Base
   *              1: Leap into the past within the received time for Time Base exceeds a configured
   *       Bit 6  Reserved always set to 0
   *       Bit 7  Reserved always set to 0
   */
  uint8_t FeedBack_TimeSynSta;
  struct EgoVehicleData ego_vehicle_data;
  std::array<uint8_t, 54> FeedBack_Unassigned;
} __attribute__((packed));

static_assert(
  sizeof(SensorFeedback) == SensorFeedback::kPduSize,
  "Wrong SensorFeedback struct size!");

struct SenStInfo_SwNu_Int
{
  /**
   * \brief Software version - Internal release revision
   * \note  Format: 0xVVWWXXYYZZ
   *          Byte0 ZZ: value 1
   *          Byte1 YY: value 2
   *          Byte2 XX: value 3
   *          Byte3 WW: value 4
   *          Byte4 VV: value 5
   *        Example for Release version(customer version and internal revision): V3.0.1_0339c69230
   *        Internal revision 0339c69230: 0x0339c69230
   */
  std::array<uint8_t, 5> CommitId;
} __attribute__((packed));

struct SensorStateData
{
  std::array<uint8_t, 9> SenStInfo_Unassigned1;
  /**
   * \brief Sensor State
   * \note Refer Diagnostic_Specification_Document.pdf for detailed use cases
   * \note  1   In Specification
   *        2 - 50  Reserved
   *        51  Temporary Software Application Failure
   *        52  Temporary Software Application Mismatch
   *        53  Temporary Software Application Execution Continued
   *        54  Temporary Radar MMIC Mismatch
   *        55  Temporary Radar Exec Mismatch
   *        56  Temporary Radar Communication Mismatch
   *        57  Temporary ASIC Hardware Mismatch
   *        58  Temporary ASIC Voltage Execution Continued
   *        59  Temporary ECU Voltage Execution Continued
   *        60  Temporary ECU Temperature Mismatch
   *        61  Temporary Uc Clock Execution Continued
   *        62   Temporary Uc Inter Com Mismatch
   *        63  Temporary Radar Mismatch
   *        64  Temporary Radar Execution Continued
   *        65  Temporary ASIC Mismatch
   *        66  Temporary ECU Mismatch
   *        67  Temporary Uc Mismatch
   *        68  Temporary Failure
   *        69  Temporary Mismatch
   *        70  Temporary Execution Continued
   *        71 - 99 Reserved
   *        100 Persistent Software Application Failure
   *        101 Persistent Radar MMIC Failure
   *        102 Persistent Radar RIF Failure
   *        103 Persistent Radar SPU Failure
   *        104 Persistent Radar Exec Failure
   *        105 Persistent Radar Com Failure
   *        106 Persistent ASIC Voltage Execution Continued
   *        107 Persistent UcClock Execution Continued
   *        108 Persistent Radar Failure
   *        109 Persistent Radar Execution Continued
   *        110 Persistent ASIC Failure
   *        111 Persistent ECU Failure
   *        112 Persistent Uc Failure
   *        113 Persistent Failure
   *        114 Persistent Mismatch
   *        115 Persistent Execution Continued
   *        116 - 239 Reserved
   *        240 Shutdown
   *        241 - 254 Reserved
   *        255 Invalid
   */
  uint8_t SenStInfo_SenSt;
  /**
   * \brief Software version - Customer release version
   * \note Format: 0xWWXXYYZZ
   *         Byte1 ZZ: Patch Version,
   *         Byte2 YY: Minor Version,
   *         Byte3 XX: Major Version,
   *         Byte4 WW: Fixed to 0x00
   *       Example for customer version V3.0.1: 0x00030001
   */
  uint32_t SenStInfo_SwNu_Cust;
  struct SenStInfo_SwNu_Int sen_st_info_sw_nu_int;
} __attribute__((packed));

struct SensorStateInformation
{
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x1338DDCF};
  static constexpr uint32_t kPduPayloadLength{64u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  explicit SensorStateInformation(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct E2E_Header e2e_header;
  /**
   * \brief LGP Version
   */
  uint32_t SenStInfo_LgpVer;
  struct SensorStateData sensor_state_data;
  std::array<uint8_t, 29> SenStInfo_Unassigned;
} __attribute__((packed));

static_assert(
  sizeof(SensorStateInformation) == SensorStateInformation::kPduSize,
  "Wrong SensorStateInformation struct size!");

struct SensorEthernetConfigurationInformation
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief IP address by the Sensor to send/receive all the data to/from it except Diagnostic
   * related
   * \note Example : If IP address is 10.250.50.20 then signal value is 0x0AFA3210
   */
  uint32_t BroadCast_SenIpAdd;
  /**
   * \brief IP address by the destination (other ECU or gateway)
   * \note Example : If IP address is 10.250.50.20 then signal value is 0x0AFA3210
   */
  uint32_t BroadCast_DestIpAdd;
  /**
   * \brief Netmask used for communication except for Diagnostics
   * \note IP Address Range 1: 10.0.0.1 to 10.255.255.254 (Net Mask - FF00 0000)
   *       IP Address Range 2: 172.16.0.1 to 172.31.255.254 (Net Mask - FFF0 0000)
   *       IP Address Range 3: 192.168.0.1 to 192.168.255.254 (Net Mask - FFFF 0000)
   */
  uint32_t BroadCast_SenNetmask;
  /**
   * \brief VLAN by the Sensor to Send/receive all the data to/from it except Diagnostic related
   * \note If no VLAN is used then 0xFFFF is sent
   */
  uint16_t BroadCast_SenVlan;
  /**
   * \brief Port number by the Sensor to Send/receive all the data to/from it except Diagnostic
   * related
   */
  uint16_t BroadCast_SouPort;
  /**
   * \brief 0
   */
  uint32_t BroadCast_SouPortUnassigned;
  /**
   * \brief Port number by the destination(other ECU or gateway)
   */
  uint16_t BroadCast_DestPort;
  /**
   * \brief 0
   */
  uint32_t BroadCast_DestPortUnassigned;
} __attribute__((packed));

struct DignosticsEthernetConfigurationInformation
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief IP address by the Sensor to Send/receive all Diagnostic related information
   * \note Example : If IP address is 192.250.50.20 then signal value is 0xC0FA3210
   */
  uint32_t BroadCast_DiagSouIpAdd;
  /**
   * \brief netmask for Diagnostics communication
   * \note IP Address Range 4: 169.254.0.1 to 169.254.255.254 (Net Mask - FFFF 0000)
   */
  uint32_t BroadCast_DiagNetmask;
  /**
   * \brief VLAN by the Sensor to Send/receive all Diagnostic related information
   */
  uint16_t BroadCast_DiagVlan;
  /**
   * \brief Port number by the Sensor to Send/receive all Diagnostic related information
   */
  uint16_t BroadCast_DiagPort;
} __attribute__((packed));

struct DoIPInformation
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief DoIP Physical Address by the Sensor to Send/receive all Diagnostic related information
   */
  uint16_t BroadCast_SenDoIPPhyAdd;
  /**
   * \brief DoIP Functional Address by the Sensor to Send/receive all Diagnostic related information
   * \note Always Fixed to 0xFFFF
   */
  uint16_t BroadCast_SenDoIPFuncAdd;
  /**
   * \brief DoIP Target Address
   */
  uint16_t BroadCast_DoIPTarAdd;
} __attribute__((packed));

struct SensorBroadcastData
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Software version - Customer release version
   * \note Format: 0xWWXXYYZZ
   *         Byte1 ZZ: Patch Version,
   *         Byte2 YY: Minor Version,
   *         Byte3 XX: Major Version,
   *         Byte4 WW: Fixed to 0x00
   *       Example for customer version V3.0.1: 0x00030001
   */
  uint32_t BroadCast_SwCust;
  struct SensorEthernetConfigurationInformation sensor_ethernet_configuration_information;
  struct DignosticsEthernetConfigurationInformation dignostics_ethernet_configuration_information;
  /**
   * \brief MAC address by the Sensor to Send/receive all the data to/from.
   * \note Example : If MAC address is 00:1B:44:11:3A:B7 then signal value is 0x0000001B44113AB7
   */
  uint64_t BroadCast_SenMacAd;
  struct DoIPInformation doip_information;
} __attribute__((packed));

struct SensorBroadcast
{
  static constexpr uint32_t kPduId{0x133CDDCF};
  static constexpr uint32_t kPduPayloadLength{160u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  explicit SensorBroadcast(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  /**
   * \brief LGP Version
   */
  uint32_t BroadCast_LgpVer;
  struct SensorBroadcastData sensor_broadcast_data;
  std::array<uint8_t, 100> BroadCast_Unassigned;
} __attribute__((packed));

static_assert(
  sizeof(SensorBroadcast) == SensorBroadcast::kPduSize,
  "Wrong SensorBroadcast struct size!");

/**
 * \brief Location Attributes Header
 * \note i indicates the PDU number
 */
struct LocAtr_Header_i
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief LGP Version
   * \note Format: 0xWWXXYYZZ
   *         Byte1 ZZ: Minor Version,
   *         Byte2 YY: Major Version,
   *         Byte3 XX: Fixed to 0x00,
   *         Byte4 WW: Fixed to 0x00
   *       Example for "LGPONETH_V3.1": 0x00000301
   */
  uint32_t LocAtr_LgpVer;
  /**
   * \brief Block Counter
   * \note Shall increment for every transmission cycle, the PDU’s with the same signal value
   *       indicates they are measured at the same modulation time, when signal reaches maximum
   *       value counter shall reset
   */
  uint8_t LocAtr_BlockCounter;
  /**
   * \brief Start of the measurement Time Stamp (seconds)
   * \note Total time is the sum of LocAtr_TimeSts and LocAtr_TimeStns
   * \note Unit: s
   */
  uint32_t LocAtr_TimeSts;
  /**
   * \brief Start of the measurement Time Stamp (nano seconds)
   * \note Total time is the sum of LocAtr_TimeSts and LocAtr_TimeStns
   * \note Unit: ns
   */
  uint32_t LocAtr_TimeStns;
  /**
   * \brief Operation Mode
   * \note 13 - Initialization/Re-Initialize
   *       20 - Normal Mode
   *       22 - Modulation Off (high temperature)
   *       40 - Alignment Mode
   *       51 - Radar Off (Failure, over temperature)
   *       61 - Drive test
   *       100 - Error
   */
  uint8_t LocAtr_OpMode;
  /**
   * \brief Validity of the data measured
   * \note 0 - values of measurement in the current cycle invalid
   *       1 - values of measurement in the current cycle valid
   */
  uint8_t LocAtr_DataMeas;
  /**
   * \brief Reserved
   * \note Fixed value 0xFFFFFFFFFFFFFFFF
   */
  uint8_t LocAtr_Reserved[8];
} __attribute__((packed));

/**
 * \brief Sensor Modulation Performance packet
 */
struct SensorModulationPerformance
{
  static constexpr Range<uint8_t> r_LocAtr_DmpID{0u, 254u, 0xFF};
  static constexpr Range<uint16_t> r_LocAtr_ModID{0u, 1023u, 0xFFFF};
  static constexpr Range<float> r_LocAtr_DistRangScalFact{0.0F, 1.0F, NAN};
  static constexpr Range<float> r_LocAtr_SepRadDist{0.0F, 300.0F, NAN};
  static constexpr Range<float> r_LocAtr_SepRadVelo{0.0F, 10.0F, NAN};
  static constexpr Range<float> r_LocAtr_PrecRadDist{0.0F, 10.0F, NAN};
  static constexpr Range<float> r_LocAtr_PrecRadVelo{0.0F, 10.0F, NAN};
  static constexpr Range<float> r_LocAtr_RadDistVeloCovVar{-0.1F, 0.1F, NAN};
  static constexpr Range<float> r_LocAtr_MinRadDist{0.0F, 10.0F, NAN};
  static constexpr Range<float> r_LocAtr_MaxRadDist{0.0F, 300.0F, NAN};
  static constexpr Range<float> r_LocAtr_MinRadVelo{-150.0F, 0.0F, NAN};
  static constexpr Range<float> r_LocAtr_MaxRadVelo{0.0F, 150.0F, NAN};

  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  /**
   * \brief Detection of measurement program
   * \note See MeasurementProgram
   */
  uint8_t LocAtr_DmpID;
  /**
   * \brief Modulation ID used for the corresponding measurement program ID
   */
  uint16_t LocAtr_ModID;
  /**
   * \brief Distance Range Scaling Factor
   */
  float LocAtr_DistRangScalFact;
  /**
   * \brief Separability in distance of the locations/points which the sensor is capable of
   * detecting
   * \note Unit: m
   */
  float LocAtr_SepRadDist;
  /**
   * \brief Separability in relative velocity of the locations/points which the sensor is capable of
   * detecting
   * \note Unit: m/s
   */
  float LocAtr_SepRadVelo;
  /**
   * \brief Precision in distance of the locations/points which the sensor is capable of detecting
   * \note Unit: m
   */
  float LocAtr_PrecRadDist;
  /**
   * \brief Precision in relative velocity of the locations/points which the sensor is capable of
   * detecting
   * \note Unit: m/s
   */
  float LocAtr_PrecRadVelo;
  /**
   * \brief Covariance of distance and relative velocity of the locations/points which the sensor is
   * capable of detecting
   * \note Unit: m^2/s
   */
  float LocAtr_RadDistVeloCovVar;
  /**
   * \brief Minimal measurable distance of the locations/points which the sensor is capable of
   * detecting
   * \note Unit: m
   */
  float LocAtr_MinRadDist;
  /**
   * \brief Maximal measurable distance of the locations/points which the sensor is capable of
   * detecting
   * \note Unit: m
   */
  float LocAtr_MaxRadDist;
  /**
   * \brief Minimal measurable relative velocity of the locations/points which the sensor is capable
   * of detecting
   * \note Unit: m/s
   */
  float LocAtr_MinRadVelo;
  /**
   * \brief Maximal measurable relative velocity of the locations/points which the sensor is capable
   * of detecting
   * \note Unit: m/s
   */
  float LocAtr_MaxRadVelo;
} __attribute__((packed));

/**
 * \brief Misalignment Packet
 * \note Misalignment Packet signals will send the valid value when the below pre-conditions
 *       are satisfied:
 *         Ego Vehicle is received by the Sensor
 *         Sensor Detects at-least 8 to 10 Moving locations (preferably more locations)
 *         Sensor Detects at-least 8 to 10 Stationary locations (preferably more locations)
 */
struct Misalignment
{
  static constexpr Range<float> r_LocAtr_ThetaMalAng{-3.141592654F, 3.141592654F, NAN};
  static constexpr Range<float> r_LocAtr_ThetaMalAngVar{std::numeric_limits<float>::min(),
    std::numeric_limits<float>::max(), NAN};
  static constexpr Range<float> r_LocAtr_PhiMalAng{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<float> r_LocAtr_PhiMalAngVar{std::numeric_limits<float>::min(),
    std::numeric_limits<float>::max(), NAN};
  static constexpr Range<float> r_LocAtr_PhiMalAngEme{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<float> r_LocAtr_PhiMalAngEmeVar{std::numeric_limits<float>::min(),
    std::numeric_limits<float>::max(), NAN};
  static constexpr Range<uint16_t> r_LocAtr_MalStatus{0u, 255u, 0xFFFF};
  static constexpr Range<uint16_t> r_LocAtr_MalStatusEme{0u, 255u, 0xFFFF};
  static constexpr Range<float> r_LocAtr_PercNegativeTheta{0.0F, 100.0F, NAN};
  static constexpr Range<float> r_LocAtr_MinThetaMalSOs{-3.141592654F, 3.141592654F, NAN};
  static constexpr Range<float> r_LocAtr_MaxThetaMalSOs{-3.141592654F, 3.141592654F, NAN};
  static constexpr Range<float> r_LocAtr_VarThetaMalSOs{std::numeric_limits<float>::min(),
    std::numeric_limits<float>::max(), NAN};
  static constexpr Range<float> r_LocAtr_MeanThetaMalSOs{-3.141592654F, 3.141592654F, NAN};
  static constexpr Range<float> r_LocAtr_MinPhiMalSOs{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<float> r_LocAtr_MaxPhiMalSOs{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<float> r_LocAtr_VarPhiMalSOs{std::numeric_limits<float>::min(),
    std::numeric_limits<float>::max(), NAN};
  static constexpr Range<float> r_LocAtr_MeanPhiMalSOs{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<float> r_LocAtr_SpreadPhiMalSOs{-0.785398163F, 0.785398163F, NAN};
  static constexpr Range<uint16_t> r_LocAtr_NumSOs{0u, 1023u, 0xFFFF};
  static constexpr Range<uint16_t> r_LocAtr_NumEmeLocs{0u, 1023u, 0xFFFF};

  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  /**
   * \brief Estimated Misalignment angle in Azimuth (Spherical Coordinates)
   * \note Unit: rad
   */
  float LocAtr_ThetaMalAng;
  /**
   * \brief Variance of Estimated Misalignment angle in Azimuth (Spherical Coordinates)
   * \note Unit: rad^2
   */
  float LocAtr_ThetaMalAngVar;
  /**
   * \brief Estimated Misalignment angle in Elevation (Spherical Coordinates)
   * \note Unit: rad
   */
  float LocAtr_PhiMalAng;
  /**
   * \brief Variance of Estimated Misalignment angle in Elevation (Spherical Coordinates)
   * \note Unit: rad^2
   */
  float LocAtr_PhiMalAngVar;
  /**
   * \brief Estimated Misalignment angle in Elevation (EME) (Spherical Coordinates)
   * \note Unit: rad
   */
  float LocAtr_PhiMalAngEme;
  /**
   * \brief Variance of Estimated Misalignment angle in Elevation (EME) (Spherical Coordinates)
   * \note Unit: rad^2
   */
  float LocAtr_PhiMalAngEmeVar;
  /**
   * \brief Status of Estimation
   */
  uint16_t LocAtr_MalStatus;
  /**
   * \brief Status of EME based Estimation
   */
  uint16_t LocAtr_MalStatusEme;
  /**
   * \brief Percentage of standing locations selected by MAL on one side of the radar axis (<0° in
   * Azimuth)
   * \note Unit: %
   */
  float LocAtr_PercNegativeTheta;
  /**
   * \brief Minimal Theta angle of selected SOs used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MinThetaMalSOs;
  /**
   * \brief Maximum Theta angle of selected SOs used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MaxThetaMalSOs;
  /**
   * \brief Variance of Theta selected SOs angles used by doppler MAL algorithm
   * \note Unit: rad^2
   */
  float LocAtr_VarThetaMalSOs;
  /**
   * \brief Mean of Theta selected SOs angles used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MeanThetaMalSOs;
  /**
   * \brief Minimal Phi angle of selected SOs used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MinPhiMalSOs;
  /**
   * \brief Maximum Phi angle of selected SOs used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MaxPhiMalSOs;
  /**
   * \brief Variance of Phi selected SOs angles used by doppler MAL algorithm
   * \note Unit: rad^2
   */
  float LocAtr_VarPhiMalSOs;
  /**
   * \brief Mean of Phi selected SOs angles used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_MeanPhiMalSOs;
  /**
   * \brief Spread indicator of Phi selected SOs angles used by doppler MAL algorithm
   * \note Unit: rad
   */
  float LocAtr_SpreadPhiMalSOs;
  /**
   * \brief Number of selected SOs used by doppler MAL algorithm
   */
  uint16_t LocAtr_NumSOs;
  /**
   * \brief Number of selected locations used by EME algorithm
   */
  uint16_t LocAtr_NumEmeLocs;
} __attribute__((packed));

/**
 * \brief Location Attributes Header
 * \note i indicates the PDU number
 */
struct InterferenceIndicator
{
  static constexpr Range<float> r_LocAtr_FovRedInt{0.0F, 1.0F, NAN};
  static constexpr Range<uint8_t> r_LocAtr_IntStat{0u, 2u, 0xFF};

  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  /**
   * \brief FoV reduction due to interference
   * \note  1.00 No interference found, no degradation
   *        0.00 Completely blind due to interference
   */
  float LocAtr_FovRedInt;
  /**
   * \brief Status of interference indicator
   * \note  0 Invalid
   *        1 Valid and no interference found
   *        2 Valid and interference found
   */
  uint8_t LocAtr_IntStat;
} __attribute__((packed));

/**
 * \brief Location Attributes Header
 * \note i indicates the PDU number
 */
struct SensorFieldOfView
{
  static constexpr Range<float> r_LocAtr_FoVRange{0.0F, 511.0F, NAN};
  static constexpr Range<float> r_LocAtr_AziAngArr{-1.5708F, 1.5708F, NAN};
  static constexpr Range<float> r_LocAtr_RangScaEle{0.0F, 1.0F, NAN};
  static constexpr Range<float> r_LocAtr_EleAngArr{-0.785398F, 0.785398F, NAN};

  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  /**
   * \brief Field of view
   * \note FoV range for 10m^2 target at azimuth angle (cone angle) thetaViewAry[x], Elevation angle
   *       is 0 of the sensor
   * \note Unit: m
   */
  std::array<float, 25> LocAtr_FoVRange;
  /**
   * \brief Azimuth angle array (cone angle) for FoV
   * \note Unit: rad
   */
  std::array<float, 25> LocAtr_AziAngArr;
  /**
   * \brief Range scaling for elevation angle (cone angle)
   */
  std::array<float, 11> LocAtr_RangScaEle;
  /**
   * \brief Elevation angle array (cone angle) for FoV
   * \note Unit: rad
   */
  std::array<float, 11> LocAtr_EleAngArr;
} __attribute__((packed));

/**
 * \brief Location Attributes Packet
 * \note i indicates the PDU number
 */
struct LocAttributes_Packet
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Check signals in struct for range and replace with SNA if out of range
   */
  void check();

  struct SensorModulationPerformance sensor_modulation_performance;
  struct Misalignment misalignment;
  struct InterferenceIndicator interference_indicator;
  struct SensorFieldOfView sensor_field_of_view;
  std::array<uint8_t, 50> LocAtr_CoatingIndicationRes;   // 0xFF (all 50 Bytes)
} __attribute__((packed));

/**
 * \brief Location Attributes Mounting Position
 */
struct LocAtr_MountingPosition
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Sensor mounting position for "X" axis
   * \note  Default value 0m rad is sent when no Mounting position is written into the sensor
   * \note Unit: m
   */
  float LocAtr_SenPosX;
  /**
   * \brief Sensor mounting position for "Y" axis
   * \note  Default value 0m is sent when no Mounting position is written into the sensor
   * \note Unit: m
   */
  float LocAtr_SenPosY;
  /**
   * \brief Sensor mounting position for "Z" axis
   * \note  Default value 0m is sent when no Mounting position is written into the sensor
   * \note Unit: m
   */
  float LocAtr_SenPosZ;
  /**
   * \brief Sensor mounting angle for Azimuth
   * \note  Default value 0 rad is sent when no Mounting position is written into the sensor
   * \note Unit: rad
   */
  float LocAtr_SenPosAzi;
  /**
   * \brief Sensor mounting angle for Elevation
   * \note  Default value 0 rad is sent when no Mounting position is written into the sensor
   * \note Unit: rad
   */
  float LocAtr_SenPosEle;
  /**
   * \brief Sensor orientation
   * \note -1 and +1 are only the valid values and 0 is not used
   *       +1: nominal : 0° Azimuth and Elevation are the sensor normal, positive Azimuth Angles
   *       are measured, left from the sensor normal, positive Elevation Angles are measured
   *       upwards from the sensor normal. Connector right side / right back side
   *       -1: 0° Azimuth and Elevation are the sensor normal, positive Azimuth Angles are
   *       measured, right from the sensor normal, positive Elevation Angles are measured
   *       downwards from the sensor normal. Connector left side / left back side
   *       Default value 1 is sent when no Mounting position is written into the sensor
   */
  int8_t LocAtr_SenOrient;
} __attribute__((packed));

struct LocationAttributes
{
  static constexpr uint32_t kPduId{0x133BDDCF};
  static constexpr uint32_t kPduPayloadLength{514u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  explicit LocationAttributes(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct E2E_Header e2e_header;
  struct LocAtr_Header_i loc_atr_header;
  struct LocAttributes_Packet loc_atr_packet;
  struct LocAtr_MountingPosition loc_atr_mounting_position;
} __attribute__((packed));

static_assert(
  sizeof(LocationAttributes) == LocationAttributes::kPduSize,
  "Wrong LocationAttributes struct size!");


/**
  * \brief DTC Information Data
  * \note The signal indicates DTC and DTC status information.
  *       Based on the number of DTC’s set by the sensor, SensorDtc_Dtc## will be updated and
  *       rest will be set to 0.
  *       Example if 5 DTCs are set then SensorDtc_Dtc01 to SensorDtc_Dtc05 will be updated and
  *       SensorDtc_Dtc06 to SensorDtc_Dtc10 will have the data 0.
  */
struct DTCInformationData
{
  /**
  * \brief Sensor DTC
  * \note MSB 3 bytes represents the DTC information
  *       LSB Byte represents the DTC status information
  */
  std::array<uint32_t, 10> SensorDtc_Dtc;
} __attribute__((packed));

/**
  * \brief Sensor DTC Information
  * \note Below are the events in which the PDU will be transmitted by the sensor
  *         - When a new DTC is recorded
  *         - When a Active DTC becomes Passive or vice versa
  *         - When clear DTC diagnostic command is successfully executed
  */
struct SensorDTCInformation
{
  static constexpr uint32_t kPduId{0x133DDDCF};
  static constexpr uint32_t kPduPayloadLength{100u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  explicit SensorDTCInformation(const std::array<uint8_t, kPduSize> & buffer);

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct E2E_Header e2e_header;
  /**
 * \brief LGP Version
 * \note Format: 0xWWXXYYZZ
 *         Byte1 ZZ: Minor Version,
 *         Byte2 YY: Major Version,
 *         Byte3 XX: Fixed to 0x00,
 *         Byte4 WW: Fixed to 0x00
 *       Example for "LGPONETH_V3.1": 0x00000301
 */
  uint32_t SensorDtc_LgpVer;
  DTCInformationData dtc_information_data;
  /**
   * \brief Unassigned will be set to 0
   */
  std::array<uint8_t, 44> BroadCast_Unassigned;
} __attribute__((packed));

static_assert(
  sizeof(SensorDTCInformation) == SensorDTCInformation::kPduSize,
  "Wrong SensorDTCInformation struct size!");


struct VehicleData
{
  /**
   * \brief Relative Yaw Rate
   * \note Unit: deg/s
   */
  float EgoData_RelYawRate;
  /**
   * \brief Vehicle speed
   * \note Unit: m/s
   */
  float EgoData_VehSpd;
  /**
   * \brief Vehicle speed standard deviation
   * \note Unit: m/s
   */
  float EgoData_VehSpdStdDev;
  /**
   * \brief Longitudinal Acceleration
   * \note Unit: m/s^2
   */
  float EgoData_LogAcc;
} __attribute__((packed));

struct EgoVehicleInput
{
  static constexpr uint32_t kPduId{0x13370050};
  static constexpr uint32_t kPduPayloadLength{16u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct VehicleData vehicle_data;
} __attribute__((packed));

static_assert(
  sizeof(EgoVehicleInput) == EgoVehicleInput::kPduSize,
  "Wrong EgoVehicleInput struct size!");


struct MCSData
{
  /**
   * \brief Synchronisation Type
   * \note Set to 1 when we want to sync 2 or 3 sensors else will be set to 0 when no
   *       synchronization is required.
   *          0 No synchronization
   *          1 Timeslot sync
   */
  uint8_t MCS_SyncType;
  /**
   * \brief Sensor Time Offset
   * \note Used when we want to sync 2 or 3 sensors
   * \note If MCS_SyncType is set to 1 then MCS_SenTimeOff cannot be set to 0xFFFFFFFF
   * \note If 2 sensor needs to be synchronized then MCS_SyncType is set to 1 and
   *          MCS_SenTimeOff can be 0ms for 1st sensor, 33ms for 2nd sensor.
   * \note If 3 sensor needs to be synchronized then MCS_SyncType is set to 1 and
   *          MCS_SenTimeOff can be 0ms for 1st sensor, 22ms for 2nd sensor, 44ms for 3rd sensor.
   */
  uint32_t MCS_SenTimeOff;
} __attribute__((packed));

struct MeasurementCycleSynchronisation
{
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x13370051};
  static constexpr uint32_t kPduPayloadLength{5u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct MCSData mcs_data;
} __attribute__((packed));

static_assert(
  sizeof(MeasurementCycleSynchronisation) == MeasurementCycleSynchronisation::kPduSize,
  "Wrong MeasurementCycleSynchronisation(MCS) struct size!");


struct SensorModeData
{
  /**
   * \brief Radar Mode
   * \note Used start/stop modulation of the sensors
   *          0 Start Modulation
   *          1 Stop Modulation
   */
  uint8_t SenModReq_RadMod;
  std::array<uint8_t, 63> SenModReq_Unassigned;
} __attribute__((packed));

struct SensorModeRequest
{
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x13370052};
  static constexpr uint32_t kPduPayloadLength{64u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct SensorModeData sensor_mode_data;
} __attribute__((packed));

static_assert(
  sizeof(SensorModeRequest) == SensorModeRequest::kPduSize,
  "Wrong SensorModeRequest struct size!");

// +-------------------------+------------+--------------------+-------------------+-------------+
// | LocAtr_DmpID            | 01 (DMP01) |     02 (DMP02)     | 04 (DMP04)        | 00 (DMP00)  |
// | (Measurement Program)   |            |                    |                   |             |
// |-------------------------+------------+--------------------+-------------------+-------------|
// | Detection range         |    150     |         200        |     302           |     na      |
// | (d_max in m)            |            |                    |                   |             |
// |-------------------------+------------+--------------------+-------------------+-------------|
// | Velocity of the vehicle | <=Velocity | >Velocity range 1  | >Velocity range 2 |     na      |
// |                         |   range 1  | <=Velocity range 2 | < = 360KMPH       |             |
// |-------------------------+------------+--------------------+-------------------+-------------|
// | FoV                     | ±60°/±15°  |       ±60°/±15°    |     ±60°/±15°     | ±60°/±15°   |
// |-------------------------+------------+--------------------+-------------------+-------------|
// | LocAtr_ModID            | 448U,449U, |       452U,453U,   |   460U,461U,      |     na      |
// | (active modulation ID)  | 450U,451U  |       454U,455U    |   462U,463U       |             |
// +-------------------------+------------+--------------------+-------------------+-------------+
// Velocity of the vehicle
//   Velocity range 1 : 65KMPH
//   Velocity range 2 : 115KMPH
// The Velocity ranges can be changed using the DID 0x60C using DMP05
// (Refer Bosch_Variant_Handling.pdf chapter 3.6. Measurement program)
// When sensor is using DMP00, the signals LocAtr_DmpID and LocAtr_ModID are updated based on
// velocity of the vehicle
// For velocity of the vehicle >360KMPH the sensor Modulation stops

struct MeasurementProgramData
{
  /**
   * \brief Measurement program ID
   * \note Used to set/fix the Measurement program ID of the sensors
   *          00  DMP00
   *          01  DMP01
   *          02  DMP02
   *          04  DMP04
   */
  uint16_t MeasPgm_ID;
  std::array<uint8_t, 10> MeasPgm_Unassigned;
} __attribute__((packed));

struct MeasurementProgram
{
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x13370053};
  static constexpr uint32_t kPduPayloadLength{12u};
  static constexpr uint32_t kPduSize{kPduPayloadLength + kPduHeaderLength};

  /**
   * \brief PDU ID
   */
  uint32_t pdu_id;
  /**
   * \brief PDU Payload length
   */
  uint32_t pdu_payload_length;
  struct MeasurementProgramData measurement_program_data;
} __attribute__((packed));

static_assert(
  sizeof(MeasurementProgram) == MeasurementProgram::kPduSize,
  "Wrong MeasurementProgram struct size!");

}  // namespace off_highway_premium_radar_sample
