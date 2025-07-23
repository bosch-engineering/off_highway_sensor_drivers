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

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace off_highway_premium_radar
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
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

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

using Locations = std::vector<LocData_Packet_i_j>;

struct LocationData
{
  E2E_Header e2e_header;
  LocData_Header_i header;
  Locations locations;
};

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
  /**
   * \brief Sensor State
   * \note  Bit field indicating the following information:
   *          Bit0: init phase
   *          Bit1: operational
   *          Bit2: time synchronization status
   *          Bit3: misalignment estimation status
   *          Bit4: measurement program input
   *          Bit5: unavailability
   *          Bit6: failure
   *          Bit7: unused
   */
  uint8_t SenStInfo_SenSt;
} __attribute__((packed));

struct SensorStateInformation
{
  /**
  * \brief Serialize members into byte vector with correct byte order
  */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x1338DDCF};
  static constexpr uint32_t kPduPayloadLength{13u};
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
  struct SensorStateData sensor_state_data;
} __attribute__((packed));

static_assert(
  sizeof(SensorStateInformation) == SensorStateInformation::kPduSize,
  "Wrong SensorStateInformation struct size!");

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
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

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
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

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
  /**
   * \brief Misalignment Estimated Quality
   */
  float LocAtr_MalEstQuality;
} __attribute__((packed));

/**
 * \brief Interference Indicator Packet
 */
struct InterferenceIndicator
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

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
 * \brief Field Of View Packet
 */
struct SensorFieldOfView
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Field of view
   * \note FoV range for 10m^2 target at azimuth angle (cone angle) thetaViewAry[x], Elevation angle
   *       is 0 of the sensor
   * \note Unit: m
   */
  std::array<float, 99> LocAtr_FoVRange;
  /**
   * \brief Azimuth angle array (cone angle) for FoV
   * \note Unit: rad
   */
  std::array<float, 99> LocAtr_AziAngArr;
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
 * \brief Sensor Coating Packet
 * \note Contains blindness indicators
 */
struct SensorCoating
{
  /**
   * \brief Convert content from big endian to host byte order (for each member)
   */
  void betoh();

  /**
   * \brief Average azimuth angle quality for MIMO angles
   */
  float mdThetaIndcrMIMO;
  /**
   * \brief Validity average azimuth angle quality for MIMO angles
   */
  uint8_t mdThetaIndcrMIMOVldFlg;
  /**
   * \brief High elevation quality share
   */
  float mdPhiIndcr;
  /**
   * \brief Validity high elevation quality share
   */
  uint8_t mdPhiIndcrVldFlg;
  /**
   * \brief Number of valid reflections
   */
  float nRefIndcr;
  /**
   * \brief Validity number of valid reflections
   */
  uint8_t nRefIndcrVldFlg;
  /**
   * \brief Quotient of high quality angle fits divided by the number of all available MIMO angle
   *        fits
   */
  float thetaMIMORate;
  /**
   * \brief Validity quotient of high quality angle fits divided by the number of all available MIMO
   *        angle fits
   */
  uint8_t thetaMIMORteVldFlag;
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

  struct SensorModulationPerformance sensor_modulation_performance;
  struct Misalignment misalignment;
  struct InterferenceIndicator interference_indicator;
  struct SensorFieldOfView sensor_field_of_view;
  struct SensorCoating sensor_coating;
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
  /**
   * \brief Serialize members into byte vector with correct byte order
   */
  std::vector<uint8_t> serialize();

  static constexpr uint32_t kPduId{0x133BDDCF};
  static constexpr uint32_t kPduPayloadLength{1080u};
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

  EgoVehicleInput() = default;
  explicit EgoVehicleInput(const std::array<uint8_t, kPduSize> & buffer);

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


// +-----------------------+------------+--------------------+------------+------------+-----------+
// | LocAtr_DmpID          | 01 (DMP01) |     02 (DMP02)     | 03 (DMP03) | 04 (DMP04) | 00 (DMP00)|
// | (Measurement Program) |            |                    |            |            |           |
// |-----------------------+------------+--------------------+------------+------------+-----------+
// | Detection range       |    150     |         200        |     250    |    302     |     na    |
// | (d_max in m)          |            |                    |            |            |           |
// |-----------------------+------------+--------------------+------------+------------+-----------+
// | Velocity of the       | <=Velocity | >Velocity range 1  |            | >Velocity  |           |
// | vehicle               |   range 1  | <=Velocity range 2 |     na     |  range 2   |     na    |
// |                       |            |                    |            | <=360KMPH  |           |
// |-----------------------+------------+--------------------+------------+------------+-----------+
// | FoV                   | ±60°/±15°  |      ±60°/±15°     | ±60°/±15°  |  ±60°/±15° | ±60°/±15° |
// |-----------------------+------------+--------------------+------------+------------+-----------+
// | LocAtr_ModID          | 448U,449U  |      452U,453U     | 456U,457U, |  460U,461U |     na    |
// |                       |            |                    | 458U,459U  |            |           |
// +-----------------------+------------+--------------------+------------+------------+-----------+
// Velocity of the vehicle
//   Velocity range 1 : 65KMPH
//   Velocity range 2 : 115KMPH
// The Velocity ranges can be changed using the DID 0xFD03 using DMP05
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
   *          03  DMP03
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

  MeasurementProgram() = default;
  explicit MeasurementProgram(const std::array<uint8_t, kPduSize> & buffer);

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

}  // namespace off_highway_premium_radar
