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

#include "off_highway_premium_radar/pdu_definitions.hpp"

// Our PCL for location points will contain more than 20 fields, thus we need to enable a boost mpl
// vector with more than 20 type template arguments.
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#if defined(BOOST_MPL_LIMIT_VECTOR_SIZE) && BOOST_MPL_LIMIT_VECTOR_SIZE < 30
#undef BOOST_MPL_LIMIT_VECTOR_SIZE
#endif
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"
#include "pcl/register_point_struct.h"

namespace off_highway_premium_radar
{
/**
 * \brief Location as PCL point
 */
struct EIGEN_ALIGN16 PclPointLocation
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float radial_distance{0.};
  float radial_velocity{0.};
  float azimuth_angle{0.};
  float elevation_angle{0.};
  float radar_cross_section{0.};
  float signal_noise_ratio{0.};
  float radial_distance_variance{0.};
  float radial_velocity_variance{0.};
  float azimuth_angle_variance{0.};
  float elevation_angle_variance{0.};
  float radial_distance_velocity_covariance{0.};
  float velocity_resolution_processing_probability{0.};
  float azimuth_angle_probability{0.};
  float elevation_angle_probability{0.};
  float measurement_status{0.};
  float idx_azimuth_ambiguity_peer{0.};

  /**
   * \brief Construct a new PclPointLocation object from a location data packet
   *
   * \param l Location data packet input
   */
  explicit PclPointLocation(const LocData_Packet_i_j & l)
  {
    const float & phi = l.LocData_EleAng_i_j;
    const float & theta = l.LocData_AziAng_i_j;

    x = l.LocData_RadDist_i_j * cos(theta) * cos(phi);
    y = l.LocData_RadDist_i_j * sin(theta) * cos(phi);
    z = l.LocData_RadDist_i_j * sin(phi);
    normal_x = l.LocData_RadRelVel_i_j * x / l.LocData_RadDist_i_j;
    normal_y = l.LocData_RadRelVel_i_j * y / l.LocData_RadDist_i_j;
    normal_z = l.LocData_RadRelVel_i_j * z / l.LocData_RadDist_i_j;
    radial_distance = l.LocData_RadDist_i_j;
    radial_velocity = l.LocData_RadRelVel_i_j;
    azimuth_angle = theta;
    elevation_angle = phi;
    radar_cross_section = l.LocData_Rcs_i_j;
    signal_noise_ratio = l.LocData_Snr_i_j;
    radial_distance_variance = l.LocData_RadDistVar_i_j;
    radial_velocity_variance = l.LocData_RadRelVelVar_i_j;
    azimuth_angle_variance = l.LocData_VarEle_i_j;
    elevation_angle_variance = l.LocData_VarAzi_i_j;
    radial_distance_velocity_covariance = l.LocData_DistVelCov_i_j;
    velocity_resolution_processing_probability = l.LocData_ProVelRes_i_j;
    azimuth_angle_probability = l.LocData_ProAziAng_i_j;
    elevation_angle_probability = l.LocData_ProEleAng_i_j;
    measurement_status = l.LocData_MeasStat_i_j;
    idx_azimuth_ambiguity_peer = l.LocData_IdAngAmb_i_j;
  }
  PclPointLocation() {}
};  // SSE padding

}  // namespace off_highway_premium_radar

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(off_highway_premium_radar::PclPointLocation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (float, radial_distance, radial_distance)
                                  (float, radial_velocity, radial_velocity)
                                  (float, azimuth_angle, azimuth_angle)
                                  (float, elevation_angle, elevation_angle)
                                  (float, radar_cross_section, radar_cross_section)
                                  (float, signal_noise_ratio, signal_noise_ratio)
                                  (float, radial_distance_variance, radial_distance_variance)
                                  (float, radial_velocity_variance, radial_velocity_variance)
                                  (float, azimuth_angle_variance, azimuth_angle_variance)
                                  (float, elevation_angle_variance, elevation_angle_variance)
                                  (float, radial_distance_velocity_covariance,
                                    radial_distance_velocity_covariance)
                                  (float, velocity_resolution_processing_probability,
                                    velocity_resolution_processing_probability)
                                  (float, azimuth_angle_probability, azimuth_angle_probability)
                                  (float, elevation_angle_probability, elevation_angle_probability)
                                  (float, measurement_status, measurement_status)
                                  (float, idx_azimuth_ambiguity_peer, idx_azimuth_ambiguity_peer))
/* *INDENT-ON* */
