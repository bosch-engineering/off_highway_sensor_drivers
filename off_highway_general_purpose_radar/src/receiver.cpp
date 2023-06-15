// Copyright 2023 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "off_highway_general_purpose_radar/receiver.hpp"

#include <regex>

#include "off_highway_common/helper.hpp"

#include "off_highway_general_purpose_radar/pcl_point_target.hpp"

namespace off_highway_general_purpose_radar
{

Receiver::Receiver()
: off_highway_common::Receiver()
{
  using off_highway_common::get_param_or_throw;

  pub_targets_ = private_nh_.advertise<Targets>("targets", 10);
  pub_targets_pcl_ = private_nh_.advertise<sensor_msgs::PointCloud2>("targets_pcl", 10);
  pub_info_ = private_nh_.advertise<Information>("info", 10);

  diag_task_ = std::make_shared<DiagTask>(
    "general_purpose_radar", [this](auto & status) {diagnostics(status);});
  add_diag_task(diag_task_);

  allowed_age_ = get_param_or_throw<double>(private_nh_, "allowed_age");
  double publish_frequency = get_param_or_throw<double>(private_nh_, "publish_frequency");

  target_base_id_ = get_param_or_throw<FrameId>(private_nh_, "target_base_id");
  info_id_ = get_param_or_throw<FrameId>(private_nh_, "info_id");

  Receiver::start();

  publish_timer_ = private_nh_.createTimer(
    ros::Rate(publish_frequency), &Receiver::manage_and_publish, this);
}

Receiver::Messages Receiver::fillMessageDefinitions()
{
  Messages m;

  Message a;
  a.name = "Target_XX_Data_A";
  a.crc_index = 7;
  // Start bit, length, big endian, signed, factor, offset
  a.message_counter = {54, 2, false, false, 1, 0};
  a.signals["AMessage"] = {0, 1, false, false, 1, 0};
  a.signals["ID"] = {1, 6, false, false, 1, 0};
  a.signals["dr"] = {7, 12, false, false, 0.0625, 0};
  a.signals["vr"] = {19, 12, false, true, 0.0625, 0};
  a.signals["dbPower"] = {31, 8, false, true, 0.5, 0};
  a.signals["phi"] = {39, 14, false, true, 0.0002, 0};
  a.signals["measured"] = {53, 1, false, false, 1, 0};

  Message b;
  b.crc_index = 7;
  b.name = "Target_XX_Data_B";
  b.message_counter = {54, 2, false, false, 1, 0};
  b.signals["AMessage"] = {0, 1, false, false, 1, 0};
  b.signals["ID"] = {1, 6, false, false, 1, 0};
  b.signals["phiSdv"] = {7, 6, false, false, 0.001, 0};
  b.signals["vrSdv"] = {13, 6, false, false, 0.0625, 0};
  b.signals["drSdv"] = {19, 6, false, false, 0.0625, 0};
  b.signals["pExist"] = {25, 5, false, false, 0.03125, 0};
  b.signals["timeSinceMeas"] = {30, 13, false, false, 0.0001, 0};

  Message info;
  info.crc_index = 7;
  info.name = "Info";
  info.message_counter = {54, 2, false, false, 1, 0};
  info.signals["SensorType"] = {0, 3, false, false, 1, 0};
  info.signals["HWTemperature"] = {3, 8, false, true, 1, 50};
  info.signals["SensorBlind"] = {11, 1, false, false, 1, 0};
  info.signals["SWFail"] = {12, 1, false, false, 1, 0};
  info.signals["HWFail"] = {13, 1, false, false, 1, 0};
  info.signals["CANFail"] = {14, 1, false, false, 1, 0};
  info.signals["ConfigFail"] = {15, 1, false, false, 1, 0};
  info.signals["DiagMode"] = {16, 1, false, false, 1, 0};
  info.signals["DTC"] = {17, 24, false, false, 1, 0};
  info.signals["DTCOrderId"] = {41, 8, false, false, 1, 0};
  info.signals["SensorNotSafe"] = {49, 1, false, false, 1, 0};

  // Fill message definitions
  for (uint8_t i = 0; i < kCountTargetMessages; ++i) {
    m[target_base_id_ + i] = i % 2 ? b : a;
    // Replace XX with index in message name
    auto & name = m[target_base_id_ + i].name;
    name = std::regex_replace(name, std::regex("XX"), std::to_string(i));
  }
  m[info_id_] = info;

  return m;
}

void Receiver::process(std_msgs::Header header, const FrameId & id, Message & message)
{
  using off_highway_common::auto_static_cast;

  if (id == info_id_) {
    info_.header = header;
    auto_static_cast(info_.sensor_type, message.signals["SensorType"].value);
    auto_static_cast(info_.hw_temperature, message.signals["HWTemperature"].value);
    auto_static_cast(info_.sensor_blind, message.signals["SensorBlind"].value);
    auto_static_cast(info_.sw_fail, message.signals["SWFail"].value);
    auto_static_cast(info_.hw_fail, message.signals["HWFail"].value);
    auto_static_cast(info_.can_fail, message.signals["CANFail"].value);
    auto_static_cast(info_.config_fail, message.signals["ConfigFail"].value);
    auto_static_cast(info_.diag_mode, message.signals["DiagMode"].value);
    auto_static_cast(info_.dtc, message.signals["DTC"].value);
    auto_static_cast(info_.dtc_order_id, message.signals["DTCOrderId"].value);
    auto_static_cast(info_.sensor_not_safe, message.signals["SensorNotSafe"].value);

    if (pub_info_.getNumSubscribers() > 0) {
      pub_info_.publish(info_);
    }
    force_diag_update();
    return;
  }

  bool is_a_message = static_cast<bool>(message.signals["AMessage"].value);
  int32_t target_frame_id = id - target_base_id_;
  if (target_frame_id % 2) {
    if (is_a_message) {
      return;
    }
    TargetB b;
    b.stamp = header.stamp;
    b.can_id = id;

    auto_static_cast(b.id, message.signals["ID"].value);
    auto_static_cast(b.time_since_meas, message.signals["timeSinceMeas"].value);
    auto_static_cast(b.azimuth_angle_std, message.signals["phiSdv"].value);
    auto_static_cast(b.radial_velocity_std, message.signals["vrSdv"].value);
    auto_static_cast(b.radial_distance_std, message.signals["drSdv"].value);
    auto_static_cast(b.exist_probability, message.signals["pExist"].value);

    if (!targets_[b.id]) {
      return;
    }
    targets_[b.id]->b = b;
  } else {
    if (!is_a_message) {
      return;
    }
    TargetA a;
    a.stamp = header.stamp;
    a.can_id = id;
    auto_static_cast(a.id, message.signals["ID"].value);
    auto_static_cast(a.radial_distance, message.signals["dr"].value);
    auto_static_cast(a.radial_velocity, message.signals["vr"].value);
    auto_static_cast(a.reflected_power, message.signals["dbPower"].value);
    auto_static_cast(a.azimuth_angle, message.signals["phi"].value);
    auto_static_cast(a.measured, message.signals["measured"].value);

    if (!targets_[a.id]) {
      targets_[a.id] = Target();
    }
    targets_[a.id]->header = header;  // A message sets target age
    targets_[a.id]->a = a;
  }
}

bool Receiver::filter(const Target & target)
{
  return !target.a.measured;
}

void Receiver::manage_and_publish(const ros::TimerEvent &)
{
  manage_targets();
  publish_targets();
  publish_pcl();
}

void Receiver::manage_targets()
{
  auto & list = targets_;

  for (auto & loc : list) {
    auto now = ros::Time::now();
    if (loc && (filter(*loc) || abs((now - loc->header.stamp).toSec()) > allowed_age_)) {
      loc = {};
    }
  }

  // Remove content of too old B messages
  for (auto & loc : list) {
    auto now = ros::Time::now();
    if (loc && abs((now - loc->b.stamp).toSec()) > allowed_age_) {
      loc->b = {};
      loc->b.stamp = now;  // To not trigger each call
    }
  }
}

void Receiver::publish_targets()
{
  if (pub_targets_.getNumSubscribers() == 0) {
    return;
  }

  Targets msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = node_frame_id_;

  for (const auto & target : targets_) {
    if (target) {
      msg.targets.push_back(*target);
    }
  }

  pub_targets_.publish(msg);
}

void Receiver::publish_pcl()
{
  if (pub_targets_pcl_.getNumSubscribers() == 0) {
    return;
  }

  pcl::PointCloud<PclPointTarget> targets_pcl;
  targets_pcl.is_dense = true;
  targets_pcl.header.frame_id = node_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), targets_pcl.header.stamp);

  for (const auto & target : targets_) {
    if (target) {
      targets_pcl.emplace_back(*target);
    }
  }

  pub_targets_pcl_.publish(targets_pcl);
}

void Receiver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;

  stat.add("Sensor type", std::to_string(info_.sensor_type));
  stat.add("HW temperature", info_.hw_temperature);
  stat.add("Sensor blind", static_cast<bool>(info_.sensor_blind));
  stat.add("SW fail", static_cast<bool>(info_.sw_fail));
  stat.add("HW fail", static_cast<bool>(info_.hw_fail));
  stat.add("CAN fail", static_cast<bool>(info_.can_fail));
  stat.add("Config fail", static_cast<bool>(info_.config_fail));
  stat.add("Diag mode", static_cast<bool>(info_.diag_mode));
  stat.add("Dtc", info_.dtc);
  stat.add("Sensor not safe", static_cast<bool>(info_.sensor_not_safe));

  bool not_ok = info_.sensor_blind || info_.sw_fail || info_.hw_fail ||
    info_.can_fail || info_.config_fail || info_.dtc ||
    info_.sensor_not_safe;

  if (not_ok) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}
}  // namespace off_highway_general_purpose_radar
