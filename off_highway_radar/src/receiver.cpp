// Copyright 2022 Robert Bosch GmbH and its subsidiaries
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

#include "off_highway_radar/receiver.hpp"

#include <regex>

#include "off_highway_common/helper.hpp"

#include "off_highway_radar/pcl_point_object.hpp"

namespace off_highway_radar
{

Receiver::Receiver()
: off_highway_common::Receiver()
{
  using off_highway_common::get_param_or_throw;

  pub_objects_ = private_nh_.advertise<Objects>("objects", 10);
  pub_objects_pcl_ = private_nh_.advertise<sensor_msgs::PointCloud2>("objects_pcl", 10);
  pub_info_ = private_nh_.advertise<Information>("info", 10);

  diag_task_ = std::make_shared<DiagTask>("radar", [this](auto & status) {diagnostics(status);});
  add_diag_task(diag_task_);

  allowed_age_ = get_param_or_throw<double>(private_nh_, "allowed_age");
  double publish_frequency = get_param_or_throw<double>(private_nh_, "publish_frequency");

  object_base_id_ = get_param_or_throw<FrameId>(private_nh_, "object_base_id");
  info_id_ = get_param_or_throw<FrameId>(private_nh_, "info_id");

  Receiver::start();

  publish_timer_ = private_nh_.createTimer(
    ros::Rate(publish_frequency), &Receiver::manage_and_publish, this);
}

Receiver::Messages Receiver::fillMessageDefinitions()
{
  Messages m;

  Message a;
  a.name = "Object_XX_Data_A";
  // Start bit, length, big endian, signed, factor, offset
  a.message_counter = {54, 2, false, false, 1, 0};
  a.signals["ID"] = {0, 6, false, false, 1, 0};
  a.signals["dx"] = {6, 13, false, false, 0.0625, -256};
  a.signals["dy"] = {19, 12, false, false, 0.0625, -128};
  a.signals["vx"] = {31, 10, false, false, 0.125, -64};
  a.signals["vy"] = {41, 10, false, false, 0.125, -64};
  a.signals["Meas"] = {51, 1, false, false, 1, 0};
  a.signals["Valid"] = {52, 1, false, false, 1, 0};
  a.signals["Hist"] = {53, 1, false, false, 1, 0};

  Message b;
  b.name = "Object_XX_Data_B";
  b.message_counter = {54, 2, false, false, 1, 0};
  b.signals["ID"] = {0, 6, false, false, 1, 0};
  b.signals["timeSinceMeas"] = {6, 13, false, false, 0.0001, 0};
  b.signals["Zone"] = {19, 5, false, false, 1, 0};
  b.signals["RCS"] = {24, 8, false, false, 0.5, -64};
  b.signals["Moving"] = {32, 1, false, false, 1, 0};
  b.signals["Near"] = {33, 1, false, false, 1, 0};
  b.signals["wExist"] = {34, 5, false, false, 0.03125, 0};

  Message info;
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
  info.signals["ReceptionError"] = {50, 1, false, false, 1, 0};

  // Fill message definitions
  for (uint8_t i = 0; i < kCountObjectMessages; ++i) {
    m[object_base_id_ + i] = i % 2 ? b : a;
    // Replace XX with index in message name
    auto & name = m[object_base_id_ + i].name;
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
    auto_static_cast(info_.reception_error, message.signals["ReceptionError"].value);

    if (pub_info_.getNumSubscribers() > 0) {
      pub_info_.publish(info_);
    }
    force_diag_update();
    return;
  }

  int32_t object_frame_id = id - object_base_id_;
  if (object_frame_id % 2) {
    ObjectB b;
    b.stamp = header.stamp;
    b.can_id = id;
    auto_static_cast(b.id, message.signals["ID"].value);
    auto_static_cast(b.time_since_meas, message.signals["timeSinceMeas"].value);
    auto_static_cast(b.zone, message.signals["Zone"].value);
    auto_static_cast(b.rcs, message.signals["RCS"].value);
    auto_static_cast(b.moving, message.signals["Moving"].value);
    auto_static_cast(b.near, message.signals["Near"].value);
    auto_static_cast(b.exist_probability, message.signals["wExist"].value);

    if (!objects_[b.id]) {
      return;
    }
    objects_[b.id]->b = b;
  } else {
    ObjectA a;
    a.stamp = header.stamp;
    a.can_id = id;
    auto_static_cast(a.id, message.signals["ID"].value);
    auto_static_cast(a.position.x, message.signals["dx"].value);
    auto_static_cast(a.position.y, message.signals["dy"].value);
    auto_static_cast(a.velocity.linear.x, message.signals["vx"].value);
    auto_static_cast(a.velocity.linear.y, message.signals["vy"].value);
    auto_static_cast(a.meas, message.signals["Meas"].value);
    auto_static_cast(a.valid, message.signals["Valid"].value);
    auto_static_cast(a.hist, message.signals["Hist"].value);

    if (!objects_[a.id]) {
      objects_[a.id] = Object();
    }
    objects_[a.id]->header = header;  // A message sets object age
    objects_[a.id]->a = a;
  }
}

bool Receiver::filter(const Object & object)
{
  return !object.a.valid;
}

void Receiver::manage_and_publish(const ros::TimerEvent &)
{
  manage_objects();
  publish_objects();
  publish_pcl();
}

void Receiver::manage_objects()
{
  auto & l = objects_;

  for (auto & o : l) {
    auto now = ros::Time::now();
    if (o && (filter(*o) || abs((now - o->header.stamp).toSec()) > allowed_age_)) {
      o = {};
    }
  }

  // Remove content of too old B messages
  for (auto & o : l) {
    auto now = ros::Time::now();
    if (o && abs((now - o->b.stamp).toSec()) > allowed_age_) {
      o->b = {};
      o->b.stamp = now;  // To not trigger each call
    }
  }
}

void Receiver::publish_objects()
{
  if (pub_objects_.getNumSubscribers() == 0) {
    return;
  }

  Objects msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = node_frame_id_;

  for (const auto & object : objects_) {
    if (object) {
      msg.objects.push_back(*object);
    }
  }

  pub_objects_.publish(msg);
}

void Receiver::publish_pcl()
{
  if (pub_objects_pcl_.getNumSubscribers() == 0) {
    return;
  }

  pcl::PointCloud<PclPointObject> objects_pcl;
  objects_pcl.is_dense = true;
  objects_pcl.header.frame_id = node_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), objects_pcl.header.stamp);

  for (const auto & object : objects_) {
    if (object) {
      objects_pcl.emplace_back(*object);
    }
  }

  pub_objects_pcl_.publish(objects_pcl);
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
  stat.add("Reception error", static_cast<bool>(info_.reception_error));

  bool not_ok = info_.sensor_blind || info_.sw_fail || info_.hw_fail ||
    info_.can_fail || info_.config_fail || info_.dtc ||
    info_.sensor_not_safe || info_.reception_error;

  if (not_ok) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}
}  // namespace off_highway_radar
