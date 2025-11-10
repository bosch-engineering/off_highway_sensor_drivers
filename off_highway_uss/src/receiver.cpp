// Copyright 2022 Robert Bosch GmbH and its subsidiaries
// Copyright 2023 digital workbench GmbH
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

#include "off_highway_uss/receiver.hpp"

#include <regex>

#include "pcl_conversions/pcl_conversions.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "off_highway_can/helper.hpp"

#include "off_highway_uss/interpolate_line.hpp"
#include "off_highway_uss/pcl_point_object.hpp"

namespace off_highway_uss
{

Receiver::Receiver(const rclcpp::NodeOptions & options)
: off_highway_can::Receiver("receiver", options)
{
  declare_and_get_parameters();

  pub_objects_ = create_publisher<Objects>("objects", 10);
  pub_objects_pcl_ = create_publisher<sensor_msgs::msg::PointCloud2>("objects_pcl", 10);
  pub_direct_echos_ = create_publisher<DirectEchos>("direct_echos", 10);
  pub_max_detection_range_ = create_publisher<MaxDetectionRange>("max_detection_range", 10);
  pub_info_ = create_publisher<Information>("info", 10);

  diag_task_ = std::make_shared<DiagTask>("uss", [this](auto & status) {diagnostics(status);});
  add_diag_task(diag_task_);

  Receiver::start();

  publish_objects_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::duration<double>(1.0 / publish_frequency_),
    std::bind(&Receiver::manage_and_publish_objects, this)
  );

  publish_direct_echos_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::duration<double>(1.0 / publish_frequency_),
    std::bind(&Receiver::manage_and_publish_direct_echos, this)
  );
}

Receiver::Messages Receiver::fillMessageDefinitions()
{
  Messages m;

  Message object;
  object.name = "MAP_OBJ_XX";
  // Start bit, length, big endian, signed, factor, offset
  object.message_counter = {54, 2, false, false, 1, 0};
  // Lengths are directly converted to SI unit [m]
  object.signals["1stPointX"] = {0, 10, false, true, 2 * kCentimeterToMeter, 0};
  object.signals["1stPointY"] = {10, 10, false, true, 2 * kCentimeterToMeter, 0};
  object.signals["ExistProbability"] = {20, 3, false, false, 1, 0};
  object.signals["2ndPointX"] = {23, 10, false, true, 2 * kCentimeterToMeter, 0};
  object.signals["2ndPointY"] = {33, 10, false, true, 2 * kCentimeterToMeter, 0};
  object.signals["Multiplexor"] = {43, 1, false, false, 1, 0};
  object.signals["ObjectType"] = {44, 2, false, false, 1, 0};

  Message direct_echo;
  direct_echo.name = "DEMsg_Sens_XX";
  direct_echo.message_counter = {48, 2, false, false, 1, 0};
  direct_echo.signals["De1Distance"] = {0, 10, false, false, 1, 0};
  direct_echo.signals["De2Distance"] = {10, 10, false, false, 1, 0};
  direct_echo.signals["De1FilteredDistance"] = {20, 10, false, false, 1, 0};
  direct_echo.signals["Amplitude1"] = {30, 6, false, false, 1, 0};
  direct_echo.signals["Amplitude2"] = {36, 6, false, false, 1, 0};
  direct_echo.signals["FilteredAmplitude1"] = {42, 6, false, false, 1, 0};

  Message max_detection_range;
  max_detection_range.name = "USS_MaxDetRange";
  max_detection_range.message_counter = {48, 2, false, false, 1, 0};
  max_detection_range.signals["sens01_MaxDetRange"] =
  {0, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens02_MaxDetRange"] =
  {4, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens03_MaxDetRange"] =
  {8, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens04_MaxDetRange"] =
  {12, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens05_MaxDetRange"] =
  {16, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens06_MaxDetRange"] =
  {20, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens07_MaxDetRange"] =
  {24, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens08_MaxDetRange"] =
  {28, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens09_MaxDetRange"] =
  {32, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens10_MaxDetRange"] =
  {36, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens11_MaxDetRange"] =
  {40, 4, false, false, 36.667 * kCentimeterToMeter, 0};
  max_detection_range.signals["sens12_MaxDetRange"] =
  {44, 4, false, false, 36.667 * kCentimeterToMeter, 0};

  Message info;
  info.name = "Info";
  info.message_counter = {44, 2, false, false, 1, 0};
  info.signals["NumberSensors"] = {0, 4, false, false, 1, 0};
  info.signals["SendingPattern"] = {4, 2, false, false, 1, 0};
  info.signals["OperatingMode"] = {6, 3, false, false, 1, 0};
  info.signals["OutsideTemperature"] = {9, 8, false, false, 1, -40};
  info.signals["SensorBlindness"] = {17, 12, false, false, 1, 0};
  info.signals["Sensitivity"] = {29, 3, false, false, 1, 0};
  info.signals["SensorFaulted"] = {32, 12, false, false, 1, 0};
  info.signals["FailureStatus"] = {46, 1, false, false, 1, 0};

  // Fill message definitions
  for (uint8_t i = 0; i < kCountObjectFrames; ++i) {
    m[object_base_id_ + i] = object;
    // Replace XX with index in message name
    auto & name = m[object_base_id_ + i].name;
    name = std::regex_replace(name, std::regex("XX"), std::to_string(i));
  }
  for (uint8_t i = 0; i < kCountDirectEchos; ++i) {
    m[direct_echo_base_id_ + i] = direct_echo;
    // Replace XX with index in message name
    auto & name = m[direct_echo_base_id_ + i].name;
    name = std::regex_replace(name, std::regex("XX"), std::to_string(i));
  }
  m[max_detection_range_id_] = max_detection_range;
  m[info_id_] = info;

  return m;
}

void Receiver::process(std_msgs::msg::Header header, const FrameId & id, Message & message)
{
  using off_highway_can::auto_static_cast;

  if (id == info_id_) {
    info_.header = header;
    auto_static_cast(info_.number_sensors, message.signals["NumberSensors"].value);
    auto_static_cast(info_.sending_pattern, message.signals["SendingPattern"].value);
    auto_static_cast(info_.operating_mode, message.signals["OperatingMode"].value);
    auto_static_cast(info_.outside_temperature, message.signals["OutsideTemperature"].value);
    auto_static_cast(info_.sensor_blindness, message.signals["SensorBlindness"].value);
    auto_static_cast(info_.sensitivity, message.signals["Sensitivity"].value);
    auto_static_cast(info_.sensor_faulted, message.signals["SensorFaulted"].value);
    auto_static_cast(info_.failure_status, message.signals["FailureStatus"].value);
    if (pub_info_->get_subscription_count() > 0) {
      pub_info_->publish(info_);
    }
    force_diag_update();
    return;
  }

  if (id == max_detection_range_id_) {
    MaxDetectionRange max_detection_range;
    max_detection_range.header = header;
    auto_static_cast(
      max_detection_range.max_detection_ranges[0],
      message.signals["sens01_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[1],
      message.signals["sens02_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[2],
      message.signals["sens03_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[3],
      message.signals["sens04_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[4],
      message.signals["sens05_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[5],
      message.signals["sens06_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[6],
      message.signals["sens07_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[7],
      message.signals["sens08_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[8],
      message.signals["sens09_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[9],
      message.signals["sens10_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[10],
      message.signals["sens11_MaxDetRange"].value);
    auto_static_cast(
      max_detection_range.max_detection_ranges[11],
      message.signals["sens12_MaxDetRange"].value);
    if (pub_max_detection_range_->get_subscription_count() > 0) {
      pub_max_detection_range_->publish(max_detection_range);
    }
    return;
  }

  int32_t object_frame_id = id - object_base_id_;
  if (object_frame_id >= 0 && object_frame_id < kCountObjectFrames) {
    Object o;
    o.header = header;
    auto_static_cast(o.position_first.x, message.signals["1stPointX"].value);
    auto_static_cast(o.position_first.y, message.signals["1stPointY"].value);
    auto_static_cast(o.position_second.x, message.signals["2ndPointX"].value);
    auto_static_cast(o.position_second.y, message.signals["2ndPointY"].value);
    auto_static_cast(o.object_type, message.signals["ObjectType"].value);

    // Use discrete steps of exist probability
    uint8_t exist_probability_step =
      static_cast<uint8_t>(message.signals["ExistProbability"].value);
    o.exist_probability = kExistProbabilitySteps.at(exist_probability_step);

    uint32_t object_id = object_frame_id + kCountObjectFrames *
      message.signals["Multiplexor"].value;
    o.id = object_id;

    objects_[object_id] = o;
    return;
  }

  int32_t echo_id = id - direct_echo_base_id_;
  if (echo_id >= 0 && echo_id < kCountDirectEchos) {
    DirectEcho d;
    d.header = header;
    d.id = echo_id;
    auto_static_cast(d.first.distance, message.signals["De1Distance"].value);
    auto_static_cast(d.second.distance, message.signals["De2Distance"].value);
    auto_static_cast(d.first_filtered.distance, message.signals["De1FilteredDistance"].value);
    auto_static_cast(d.first.amplitude, message.signals["Amplitude1"].value);
    auto_static_cast(d.second.amplitude, message.signals["Amplitude2"].value);
    auto_static_cast(d.first_filtered.amplitude, message.signals["FilteredAmplitude1"].value);

    direct_echos_[echo_id] = d;
    return;
  }
}

bool Receiver::is_j1939_source_address_matching(uint8_t source_address)
{
  return source_address == j1939_source_address_;
}

bool Receiver::filter(const Object & object)
{
  return object.object_type == Object::TYPE_NONE || object.object_type == Object::TYPE_SNA;
}

void Receiver::manage_and_publish_objects()
{
  manage(objects_);
  publish_objects();
  publish_pcl();
}

void Receiver::publish_objects()
{
  if (pub_objects_->get_subscription_count() == 0 || is_timeout_) {
    return;
  }

  Objects msg;
  msg.header.stamp = now();
  msg.header.frame_id = node_frame_id_;

  for (const auto & object : objects_) {
    if (object) {
      msg.objects.push_back(*object);
    }
  }

  pub_objects_->publish(msg);
}

void Receiver::publish_pcl()
{
  if (pub_objects_pcl_->get_subscription_count() == 0 || is_timeout_) {
    return;
  }

  pcl::PointCloud<PclPointObject> objects_pcl;
  objects_pcl.is_dense = true;
  objects_pcl.header.frame_id = node_frame_id_;
  pcl_conversions::toPCL(now(), objects_pcl.header.stamp);

  uint8_t id = 0;
  for (const auto & object : objects_) {
    if (object) {
      switch (object->object_type) {
        case off_highway_uss_msgs::msg::Object::TYPE_POINT:
          objects_pcl.emplace_back(*object, id);
          break;
        case off_highway_uss_msgs::msg::Object::TYPE_LINE:
          {
            auto samples = interpolate_segment(
              object->position_first, object->position_second,
              line_sample_distance_
            );
            for (auto & sample : samples) {
              objects_pcl.emplace_back(sample, object->exist_probability, object->object_type, id);
            }
          }
          break;
        default:
          throw std::logic_error(
                  "USS type " + std::to_string(object->object_type) + " is not point or line!");
          break;
      }
    }
    ++id;
  }

  sensor_msgs::msg::PointCloud2 pointcloud2;
  pcl::toROSMsg(objects_pcl, pointcloud2);
  pub_objects_pcl_->publish(pointcloud2);
}

bool Receiver::filter(const DirectEcho &)
{
  return false;
}

void Receiver::manage_and_publish_direct_echos()
{
  manage(direct_echos_);
  publish_direct_echos();
}

void Receiver::publish_direct_echos()
{
  if (is_timeout_) {
    return;
  }

  DirectEchos msg;
  msg.header.stamp = now();
  msg.header.frame_id = node_frame_id_;

  for (const auto & direct_echo : direct_echos_) {
    if (direct_echo) {
      msg.direct_echos.push_back(*direct_echo);
    }
  }

  pub_direct_echos_->publish(msg);
}

void Receiver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  stat.add("Number sensors", std::to_string(info_.number_sensors));
  stat.add("Sending pattern", std::to_string(info_.sending_pattern));
  stat.add("Operating mode", std::to_string(info_.operating_mode));
  stat.add("Outside temperature", info_.outside_temperature);
  stat.add("Sensor blindness", std::to_string(info_.sensor_blindness));
  stat.add("Sensitivity", std::to_string(info_.sensitivity));
  stat.add("Sensor faulted", std::to_string(info_.sensor_faulted));
  stat.add("Failure status", static_cast<bool>(info_.failure_status));

  bool not_ok = info_.sensor_blindness || info_.sensor_faulted || info_.failure_status;

  if (not_ok) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}

void Receiver::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description =
    "Allowed age corresponding to output cycle time of sensor plus safety margin";
  declare_parameter<double>("allowed_age", 0.06, param_desc);
  allowed_age_ = get_parameter("allowed_age").as_double();

  param_desc.description =
    "Distance to sample segment of line objects for representing the segment in point "
    "cloud, zero distance would result in only using both end points";
  declare_parameter<double>("line_sample_distance", 0.1, param_desc);
  line_sample_distance_ = get_parameter("line_sample_distance").as_double();

  param_desc.description =
    "Frequency at which current object list (point cloud) and distance list is published. "
    "Corresponds to 40 ms USS sending cycle time.";
  declare_parameter<double>("publish_frequency", 25.0, param_desc);
  publish_frequency_ = get_parameter("publish_frequency").as_double();

  param_desc.description = "Use J1939 protocol instead of automotive CAN";
  declare_parameter<bool>("use_j1939", false, param_desc);
  use_j1939_ = get_parameter("use_j1939").as_bool();

  param_desc.description = "CAN frame id offset for functional frames";
  declare_parameter<int>("can.id_offset", 0x170, param_desc);
  can_id_offset_ = get_parameter("can.id_offset").as_int();

  param_desc.description = "J1939 parameter group number (PGN)";
  declare_parameter<int>("j1939.pgn_offset", 0xFF70, param_desc);
  j1939_pgn_offset_ = get_parameter("j1939.pgn_offset").as_int();

  param_desc.description = "J1939 source address";
  declare_parameter<int>("j1939.source_address", 0x98, param_desc);
  j1939_source_address_ = get_parameter("j1939.source_address").as_int();

  auto offset = use_j1939_ ? j1939_pgn_offset_ : can_id_offset_;
  direct_echo_base_id_ = offset + kDirectEchoBaseIdOffset;
  info_id_ = offset + kInfoIdOffset;
  max_detection_range_id_ = offset + kMaxDetectionRangeIdOffset;
  object_base_id_ = offset + kObjectBaseIdOffset;
}
}  // namespace off_highway_uss

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(off_highway_uss::Receiver)
