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

#include "off_highway_premium_radar/converters/default_converter.hpp"

// Needs to be in front of pcl includes for precompilation settings
#include "off_highway_premium_radar/converters/pcl_radar_point_type.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "ros_message_conversions.hpp"

namespace off_highway_premium_radar
{

DefaultConverter::DefaultConverter()
: diag_frequencies_locations_{13., 17.},
  diag_frequencies_sensor_feedback_{18., 22.},
  diag_frequencies_sensor_state_information_{90., 110.},
  diag_frequencies_sensor_broadcast_{0.5, 1.5}
{
}

void DefaultConverter::on_configure()
{
  using diagnostic_updater::FrequencyStatusParam;
  using diagnostic_updater::TimeStampStatusParam;

  declare_and_get_parameters();

  auto node = parent_.lock();

  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node);
  diag_updater_->setHardwareID(node->get_name());

  publisher_locations_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/locations", 10);
  publisher_locations_header_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::LocationDataHeader>(
    "~/locations_header", 10);
  publisher_sensor_feedback_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::SensorFeedback>(
    "~/sensor_feedback", 10);
  publisher_sensor_state_information_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::SensorStateInformation>(
    "~/sensor_state_information", 10);
  publisher_sensor_broadcast_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::SensorBroadcast>(
    "~/sensor_broadcast", 10);
  publisher_location_attributes_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::LocationAttributes>(
    "~/location_attributes", 10);
  // Event based publishers, frequency-based diagnosis not applicable
  publisher_sensor_dtc_information_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::SensorDtcInformation>(
    "~/sensor_dtc_information", 10);

  diag_locations_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_locations_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(&diag_frequencies_locations_.min, &diag_frequencies_locations_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));
  diag_sensor_feedback_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_sensor_feedback_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(
      &diag_frequencies_sensor_feedback_.min,
      &diag_frequencies_sensor_feedback_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));
  diag_sensor_state_information_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_sensor_state_information_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(
      &diag_frequencies_sensor_state_information_.min,
      &diag_frequencies_sensor_state_information_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));
  diag_sensor_broadcast_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_sensor_broadcast_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(
      &diag_frequencies_sensor_broadcast_.min,
      &diag_frequencies_sensor_broadcast_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));
  diag_location_attributes_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_location_attributes_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(&diag_frequencies_locations_.min, &diag_frequencies_locations_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));

  if (send_ego_vehicle_data_) {
    subscriber_ego_vehicle_input_ =
      node->create_subscription<off_highway_premium_radar_msgs::msg::EgoVehicleInput>(
      "~/ego_vehicle_data", 10,
      std::bind(&DefaultConverter::on_ego_vehicle_data, this, std::placeholders::_1));
  }

  if (synchronize_measurement_cycle_) {
    sensor_sync_timer_ =
      create_timer(
      node, clock_, kSendSyncPeriod,
      std::bind(&DefaultConverter::send_measurement_cycle_sync, this));
  }

  measurement_program_service_ =
    node->create_service<off_highway_premium_radar_msgs::srv::MeasurementProgram>(
    "~/set_measurement_program",
    std::bind(
      &DefaultConverter::on_measurement_program, this, std::placeholders::_1,
      std::placeholders::_2));
  sensor_mode_request_service_ =
    node->create_service<off_highway_premium_radar_msgs::srv::SensorModeRequest>(
    "~/request_sensor_mode",
    std::bind(
      &DefaultConverter::on_sensor_mode_request, this, std::placeholders::_1,
      std::placeholders::_2));
}


void DefaultConverter::on_location_data(const LocationData & data)
{
  RCLCPP_INFO_ONCE(logger_, "Sensor is sending data.");
  RCLCPP_DEBUG_STREAM(logger_, "Received " << data.locations.size() << " locations.");

  auto stamp = decide_on_stamp(data.header.LocData_TimeSts_i, data.header.LocData_TimeStns_i);

  if (publisher_locations_->get_subscription_count()) {
    pcl::PointCloud<PclPointLocation> locations_pcl;
    locations_pcl.is_dense = true;
    locations_pcl.header.frame_id = frame_id_;

    pcl_conversions::toPCL(stamp, locations_pcl.header.stamp);

    for (const auto & l : data.locations) {
      locations_pcl.emplace_back(l);
    }

    sensor_msgs::msg::PointCloud2 pointcloud2_msg;
    pcl::toROSMsg(locations_pcl, pointcloud2_msg);

    publisher_locations_->publish(pointcloud2_msg);
  }

  publish_tick_diag(data.header, publisher_locations_header_, diag_locations_, stamp);
}

void DefaultConverter::on_sensor_feedback(const SensorFeedback & data)
{
  auto stamp = decide_on_stamp(data.FeedBack_TimeS, data.FeedBack_TimeNs);

  publish_tick_diag(data, publisher_sensor_feedback_, diag_sensor_feedback_, stamp);
}

void DefaultConverter::on_sensor_state_information(const SensorStateInformation & data)
{
  publish_tick_diag(data, publisher_sensor_state_information_, diag_sensor_state_information_);
}

void DefaultConverter::on_sensor_broadcast(const SensorBroadcast & data)
{
  publish_tick_diag(data, publisher_sensor_broadcast_, diag_sensor_broadcast_);
}

void DefaultConverter::on_location_attributes(const LocationAttributes & data)
{
  auto stamp = decide_on_stamp(
    data.loc_atr_header.LocAtr_TimeSts,
    data.loc_atr_header.LocAtr_TimeStns);

  publish_tick_diag(data, publisher_location_attributes_, diag_location_attributes_, stamp);
}

void DefaultConverter::on_sensor_dtc_information(const SensorDTCInformation & data)
{
  // Event-based, so no diagnosis
  if (!publisher_sensor_dtc_information_->get_subscription_count()) {
    // Do not publish if no one is subscribed
    return;
  }

  auto msg = to_msg(data, clock_->now(), frame_id_);

  publisher_sensor_dtc_information_->publish(msg);
}


void DefaultConverter::on_ego_vehicle_data(
  const off_highway_premium_radar_msgs::msg::EgoVehicleInput::ConstSharedPtr & msg)
{
  // TODO(rcp1-beg) Check the x velocity range [-100, 100]?
  EgoVehicleInput d;
  try {
    d = from_msg(msg);
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Failed to convert ego vehicle data, will not be sent to sensor: " << e.what());
    return;
  }

  if (!sender_->send_ego_vehicle_data(d)) {
    RCLCPP_ERROR(logger_, "Failed to send all bytes of ego vehicle data to sensor.");
  }
}

void DefaultConverter::send_measurement_cycle_sync()
{
  MeasurementCycleSynchronisation sync;
  sync.mcs_data.MCS_SenTimeOff = sensor_time_offset_;
  sync.mcs_data.MCS_SyncType = true;

  sender_->send_measurement_cycle_sync(sync);
}

void DefaultConverter::on_sensor_mode_request(
  const off_highway_premium_radar_msgs::srv::SensorModeRequest::Request::SharedPtr request,
  off_highway_premium_radar_msgs::srv::SensorModeRequest::Response::SharedPtr response)
{
  response->success = false;

  auto d = from_srv(request);
  if (sender_->send_sensor_mode_request(d)) {
    response->success = true;
  }
}

void DefaultConverter::on_measurement_program(
  const off_highway_premium_radar_msgs::srv::MeasurementProgram::Request::SharedPtr request,
  off_highway_premium_radar_msgs::srv::MeasurementProgram::Response::SharedPtr response)
{
  response->success = false;

  auto d = from_srv(request);
  if (sender_->send_measurement_program(d)) {
    response->success = true;
  }
}

void DefaultConverter::declare_and_get_parameters()
{
  auto node = parent_.lock();

  if (!node->has_parameter("synchronize_measurement_cycle")) {
    node->declare_parameter("synchronize_measurement_cycle", synchronize_measurement_cycle_);
  }
  node->get_parameter("synchronize_measurement_cycle", synchronize_measurement_cycle_);

  if (!node->has_parameter("sensor_time_offset")) {
    node->declare_parameter<int64_t>("sensor_time_offset", sensor_time_offset_);
  }
  int64_t sensor_time_offset{0};
  node->get_parameter("sensor_time_offset", sensor_time_offset);
  if (sensor_time_offset < 0) {
    throw std::out_of_range(
            "Parameter 'sensor_time_offset' negative: " +
            std::to_string(sensor_time_offset));
  }
  sensor_time_offset_ = sensor_time_offset;

  if (!node->has_parameter("send_ego_vehicle_data")) {
    node->declare_parameter("send_ego_vehicle_data", send_ego_vehicle_data_);
  }
  node->get_parameter("send_ego_vehicle_data", send_ego_vehicle_data_);
}

}  // namespace off_highway_premium_radar
