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

#include "off_highway_premium_radar/converters/default_converter.hpp"

#include "ros_message_conversions.hpp"

namespace off_highway_premium_radar
{

DefaultConverter::DefaultConverter()
: diag_frequencies_locations_{11., 20.},  // 70ms +/- 20ms
  diag_frequencies_sensor_state_information_{90., 110.}  // 10ms +/- 10%
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
  publisher_sensor_state_information_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::SensorStateInformation>(
    "~/sensor_state_information", 10);
  publisher_location_attributes_ =
    node->create_publisher<off_highway_premium_radar_msgs::msg::LocationAttributes>(
    "~/location_attributes", 10);

  diag_locations_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_locations_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(&diag_frequencies_locations_.min, &diag_frequencies_locations_.max),
    TimeStampStatusParam(diag_timestamps_.min, diag_timestamps_.max));
  diag_sensor_state_information_ = std::make_shared<diagnostic_updater::TopicDiagnostic>(
    publisher_sensor_state_information_->get_topic_name(), *diag_updater_,
    FrequencyStatusParam(
      &diag_frequencies_sensor_state_information_.min,
      &diag_frequencies_sensor_state_information_.max),
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

  measurement_program_service_ =
    node->create_service<off_highway_premium_radar_msgs::srv::MeasurementProgram>(
    "~/set_measurement_program",
    std::bind(
      &DefaultConverter::on_measurement_program, this, std::placeholders::_1,
      std::placeholders::_2));
}


void DefaultConverter::on_location_data(const LocationData & data)
{
  RCLCPP_INFO_ONCE(logger_, "Sensor is sending data.");
  RCLCPP_DEBUG_STREAM(logger_, "Received " << data.locations.size() << " locations.");

  auto stamp = decide_on_stamp(data.header.LocData_TimeSts_i, data.header.LocData_TimeStns_i);

  if (publisher_locations_->get_subscription_count()) {
    publisher_locations_->publish(to_msg(data.locations, stamp, frame_id_));
  }

  publish_tick_diag(data.header, publisher_locations_header_, diag_locations_, stamp);
}

void DefaultConverter::on_sensor_state_information(const SensorStateInformation & data)
{
  publish_tick_diag(data, publisher_sensor_state_information_, diag_sensor_state_information_);
}

void DefaultConverter::on_location_attributes(const LocationAttributes & data)
{
  auto stamp = decide_on_stamp(
    data.loc_atr_header.LocAtr_TimeSts,
    data.loc_atr_header.LocAtr_TimeStns);

  publish_tick_diag(data, publisher_location_attributes_, diag_location_attributes_, stamp);
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

  if (!node->has_parameter("send_ego_vehicle_data")) {
    node->declare_parameter("send_ego_vehicle_data", send_ego_vehicle_data_);
  }
  node->get_parameter("send_ego_vehicle_data", send_ego_vehicle_data_);
}

}  // namespace off_highway_premium_radar
