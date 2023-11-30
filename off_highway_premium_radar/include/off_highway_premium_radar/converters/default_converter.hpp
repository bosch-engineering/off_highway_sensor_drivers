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

#include <memory>
#include <string>
#include <chrono>

#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "off_highway_premium_radar_msgs/msg/location_attributes.hpp"
#include "off_highway_premium_radar_msgs/msg/location_data_header.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_broadcast.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_dtc_information.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_feedback.hpp"
#include "off_highway_premium_radar_msgs/msg/sensor_state_information.hpp"

#include "off_highway_premium_radar_msgs/msg/ego_vehicle_input.hpp"
#include "off_highway_premium_radar_msgs/srv/measurement_program.hpp"
#include "off_highway_premium_radar_msgs/srv/sensor_mode_request.hpp"

#include "off_highway_premium_radar/interface/converter.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Default Converter publishing locations as point cloud and reset as 1:1 mapping.
 */
class DefaultConverter : public Converter
{
public:
  /**
   * \brief Construct a new DefaultConverter object
   */
  DefaultConverter();

  /**
   * \brief Configure object (read parameters, create publishers / subscribers)
   */
  void on_configure() override;

private:
  // Publish
  /**
   * \brief Called from receiving thread on receiving a full location data measurement
   *
   * \param data Location data measurement (in host order)
   */
  void on_location_data(const LocationData & data) override;

  /**
   * \brief Called from receiving thread on receiving a sensor feedback PDU
   *
   * \param data Sensor feedback PDU (in host order)
   */
  void on_sensor_feedback(const SensorFeedback & data) override;

  /**
   * \brief Called from receiving thread on receiving a sensor information PDU
   *
   * \param data Sensor information PDU (in host order)
   */
  void on_sensor_state_information(const SensorStateInformation & data) override;

  /**
   * \brief Called from receiving thread on receiving a sensor broadcast PDU
   *
   * \param data Sensor broadcast PDU (in host order)
   */
  void on_sensor_broadcast(const SensorBroadcast & data) override;

  /**
   * \brief Called from receiving thread on receiving a location attributes PDU
   *
   * \param data Location attributes PDU (in host order)
   */
  void on_location_attributes(const LocationAttributes & data) override;

  /**
   * \brief Called from receiving thread on receiving a sensor DTC information PDU
   *
   * \param data Sensor information PDU (in host order)
   */
  void on_sensor_dtc_information(const SensorDTCInformation & data) override;

  /**
   * \brief Same as publish tick diag but using ROS time as timestamp.
   */
  template<typename Input, class Publisher, class TopicDiagnostic>
  void publish_tick_diag(const Input & i, const Publisher & p, const TopicDiagnostic & d)
  {
    publish_tick_diag(i, p, d, clock_->now());
  }

  /**
   * \brief Convert data to message, publish data and tick topic diagnosis
   *
   * Will not publish data if no subscriber is connected but will tick topic diagnosis nonetheless.
   *
   * \tparam Input Input data type
   * \tparam Publisher Publisher type
   * \tparam TopicDiagnostic Diagnostic type
   * \param i Input data
   * \param p Publisher
   * \param d Topic diagnosis
   * \param stamp Timestamp to use for message
   */
  template<typename Input, class Publisher, class TopicDiagnostic>
  void publish_tick_diag(
    const Input & i, const Publisher & p, const TopicDiagnostic & d,
    const rclcpp::Time & stamp)
  {
    d->tick(stamp);

    if (!p->get_subscription_count()) {
      // Do not publish if no one is subscribed
      return;
    }

    auto msg = to_msg(i, stamp, frame_id_);

    p->publish(msg);
  }

  // Subscribe
  /**
   * \brief Subscription callback on ego vehicle data
   */
  void on_ego_vehicle_data(
    const off_highway_premium_radar_msgs::msg::EgoVehicleInput::SharedPtr msg);

  /**
   * \brief Send measurement cycle sync (triggered by timer)
   */
  void send_measurement_cycle_sync();

  /**
   * \brief Service callback on sensor mode request
   */
  void on_sensor_mode_request(
    const off_highway_premium_radar_msgs::srv::SensorModeRequest::Request::SharedPtr request,
    off_highway_premium_radar_msgs::srv::SensorModeRequest::Response::SharedPtr response);

  /**
   * \brief Service callback on measurement program
   */
  void on_measurement_program(
    const off_highway_premium_radar_msgs::srv::MeasurementProgram::Request::SharedPtr request,
    off_highway_premium_radar_msgs::srv::MeasurementProgram::Response::SharedPtr response);

  /**
   * \brief Declare and get ROS parameters
   */
  void declare_and_get_parameters();

  //! Converter name
  std::string name_{"default_converter"};

  // Parameters
  //! Send sensor sync
  bool synchronize_measurement_cycle_{false};
  //! Sensor time offset for sync
  uint32_t sensor_time_offset_{0};
  //! Subscribe to and send ego vehicle data
  bool send_ego_vehicle_data_{false};

  //! Timer to send sensor sync
  rclcpp::TimerBase::SharedPtr sensor_sync_timer_;
  //! Sensor sync frequency (50 ms cycle time)
  static constexpr std::chrono::milliseconds kSendSyncPeriod{50};

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_locations_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::LocationDataHeader>::SharedPtr
    publisher_locations_header_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::SensorFeedback>::SharedPtr
    publisher_sensor_feedback_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::SensorStateInformation>::SharedPtr
    publisher_sensor_state_information_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::SensorBroadcast>::SharedPtr
    publisher_sensor_broadcast_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::LocationAttributes>::SharedPtr
    publisher_location_attributes_;
  rclcpp::Publisher<off_highway_premium_radar_msgs::msg::SensorDtcInformation>::SharedPtr
    publisher_sensor_dtc_information_;

  // Subscriptions
  rclcpp::Subscription<off_highway_premium_radar_msgs::msg::EgoVehicleInput>::SharedPtr
    subscriber_ego_vehicle_input_;

  // Services
  rclcpp::Service<off_highway_premium_radar_msgs::srv::MeasurementProgram>::SharedPtr
    measurement_program_service_;
  rclcpp::Service<off_highway_premium_radar_msgs::srv::SensorModeRequest>::SharedPtr
    sensor_mode_request_service_;

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

  using TopicDiagnosticSharedPtr = std::shared_ptr<diagnostic_updater::TopicDiagnostic>;
  // Output
  TopicDiagnosticSharedPtr diag_locations_;
  TopicDiagnosticSharedPtr diag_sensor_feedback_;
  TopicDiagnosticSharedPtr diag_sensor_state_information_;
  TopicDiagnosticSharedPtr diag_sensor_broadcast_;
  TopicDiagnosticSharedPtr diag_location_attributes_;
  // Input
  TopicDiagnosticSharedPtr diag_ego_vehicle_data_;

  struct Limit
  {
    double min;
    double max;
  };

  // Diagnosis timestamp and frequency limits (set in ctor)
  static constexpr Limit diag_timestamps_{0.0, 0.1};
  Limit diag_frequencies_locations_;
  Limit diag_frequencies_sensor_feedback_;
  Limit diag_frequencies_sensor_state_information_;
  Limit diag_frequencies_sensor_broadcast_;
  Limit diag_frequencies_ego_vehicle_data_;
};

}  // namespace off_highway_premium_radar
