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

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "off_highway_premium_radar/interface/receiver.hpp"
#include "off_highway_premium_radar/interface/sender.hpp"

#include "rclcpp/rclcpp.hpp"

namespace off_highway_premium_radar
{

/**
 * \brief Converter interface to receive and send sensor data
 */
class Converter : public Receiver
{
public:
  using SharedPtr = std::shared_ptr<Converter>;

  /**
   * \brief Destructor
   */
  virtual ~Converter() = default;

  /**
   * \brief Configure converter with parent node and sender interface
   */
  virtual void configure(rclcpp::Node::WeakPtr parent, Sender::SharedPtr sender)
  {
    parent_ = parent;
    sender_ = sender;
    {
      auto node = parent_.lock();
      clock_ = node->get_clock();
      logger_ = node->get_logger();
    }
    declare_and_get_parameters();
    on_configure();
  }

  /**
   * \brief Callback that is called in configure
   */
  virtual void on_configure() = 0;

protected:
  /**
   * \brief Decide to use sensor or ROS time stamp and get it
   *
   * Sensor time may not be compatible since ROS uses int32 as type for the seconds in its time
   * message while the sensor uses uint32. This method falls back to ROS time in an incompatible
   * case.
   *
   * \param sensor_sec Sensor seconds
   * \param sensor_nanosec Sensor nanoseconds
   * \return Timestamp to use
   */
  rclcpp::Time decide_on_stamp(uint32_t sensor_sec, uint32_t sensor_nanosec)
  {
    bool sensor_time_compatible = sensor_sec <=
      std::numeric_limits<builtin_interfaces::msg::Time::_sec_type>::max();

    if (use_sensor_time_ && !sensor_time_compatible) {
      RCLCPP_ERROR_STREAM(
        logger_, "Trying to use time from sensor, but received seconds out of range for ROS: "
          << sensor_sec << " . Falling back to ROS time.");
    }

    return use_sensor_time_ && sensor_time_compatible ?
           rclcpp::Time(sensor_sec, sensor_nanosec) :
           clock_->now();
  }

  //! Parent node
  rclcpp::Node::WeakPtr parent_;
  //! Sender interface to send data to sensor
  Sender::SharedPtr sender_;
  //! Clock to use
  rclcpp::Clock::SharedPtr clock_;
  //! Logger to use
  rclcpp::Logger logger_{rclcpp::get_logger("Converter")};

  // ROS parameters
  //! Frame ID for messages
  std::string frame_id_{"base_link"};
  //! Stamp messages with time from sensor if possible for message or with ROS time
  bool use_sensor_time_{false};

private:
  /**
   * \brief Declare and get ROS parameters
   */
  void declare_and_get_parameters()
  {
    auto node = parent_.lock();
    if (!node->has_parameter("frame_id")) {
      node->declare_parameter("frame_id", frame_id_);
    }
    node->get_parameter("frame_id", frame_id_);

    if (!node->has_parameter("use_sensor_time")) {
      node->declare_parameter("use_sensor_time", use_sensor_time_);
    }
    node->get_parameter("use_sensor_time", use_sensor_time_);
  }
};

}  // namespace off_highway_premium_radar
