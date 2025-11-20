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

#include "off_highway_mm7p10/receiver.hpp"

#include <regex>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "off_highway_can/helper.hpp"

namespace off_highway_mm7p10
{

Receiver::Receiver(const rclcpp::NodeOptions & options)
: off_highway_can::Receiver("receiver", options)
{
  declare_and_get_parameters();

  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  pub_info_ = create_publisher<Information>("info", 10);

  diag_task_ = std::make_shared<DiagTask>(
    "mm7p10", [this](auto & status) {diagnostics(status);});
  add_diag_task(diag_task_);

  Receiver::start();
}

Receiver::Messages Receiver::fillMessageDefinitions()
{
  Messages m;

  Message z_ay;
  z_ay.name = "MM7_10_TX1_174_z_ay";
  z_ay.length = 8;
  z_ay.crc_index = std::nullopt;
  z_ay.signals["YawRate"] = {0, 16, false, false, 0.005, -163.84};
  z_ay.signals["YawRate_STAT"] = {20, 4, false, false, 1, 0};
  z_ay.signals["TEMP_RATE_Z"] = {24, 8, false, false, 1, -50};
  z_ay.signals["AY"] = {32, 16, false, false, 0.000127465, -4.1768};
  z_ay.signals["MSG_CNT_TX1"] = {48, 4, false, false, 1, 0};
  z_ay.signals["AY_STAT"] = {52, 4, false, false, 1, 0};
  z_ay.signals["CRC_TX1"] = {56, 8, false, false, 1, 0};

  Message x_ax;
  x_ax.name = "MM7_10_TX2_178_x_ax";
  x_ax.length = 8;
  x_ax.crc_index = std::nullopt;
  x_ax.signals["RollRate"] = {0, 16, false, false, 0.005, -163.84};
  x_ax.signals["RollRate_STAT"] = {20, 4, false, false, 1, 0};
  x_ax.signals["AX"] = {32, 16, false, false, 0.000127465, -4.1768};
  x_ax.signals["MSG_CNT_TX2"] = {48, 4, false, false, 1, 0};
  x_ax.signals["AX_STAT"] = {52, 4, false, false, 1, 0};
  x_ax.signals["CRC_TX2"] = {56, 8, false, false, 1, 0};

  Message y_az;
  y_az.name = "MM7_10_TX3_17C_y_az";
  y_az.length = 8;
  y_az.crc_index = std::nullopt;
  y_az.signals["PitchRate"] = {0, 16, false, false, 0.005, -163.84};
  y_az.signals["HW_Index"] = {16, 4, false, false, 1, 0};
  y_az.signals["PitchRate_STAT"] = {20, 4, false, false, 1, 0};
  y_az.signals["AZ"] = {32, 16, false, false, 0.000127465, -4.1768};
  y_az.signals["MSG_CNT_TX3"] = {48, 4, false, false, 1, 0};
  y_az.signals["AZ_STAT"] = {52, 4, false, false, 1, 0};
  y_az.signals["CRC_TX3"] = {56, 8, false, false, 1, 0};

  // Fill message definitions
  m[MM7_10_Z_AY_ID] = z_ay;
  m[MM7_10_X_AX_ID] = x_ax;
  m[MM7_10_Y_AZ_ID] = y_az;

  return m;
}

void Receiver::process(std_msgs::msg::Header header, const FrameId & id, Message & message)
{
  using off_highway_can::auto_static_cast;

  if (id == MM7_10_Z_AY_ID) {
    current_can_frame_msg_counter_ = static_cast<uint8_t>(message.signals["MSG_CNT_TX1"].value);
    frame_counter_ = 1;

    auto_static_cast(
      imu_.angular_velocity.z,
      message.signals["YawRate"].value * DEGREE_TO_RAD_FACTOR);
    auto_static_cast(imu_.linear_acceleration.y, message.signals["AY"].value * G_TO_MPS2_FACTOR);
    info_.yaw_rate_stat = static_cast<uint8_t>(message.signals["YawRate_STAT"].value);
    info_.ay_stat = static_cast<uint8_t>(message.signals["AY_STAT"].value);
    info_.temp_rate_z = static_cast<uint8_t>(message.signals["TEMP_RATE_Z"].value);
  }

  if (id == MM7_10_X_AX_ID) {
    uint8_t msg_counter = static_cast<uint8_t>(message.signals["MSG_CNT_TX2"].value);
    if (current_can_frame_msg_counter_ == msg_counter) {
      // Roll is received negative as per right-hand rule for rotation around X axis
      // The sign is inverted here to match ROS coordinate system
      double roll_rate;
      auto_static_cast(roll_rate, message.signals["RollRate"].value * DEGREE_TO_RAD_FACTOR);
      imu_.angular_velocity.x = -roll_rate;
      auto_static_cast(imu_.linear_acceleration.x, message.signals["AX"].value * G_TO_MPS2_FACTOR);
      info_.roll_rate_stat = static_cast<uint8_t>(message.signals["RollRate_STAT"].value);
      info_.ax_stat = static_cast<uint8_t>(message.signals["AX_STAT"].value);
      frame_counter_++;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Message counter mismatch in TX2: expected %d, got %d",
        current_can_frame_msg_counter_, msg_counter);
    }
  }

  if (id == MM7_10_Y_AZ_ID) {
    uint8_t msg_counter = static_cast<uint8_t>(message.signals["MSG_CNT_TX3"].value);
    if (current_can_frame_msg_counter_ == msg_counter) {
      if (frame_counter_ == 2) {
        imu_.header = header;
        auto_static_cast(
          imu_.angular_velocity.y,
          message.signals["PitchRate"].value * DEGREE_TO_RAD_FACTOR);
        auto_static_cast(
          imu_.linear_acceleration.z,
          message.signals["AZ"].value * G_TO_MPS2_FACTOR);
        info_.header = header;
        info_.pitch_rate_stat = static_cast<uint8_t>(message.signals["PitchRate_STAT"].value);
        info_.az_stat = static_cast<uint8_t>(message.signals["AZ_STAT"].value);
        info_.hw_index = static_cast<uint8_t>(message.signals["HW_Index"].value);
        manage_and_publish_imu();
        force_diag_update();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Missed CAN frames: expected 2, got %d", frame_counter_);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Message counter mismatch in TX3: expected %d, got %d",
        current_can_frame_msg_counter_, msg_counter);
    }
    frame_counter_ = 0;
  }

  return;
}

bool Receiver::filter(const sensor_msgs::msg::Imu & imu)
{
  // Check if all imu values are zero
  return !imu.angular_velocity.x && !imu.angular_velocity.y && !imu.angular_velocity.z &&
         !imu.linear_acceleration.x && !imu.linear_acceleration.y && !imu.linear_acceleration.z;
}

void Receiver::manage_and_publish_imu()
{
  // Check if IMU data is valid (not filtered out as all zeros) and within allowed age
  if (!filter(imu_) || abs((now() - imu_.header.stamp).seconds()) > allowed_age_) {
    publish_imu_data();
  }
}

void Receiver::publish_imu_data()
{
  publish_imu();
  publish_info();
}

void Receiver::publish_imu()
{
  if (pub_imu_->get_subscription_count() == 0) {
    return;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = node_frame_id_;

  msg.angular_velocity = imu_.angular_velocity;
  msg.linear_acceleration = imu_.linear_acceleration;
  msg.orientation_covariance[0] = -1.0;

  pub_imu_->publish(msg);
}

void Receiver::publish_info()
{
  if (pub_info_->get_subscription_count() == 0) {
    return;
  }

  info_.header.stamp = now();
  info_.header.frame_id = node_frame_id_;

  pub_info_->publish(info_);
}

void Receiver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  stat.add("Yaw rate stat", std::to_string(info_.yaw_rate_stat));
  stat.add("Roll rate stat", std::to_string(info_.roll_rate_stat));
  stat.add("Pitch rate stat", std::to_string(info_.pitch_rate_stat));
  stat.add("AX stat", std::to_string(info_.ax_stat));
  stat.add("AY stat", std::to_string(info_.ay_stat));
  stat.add("AZ stat", std::to_string(info_.az_stat));
  stat.add("Temp rate z", std::to_string(info_.temp_rate_z));
  stat.add("HW index", std::to_string(info_.hw_index));

  auto is_status_ok = [](uint8_t status) {return (status & STATUS_MASK) == 0;};

  bool sensors_ok = is_status_ok(info_.yaw_rate_stat) &&
    is_status_ok(info_.roll_rate_stat) &&
    is_status_ok(info_.pitch_rate_stat) &&
    is_status_ok(info_.ax_stat) &&
    is_status_ok(info_.ay_stat) &&
    is_status_ok(info_.az_stat);

  bool temp_ok = (info_.temp_rate_z != TEMP_INVALID_LOW && info_.temp_rate_z != TEMP_INVALID_HIGH);

  bool all_ok = sensors_ok && temp_ok;

  if (!all_ok) {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  } else {
    stat.summary(DiagnosticStatus::OK, "Ok");
  }
}

void Receiver::declare_and_get_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "CAN frame id of Z_AY message";
  declare_parameter<int>("mm7_10_z_ay_id", 0x174, param_desc);
  MM7_10_Z_AY_ID = get_parameter("mm7_10_z_ay_id").as_int();

  param_desc.description = "CAN frame id of X_AX message";
  declare_parameter<int>("mm7_10_x_ax_id", 0x178, param_desc);
  MM7_10_X_AX_ID = get_parameter("mm7_10_x_ax_id").as_int();

  param_desc.description = "CAN frame id of Y_AZ message";
  declare_parameter<int>("mm7_10_y_az_id", 0x17C, param_desc);
  MM7_10_Y_AZ_ID = get_parameter("mm7_10_y_az_id").as_int();

  param_desc.description =
    "Allowed age corresponding to output cycle time of sensor plus safety margin";
  declare_parameter<double>("allowed_age", 0.1, param_desc);
  allowed_age_ = get_parameter("allowed_age").as_double();
}

}  // namespace off_highway_mm7p10

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(off_highway_mm7p10::Receiver)
