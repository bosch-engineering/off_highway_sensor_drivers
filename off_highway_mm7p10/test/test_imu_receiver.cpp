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

#include <random>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "test_helper.hpp"
#include "off_highway_mm7p10/receiver.hpp"
#include "off_highway_mm7p10_msgs/msg/information.hpp"
#include "off_highway_can/helper.hpp"

using off_highway_can::auto_static_cast;
using namespace std::chrono_literals;

// Conversion factors
static constexpr double G_TO_MPS2_FACTOR = 9.80665f;
static constexpr double DEGREE_TO_RAD_FACTOR = M_PI / 180;

class ImuPublisher : public rclcpp::Node
{
public:
  explicit ImuPublisher(
    sensor_msgs::msg::Imu test_imu,
    off_highway_mm7p10_msgs::msg::Information test_info,
    off_highway_can::Receiver::Messages & msg_def)
  : Node("off_highway_mm7p10_imu_pub"), msg_def_(msg_def)
  {
    // Initialize publisher
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 1);

    // Use consistent message counter across all messages for validation
    const uint8_t msg_counter = 1;

    // MM7P10 uses 3 CAN messages for complete IMU and Information data
    // Publish first CAN message (0x174) with yaw rate, ay, and information
    publish_imu_info_message(0x174, test_imu, test_info, msg_def, msg_counter);

    // Publish second CAN message (0x178) with roll rate, ax, and information
    publish_imu_info_message(0x178, test_imu, test_info, msg_def, msg_counter);

    // Publish third CAN message (0x17C) with pitch rate, az, and information
    publish_imu_info_message(0x17C, test_imu, test_info, msg_def, msg_counter);
  }

private:
  void publish_imu_info_message(
    uint32_t can_id,
    const sensor_msgs::msg::Imu & test_imu,
    const off_highway_mm7p10_msgs::msg::Information & test_info,
    off_highway_can::Receiver::Messages & msg_def,
    uint8_t msg_counter)
  {
    // Initialize can msg
    can_msgs::msg::Frame can_msg_imu_info;

    // Cast and encode IMU and Information values
    auto_static_cast(can_msg_imu_info.id, can_id);
    auto_static_cast(can_msg_imu_info.header.stamp, now());
    off_highway_can::Message & imu_info_msg = msg_def[can_msg_imu_info.id];

    if (can_id == 0x174) {
      // TX1: YawRate, AY, YawRate_STAT, AY_STAT, TEMP_RATE_Z
      auto_static_cast(imu_info_msg.signals["YawRate"].value, test_imu.angular_velocity.z);
      auto_static_cast(imu_info_msg.signals["AY"].value, test_imu.linear_acceleration.y);
      auto_static_cast(imu_info_msg.signals["YawRate_STAT"].value, test_info.yaw_rate_stat);
      auto_static_cast(imu_info_msg.signals["AY_STAT"].value, test_info.ay_stat);
      auto_static_cast(imu_info_msg.signals["TEMP_RATE_Z"].value, test_info.temp_rate_z);
      auto_static_cast(imu_info_msg.signals["MSG_CNT_TX1"].value, msg_counter);
    } else if (can_id == 0x178) {
      // TX2: RollRate, AX, RollRate_STAT, AX_STAT
      auto_static_cast(imu_info_msg.signals["RollRate"].value, test_imu.angular_velocity.x);
      auto_static_cast(imu_info_msg.signals["AX"].value, test_imu.linear_acceleration.x);
      auto_static_cast(imu_info_msg.signals["RollRate_STAT"].value, test_info.roll_rate_stat);
      auto_static_cast(imu_info_msg.signals["AX_STAT"].value, test_info.ax_stat);
      auto_static_cast(imu_info_msg.signals["MSG_CNT_TX2"].value, msg_counter);
    } else if (can_id == 0x17C) {
      // TX3: PitchRate, AZ, PitchRate_STAT, AZ_STAT, HW_Index
      auto_static_cast(imu_info_msg.signals["PitchRate"].value, test_imu.angular_velocity.y);
      auto_static_cast(imu_info_msg.signals["AZ"].value, test_imu.linear_acceleration.z);
      auto_static_cast(imu_info_msg.signals["PitchRate_STAT"].value, test_info.pitch_rate_stat);
      auto_static_cast(imu_info_msg.signals["AZ_STAT"].value, test_info.az_stat);
      auto_static_cast(imu_info_msg.signals["HW_Index"].value, test_info.hw_index);
      auto_static_cast(imu_info_msg.signals["MSG_CNT_TX3"].value, msg_counter);
    }

    // Encode message
    imu_info_msg.encode(can_msg_imu_info.data);

    // Publish
    publisher_->publish(can_msg_imu_info);
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  off_highway_can::Receiver::Messages & msg_def_;
};  // ImuPublisher

class ImuSubscriber : public rclcpp::Node
{
public:
  ImuSubscriber()
  : Node("off_highway_imu_sub")
  {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 2,
      std::bind(&ImuSubscriber::imu_callback, this, std::placeholders::_1));

    info_subscriber_ = this->create_subscription<off_highway_mm7p10_msgs::msg::Information>(
      "info", 2,
      std::bind(&ImuSubscriber::info_callback, this, std::placeholders::_1));
  }

  sensor_msgs::msg::Imu get_imu()
  {
    return received_imu_;
  }

  off_highway_mm7p10_msgs::msg::Information get_info()
  {
    return received_info_;
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu msg)
  {
    received_imu_ = msg;
  }

  void info_callback(const off_highway_mm7p10_msgs::msg::Information msg)
  {
    received_info_ = msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<off_highway_mm7p10_msgs::msg::Information>::SharedPtr info_subscriber_;
  sensor_msgs::msg::Imu received_imu_;
  off_highway_mm7p10_msgs::msg::Information received_info_;
};  // ImuSubscriber

class TestImuReceiver : public ::testing::Test
{
protected:
  void initialize_test()
  {
    // Overwrite allowed age to avoid timing issues in unit tests
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("allowed_age", 1.0)
    };

    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(params);
    node_ = std::make_shared<off_highway_mm7p10::Receiver>(node_options);

    ASSERT_EQ(node_->get_parameter("allowed_age").as_double(), 1.0);

    imu_subscriber_ = std::make_shared<ImuSubscriber>();
  }

  void publish_imu_and_info(
    sensor_msgs::msg::Imu imu,
    off_highway_mm7p10_msgs::msg::Information info);
  sensor_msgs::msg::Imu get_imu();
  off_highway_mm7p10_msgs::msg::Information get_info();
  void verify_imu_and_info(
    sensor_msgs::msg::Imu test_imu,
    sensor_msgs::msg::Imu received_imu,
    off_highway_mm7p10_msgs::msg::Information test_info,
    off_highway_mm7p10_msgs::msg::Information received_info);

private:
  void spin_receiver(const std::chrono::nanoseconds & duration);
  void spin_subscriber(const std::chrono::nanoseconds & duration);

  std::shared_ptr<off_highway_mm7p10::Receiver> node_;

  std::shared_ptr<ImuPublisher> imu_publisher_;
  std::shared_ptr<ImuSubscriber> imu_subscriber_;
};

void TestImuReceiver::publish_imu_and_info(
  sensor_msgs::msg::Imu imu,
  off_highway_mm7p10_msgs::msg::Information info)
{
  auto msg_def = node_->get_messages();
  imu_publisher_ = std::make_shared<ImuPublisher>(imu, info, msg_def);
  spin_receiver(60ms);
}

sensor_msgs::msg::Imu TestImuReceiver::get_imu()
{
  spin_subscriber(500ms);
  sensor_msgs::msg::Imu subscribed_imu = imu_subscriber_->get_imu();
  return subscribed_imu;
}

off_highway_mm7p10_msgs::msg::Information TestImuReceiver::get_info()
{
  spin_subscriber(500ms);
  off_highway_mm7p10_msgs::msg::Information subscribed_info = imu_subscriber_->get_info();
  return subscribed_info;
}

void TestImuReceiver::spin_receiver(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(node_);
  }
}

void TestImuReceiver::spin_subscriber(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= duration) {
    rclcpp::spin_some(imu_subscriber_);
    rclcpp::sleep_for(100ms);
  }
}

void TestImuReceiver::verify_imu_and_info(
  sensor_msgs::msg::Imu test_imu,
  sensor_msgs::msg::Imu received_imu,
  off_highway_mm7p10_msgs::msg::Information test_info,
  off_highway_mm7p10_msgs::msg::Information received_info)
{
  // Verify node connections
  EXPECT_EQ(node_->count_publishers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_subscribers("from_can_bus"), 1U);
  EXPECT_EQ(node_->count_publishers("imu"), 1U);
  EXPECT_EQ(node_->count_subscribers("imu"), 1U);
  EXPECT_EQ(node_->count_publishers("info"), 1U);
  EXPECT_EQ(node_->count_subscribers("info"), 1U);

  // Convert received IMU values back to test units for comparison
  received_imu.linear_acceleration.x = received_imu.linear_acceleration.x / G_TO_MPS2_FACTOR;
  received_imu.linear_acceleration.y = received_imu.linear_acceleration.y / G_TO_MPS2_FACTOR;
  received_imu.linear_acceleration.z = received_imu.linear_acceleration.z / G_TO_MPS2_FACTOR;
  received_imu.angular_velocity.x = -received_imu.angular_velocity.x / DEGREE_TO_RAD_FACTOR;
  received_imu.angular_velocity.y = received_imu.angular_velocity.y / DEGREE_TO_RAD_FACTOR;
  received_imu.angular_velocity.z = received_imu.angular_velocity.z / DEGREE_TO_RAD_FACTOR;

  // Check IMU data
  EXPECT_EQ(received_imu.header.frame_id, test_imu.header.frame_id);
  EXPECT_NEAR(received_imu.angular_velocity.x, test_imu.angular_velocity.x, 0.001);
  EXPECT_NEAR(received_imu.angular_velocity.y, test_imu.angular_velocity.y, 0.001);
  EXPECT_NEAR(received_imu.angular_velocity.z, test_imu.angular_velocity.z, 0.001);
  EXPECT_NEAR(received_imu.linear_acceleration.x, test_imu.linear_acceleration.x, 0.001);
  EXPECT_NEAR(received_imu.linear_acceleration.y, test_imu.linear_acceleration.y, 0.001);
  EXPECT_NEAR(received_imu.linear_acceleration.z, test_imu.linear_acceleration.z, 0.001);

  // Check Information data
  EXPECT_EQ(received_info.yaw_rate_stat, test_info.yaw_rate_stat);
  EXPECT_EQ(received_info.roll_rate_stat, test_info.roll_rate_stat);
  EXPECT_EQ(received_info.pitch_rate_stat, test_info.pitch_rate_stat);
  EXPECT_EQ(received_info.ax_stat, test_info.ax_stat);
  EXPECT_EQ(received_info.ay_stat, test_info.ay_stat);
  EXPECT_EQ(received_info.az_stat, test_info.az_stat);
  EXPECT_EQ(received_info.temp_rate_z, test_info.temp_rate_z);
  EXPECT_EQ(received_info.hw_index, test_info.hw_index);
}

TEST_F(TestImuReceiver, testImuAndInfoZero) {
  initialize_test();

  sensor_msgs::msg::Imu test_imu;
  test_imu.header.frame_id = "base_link";
  test_imu.angular_velocity = tf2::toMsg(tf2::Vector3(0.0, 0.0, 0.0));
  test_imu.linear_acceleration = tf2::toMsg(tf2::Vector3(0.0, 0.0, 0.0));

  off_highway_mm7p10_msgs::msg::Information test_info;
  test_info.yaw_rate_stat = 0;
  test_info.roll_rate_stat = 0;
  test_info.pitch_rate_stat = 0;
  test_info.ax_stat = 0;
  test_info.ay_stat = 0;
  test_info.az_stat = 0;
  test_info.temp_rate_z = 0;
  test_info.hw_index = 0;

  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());
}

TEST_F(TestImuReceiver, testImuAndInfoMin) {
  initialize_test();

  sensor_msgs::msg::Imu test_imu;
  test_imu.header.frame_id = "base_link";
  test_imu.angular_velocity = tf2::toMsg(tf2::Vector3(-163.84, -163.84, -163.84));
  test_imu.linear_acceleration = tf2::toMsg(tf2::Vector3(-4.1768, -4.1768, -4.1768));

  off_highway_mm7p10_msgs::msg::Information test_info;
  test_info.yaw_rate_stat = 1;
  test_info.roll_rate_stat = 2;
  test_info.pitch_rate_stat = 4;
  test_info.ax_stat = 0;
  test_info.ay_stat = 0;
  test_info.az_stat = 0;
  test_info.temp_rate_z = 0;
  test_info.hw_index = 1;

  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());
}

TEST_F(TestImuReceiver, testImuAndInfoMax) {
  initialize_test();

  sensor_msgs::msg::Imu test_imu;
  test_imu.header.frame_id = "base_link";
  test_imu.angular_velocity = tf2::toMsg(tf2::Vector3(163.83, 163.83, 163.83));
  test_imu.linear_acceleration = tf2::toMsg(tf2::Vector3(4.176618, 4.176618, 4.176618));

  off_highway_mm7p10_msgs::msg::Information test_info;
  test_info.yaw_rate_stat = 15;
  test_info.roll_rate_stat = 15;
  test_info.pitch_rate_stat = 15;
  test_info.ax_stat = 15;
  test_info.ay_stat = 15;
  test_info.az_stat = 15;
  test_info.temp_rate_z = 200;
  test_info.hw_index = 15;

  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());
}

TEST_F(TestImuReceiver, testImuAndInfoRandom) {
  initialize_test();

  std::default_random_engine rng;

  // Angular velocity: scale = 0.005, offset = -163.84, range = -163.84 to 163.835
  auto gen_angular_vel = RandomQuantizedGenerator{0.005, -163.84, 163.835};
  // Linear acceleration: scale = 0.000127465, offset = -4.1768, range = -4.1768 to 4.176618
  auto gen_linear_acc = RandomQuantizedGenerator{0.000127465, -4.1768, 4.176618};

  // Set random IMU values
  sensor_msgs::msg::Imu test_imu;
  test_imu.header.frame_id = "base_link";
  test_imu.angular_velocity = tf2::toMsg(
    tf2::Vector3(
      gen_angular_vel(rng),
      gen_angular_vel(rng),
      gen_angular_vel(rng)
  ));
  test_imu.linear_acceleration = tf2::toMsg(
    tf2::Vector3(
      gen_linear_acc(rng),
      gen_linear_acc(rng),
      gen_linear_acc(rng)
  ));

  // Set random Information values within valid ranges
  std::uniform_int_distribution<uint8_t> status_dist(0, 15);
  std::uniform_int_distribution<uint8_t> temp_dist(0, 200);
  std::uniform_int_distribution<uint8_t> hw_dist(0, 15);

  off_highway_mm7p10_msgs::msg::Information test_info;
  test_info.yaw_rate_stat = status_dist(rng);
  test_info.roll_rate_stat = status_dist(rng);
  test_info.pitch_rate_stat = status_dist(rng);
  test_info.ax_stat = status_dist(rng);
  test_info.ay_stat = status_dist(rng);
  test_info.az_stat = status_dist(rng);
  test_info.temp_rate_z = temp_dist(rng);
  test_info.hw_index = hw_dist(rng);

  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());
}

TEST_F(TestImuReceiver, testInvalidValues) {
  initialize_test();

  sensor_msgs::msg::Imu test_imu;
  test_imu.header.frame_id = "base_link";
  test_imu.angular_velocity = tf2::toMsg(tf2::Vector3(1.0, 1.0, 1.0));
  test_imu.linear_acceleration = tf2::toMsg(tf2::Vector3(1.0, 1.0, 1.0));

  off_highway_mm7p10_msgs::msg::Information test_info;

  test_info.yaw_rate_stat = 0;
  test_info.roll_rate_stat = 0;
  test_info.pitch_rate_stat = 0;
  test_info.ax_stat = 0;
  test_info.ay_stat = 0;
  test_info.az_stat = 0;
  test_info.hw_index = 5;

  test_info.temp_rate_z = off_highway_mm7p10::TEMP_INVALID_LOW;
  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());

  test_info.temp_rate_z = off_highway_mm7p10::TEMP_INVALID_HIGH;
  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());

  test_info.temp_rate_z = 100;
  const uint8_t imu_not_available =
    static_cast<uint8_t>(off_highway_mm7p10::StatusBits::IMU_NOT_AVAILABLE);
  const uint8_t signal_failure =
    static_cast<uint8_t>(off_highway_mm7p10::StatusBits::SIGNAL_FAILURE);
  const uint8_t init_in_progress =
    static_cast<uint8_t>(off_highway_mm7p10::StatusBits::INITIALIZATION_IN_PROGRESS);

  test_info.yaw_rate_stat = imu_not_available;
  test_info.roll_rate_stat = signal_failure;
  test_info.pitch_rate_stat = init_in_progress;
  test_info.ax_stat = off_highway_mm7p10::STATUS_MASK;
  test_info.ay_stat = signal_failure | imu_not_available;
  test_info.az_stat = init_in_progress;
  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());

  test_info.yaw_rate_stat = off_highway_mm7p10::STATUS_MASK;
  test_info.roll_rate_stat = signal_failure;
  test_info.pitch_rate_stat = imu_not_available;
  test_info.ax_stat = init_in_progress;
  test_info.ay_stat = off_highway_mm7p10::STATUS_MASK;
  test_info.az_stat = signal_failure | imu_not_available;
  test_info.temp_rate_z = off_highway_mm7p10::TEMP_INVALID_LOW;
  test_info.hw_index = 15;
  publish_imu_and_info(test_imu, test_info);
  verify_imu_and_info(test_imu, get_imu(), test_info, get_info());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
