// Copyright 2021 AUTHORS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the AUTHORS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "bno055/sensor_service.hpp"
#include "bno055/registers.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace bno055
{

SensorService::SensorService(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<Connector> connector,
  const SensorConfig & config)
: node_(node),
  connector_(connector),
  config_(config),
  consecutive_error_count_(0)
{
  last_successful_read_ = std::chrono::steady_clock::now();

  // Create publishers
  pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    config_.topic_prefix + "imu", 10);
  pub_imu_raw_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    config_.topic_prefix + "imu_raw", 10);
  pub_mag_ = node_->create_publisher<sensor_msgs::msg::MagneticField>(
    config_.topic_prefix + "mag", 10);
  pub_grav_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
    config_.topic_prefix + "grav", 10);
  pub_temp_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
    config_.topic_prefix + "temp", 10);
  pub_calib_status_ = node_->create_publisher<std_msgs::msg::String>(
    config_.topic_prefix + "calib_status", 10);

  // Create calibration service
  calibration_service_ = node_->create_service<example_interfaces::srv::Trigger>(
    config_.topic_prefix + "calibration_request",
    std::bind(&SensorService::calibration_request_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

bool SensorService::configure()
{
  RCLCPP_INFO(node_->get_logger(), "Configuring device...");

  // Verify chip ID
  std::vector<uint8_t> chip_id;
  if (!read_register(BNO055_CHIP_ID_ADDR, chip_id, 1)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read chip ID");
    return false;
  }

  if (chip_id[0] != BNO055_ID) {
    RCLCPP_ERROR(node_->get_logger(), "Device ID=%02x is incorrect", chip_id[0]);
    return false;
  }

  // Set to config mode
  if (!set_mode(OPERATION_MODE_CONFIG)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU into config mode");
    return false;
  }

  // Set normal power mode
  std::vector<uint8_t> power_mode = {POWER_MODE_NORMAL};
  if (!write_register(BNO055_PWR_MODE_ADDR, power_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU normal power mode");
  }

  // Set register page 0
  std::vector<uint8_t> page = {0x00};
  if (!write_register(BNO055_PAGE_ID_ADDR, page)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU register page 0");
  }

  // System trigger
  std::vector<uint8_t> trigger = {0x00};
  if (!write_register(BNO055_SYS_TRIGGER_ADDR, trigger)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to start IMU");
  }

  // Set units: Android orientation mode, degrees, Celsius
  std::vector<uint8_t> units = {0x83};
  if (!write_register(BNO055_UNIT_SEL_ADDR, units)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU units");
  }

  // Axis remapping based on placement
  std::map<std::string, std::vector<uint8_t>> mount_positions = {
    {"P0", {0x21, 0x04}},
    {"P1", {0x24, 0x00}},
    {"P2", {0x24, 0x06}},
    {"P3", {0x21, 0x02}},
    {"P4", {0x24, 0x03}},
    {"P5", {0x21, 0x02}},
    {"P6", {0x21, 0x07}},
    {"P7", {0x24, 0x05}}
  };

  if (mount_positions.find(config_.placement_axis_remap) != mount_positions.end()) {
    if (!write_register(BNO055_AXIS_MAP_CONFIG_ADDR, mount_positions[config_.placement_axis_remap])) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set sensor placement configuration");
    }
  }

  // Set calibration offsets if configured
  if (config_.set_offsets) {
    // Switch to config mode for setting offsets
    set_mode(OPERATION_MODE_CONFIG);

    // Write offsets (simplified - in production you'd read current values first)
    RCLCPP_INFO(node_->get_logger(), "Setting calibration offsets");
    
    // Set operation mode
    if (!set_mode(config_.operation_mode)) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode");
      return false;
    }
  } else {
    // Set operation mode
    if (!set_mode(config_.operation_mode)) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode");
      return false;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Bosch BNO055 IMU configuration complete");
  return true;
}

void SensorService::activate_publishers()
{
  pub_imu_->on_activate();
  pub_imu_raw_->on_activate();
  pub_mag_->on_activate();
  pub_grav_->on_activate();
  pub_temp_->on_activate();
  pub_calib_status_->on_activate();
}

void SensorService::deactivate_publishers()
{
  pub_imu_->on_deactivate();
  pub_imu_raw_->on_deactivate();
  pub_mag_->on_deactivate();
  pub_grav_->on_deactivate();
  pub_temp_->on_deactivate();
  pub_calib_status_->on_deactivate();
}

void SensorService::get_sensor_data()
{
  // Read 45 bytes starting from accelerometer data register
  std::vector<uint8_t> buf;
  if (!read_register(BNO055_ACCEL_DATA_X_LSB_ADDR, buf, 45)) {
    consecutive_error_count_++;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Failed to read sensor data (error count: %d)", consecutive_error_count_);
    return;
  }

  // Update successful read time
  last_successful_read_ = std::chrono::steady_clock::now();
  consecutive_error_count_ = 0;

  auto now = node_->get_clock()->now();

  // Publish raw IMU data
  auto imu_raw_msg = sensor_msgs::msg::Imu();
  imu_raw_msg.header.stamp = now;
  imu_raw_msg.header.frame_id = config_.frame_id;

  // Raw accelerometer data (indices 0-5)
  imu_raw_msg.linear_acceleration.x = bytes_to_int16(buf, 0) / config_.acc_factor;
  imu_raw_msg.linear_acceleration.y = bytes_to_int16(buf, 2) / config_.acc_factor;
  imu_raw_msg.linear_acceleration.z = bytes_to_int16(buf, 4) / config_.acc_factor;
  imu_raw_msg.linear_acceleration_covariance = {
    config_.variance_acc[0], 0.0, 0.0,
    0.0, config_.variance_acc[1], 0.0,
    0.0, 0.0, config_.variance_acc[2]
  };

  // Gyroscope data (indices 12-17)
  imu_raw_msg.angular_velocity.x = bytes_to_int16(buf, 12) / config_.gyr_factor;
  imu_raw_msg.angular_velocity.y = bytes_to_int16(buf, 14) / config_.gyr_factor;
  imu_raw_msg.angular_velocity.z = bytes_to_int16(buf, 16) / config_.gyr_factor;
  imu_raw_msg.angular_velocity_covariance = {
    config_.variance_angular_vel[0], 0.0, 0.0,
    0.0, config_.variance_angular_vel[1], 0.0,
    0.0, 0.0, config_.variance_angular_vel[2]
  };

  imu_raw_msg.orientation_covariance = {
    config_.variance_orientation[0], 0.0, 0.0,
    0.0, config_.variance_orientation[1], 0.0,
    0.0, 0.0, config_.variance_orientation[2]
  };

  pub_imu_raw_->publish(imu_raw_msg);

  // Publish filtered IMU data with quaternion
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = now;
  imu_msg.header.frame_id = config_.frame_id;

  // Quaternion data (indices 24-31)
  double qw = bytes_to_int16(buf, 24) / 16384.0;  // Scale factor for quaternion
  double qx = bytes_to_int16(buf, 26) / 16384.0;
  double qy = bytes_to_int16(buf, 28) / 16384.0;
  double qz = bytes_to_int16(buf, 30) / 16384.0;

  // Normalize quaternion
  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 0.0) {
    imu_msg.orientation.x = qx / norm;
    imu_msg.orientation.y = qy / norm;
    imu_msg.orientation.z = qz / norm;
    imu_msg.orientation.w = qw / norm;
  }

  // Linear acceleration (indices 32-37)
  imu_msg.linear_acceleration.x = bytes_to_int16(buf, 32) / config_.acc_factor;
  imu_msg.linear_acceleration.y = bytes_to_int16(buf, 34) / config_.acc_factor;
  imu_msg.linear_acceleration.z = bytes_to_int16(buf, 36) / config_.acc_factor;

  imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance;
  imu_msg.angular_velocity = imu_raw_msg.angular_velocity;
  imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance;
  imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance;

  pub_imu_->publish(imu_msg);

  // Publish magnetometer data (indices 6-11)
  auto mag_msg = sensor_msgs::msg::MagneticField();
  mag_msg.header.stamp = now;
  mag_msg.header.frame_id = config_.frame_id;
  mag_msg.magnetic_field.x = bytes_to_int16(buf, 6) / config_.mag_factor;
  mag_msg.magnetic_field.y = bytes_to_int16(buf, 8) / config_.mag_factor;
  mag_msg.magnetic_field.z = bytes_to_int16(buf, 10) / config_.mag_factor;
  mag_msg.magnetic_field_covariance = {
    config_.variance_mag[0], 0.0, 0.0,
    0.0, config_.variance_mag[1], 0.0,
    0.0, 0.0, config_.variance_mag[2]
  };
  pub_mag_->publish(mag_msg);

  // Publish gravity vector (indices 38-43)
  auto grav_msg = geometry_msgs::msg::Vector3();
  grav_msg.x = bytes_to_int16(buf, 38) / config_.grav_factor;
  grav_msg.y = bytes_to_int16(buf, 40) / config_.grav_factor;
  grav_msg.z = bytes_to_int16(buf, 42) / config_.grav_factor;
  pub_grav_->publish(grav_msg);

  // Publish temperature (index 44)
  auto temp_msg = sensor_msgs::msg::Temperature();
  temp_msg.header.stamp = now;
  temp_msg.header.frame_id = config_.frame_id;
  temp_msg.temperature = static_cast<double>(buf[44]);
  pub_temp_->publish(temp_msg);
}

void SensorService::get_calib_status()
{
  std::vector<uint8_t> calib_data;
  if (!read_register(BNO055_CALIB_STAT_ADDR, calib_data, 1)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to read calibration status");
    return;
  }

  uint8_t sys = (calib_data[0] >> 6) & 0x03;
  uint8_t gyro = (calib_data[0] >> 4) & 0x03;
  uint8_t accel = (calib_data[0] >> 2) & 0x03;
  uint8_t mag = calib_data[0] & 0x03;

  std::stringstream ss;
  ss << "{\"sys\": " << static_cast<int>(sys)
     << ", \"gyro\": " << static_cast<int>(gyro)
     << ", \"accel\": " << static_cast<int>(accel)
     << ", \"mag\": " << static_cast<int>(mag) << "}";

  auto calib_msg = std_msgs::msg::String();
  calib_msg.data = ss.str();
  pub_calib_status_->publish(calib_msg);

  RCLCPP_INFO(node_->get_logger(), "Calibration: %s", ss.str().c_str());
}

void SensorService::check_watchdog()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_successful_read_).count() / 1000.0;

  // Check for connection issues based on system status and self test
  std::vector<uint8_t> sys_status, sys_err, self_test;
  
  if (read_register(BNO055_SYS_STAT_ADDR, sys_status, 1) &&
      read_register(BNO055_SYS_ERR_ADDR, sys_err, 1) &&
      read_register(BNO055_SELFTEST_RESULT_ADDR, self_test, 1))
  {
    // System status: 0=idle, 1=sys error, 2=init peripheral, 3=init system,
    //                4=executing, 5=running, 6=running without fusion
    if (sys_status[0] == 1 || sys_err[0] != 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "System error detected - Status: %d, Error: %d", sys_status[0], sys_err[0]);
    }
    
    // Self test result: bit 0=accelerometer, 1=magnetometer, 2=gyroscope, 3=MCU
    if (self_test[0] != 0x0F) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Self-test failed: 0x%02X (expected 0x0F)", self_test[0]);
    }
  }

  // Check for timeout
  if (elapsed > 2.0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No data received for %.2f seconds - possible connection loss", elapsed);
  }
}

void SensorService::calibration_request_callback(
  const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
  std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
{
  (void)request;  // Unused

  // Read calibration status
  std::vector<uint8_t> calib_data;
  if (read_register(BNO055_CALIB_STAT_ADDR, calib_data, 1)) {
    uint8_t sys = (calib_data[0] >> 6) & 0x03;
    uint8_t gyro = (calib_data[0] >> 4) & 0x03;
    uint8_t accel = (calib_data[0] >> 2) & 0x03;
    uint8_t mag = calib_data[0] & 0x03;

    std::stringstream ss;
    ss << "Calibration status - System: " << static_cast<int>(sys)
       << ", Gyro: " << static_cast<int>(gyro)
       << ", Accel: " << static_cast<int>(accel)
       << ", Mag: " << static_cast<int>(mag);

    response->success = true;
    response->message = ss.str();
  } else {
    response->success = false;
    response->message = "Failed to read calibration status";
  }
}

bool SensorService::set_mode(uint8_t mode)
{
  std::vector<uint8_t> data = {mode};
  return write_register(BNO055_OPR_MODE_ADDR, data);
}

bool SensorService::read_register(uint8_t reg, std::vector<uint8_t> & data, size_t length)
{
  if (!connector_ || !connector_->is_connected()) {
    return false;
  }
  return connector_->read(reg, data, length);
}

bool SensorService::write_register(uint8_t reg, const std::vector<uint8_t> & data)
{
  if (!connector_ || !connector_->is_connected()) {
    return false;
  }
  return connector_->write(reg, data);
}

int16_t SensorService::bytes_to_int16(const std::vector<uint8_t> & data, size_t offset)
{
  if (offset + 1 >= data.size()) {
    return 0;
  }
  return static_cast<int16_t>((data[offset + 1] << 8) | data[offset]);
}

}  // namespace bno055
