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
#include <map>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <algorithm>

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
  pub_euler_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
    config_.topic_prefix + "euler", 10);
  pub_temp_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
    config_.topic_prefix + "temp", 10);
  pub_calib_status_ = node_->create_publisher<std_msgs::msg::String>(
    config_.topic_prefix + "calib_status", 10);

  // Create calibration service
  calibration_service_ = node_->create_service<example_interfaces::srv::Trigger>(
    config_.topic_prefix + "calibration_request",
    std::bind(&SensorService::calibration_request_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Create imu_ok publisher with latching QoS (transient_local)
  rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
  latching_qos.transient_local();
  pub_imu_ok_ = node_->create_publisher<std_msgs::msg::Bool>(
    config_.topic_prefix + "imu_ok", latching_qos);
}

bool SensorService::configure()
{
  RCLCPP_INFO(node_->get_logger(), "Configuring device...");

  // -- Adafruit pattern: set CONFIG mode first, THEN reset --
  // BNO055 datasheet requires CONFIG mode to reliably accept SYS_TRIGGER
  // reset. Writing reset while in a fusion mode (NDOF etc.) can be ignored.
  RCLCPP_INFO(node_->get_logger(), "Switching to config mode before reset...");
  {
    std::vector<uint8_t> config_mode = {OPERATION_MODE_CONFIG};
    // Best-effort: if this fails (e.g. sensor confused), the reset below
    // will still attempt to bring it to a known state.
    write_register(BNO055_OPR_MODE_ADDR, config_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  // Reset the sensor for a clean state
  RCLCPP_INFO(node_->get_logger(), "Resetting sensor...");
  std::vector<uint8_t> page_zero = {0x00};
  write_register(BNO055_PAGE_ID_ADDR, page_zero);
  std::vector<uint8_t> reset_trigger = {0x20};
  write_register(BNO055_SYS_TRIGGER_ADDR, reset_trigger);

  // -- Adafruit pattern: poll chip ID instead of fixed sleep --
  // Adafruit does: delay(30); while(readChipId != 0xA0) { delay(10); } delay(50);
  // The BNO055 can take 400-850ms to boot, but polling exits as soon as ready.
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  // Flush stale data from UART buffers after sensor reset
  if (connector_) {
    connector_->flush_buffers();
  }

  // Poll chip ID with 100ms intervals, up to 2 seconds total
  std::vector<uint8_t> chip_id;
  bool chip_id_ok = false;
  for (int attempt = 0; attempt < 20; attempt++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (connector_) {
      connector_->flush_buffers();
    }
    chip_id.clear();
    if (read_register(BNO055_CHIP_ID_ADDR, chip_id, 1) &&
        !chip_id.empty() && chip_id[0] == BNO055_ID)
    {
      chip_id_ok = true;
      break;
    }
    if (attempt > 0 && attempt % 5 == 0) {
      RCLCPP_WARN(node_->get_logger(),
        "Waiting for BNO055 to boot after reset... (%d/20)", attempt);
    }
  }
  // Extra 50ms settling time after chip ID read (per Adafruit)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!chip_id_ok) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read correct chip ID after reset");
    return false;
  }

  // Set to config mode (with retries - USB bus contention can cause transient failures)
  bool config_mode_ok = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (attempt > 0) {
      RCLCPP_WARN(node_->get_logger(), "Config mode retry %d/5...", attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (set_mode(OPERATION_MODE_CONFIG)) {
      config_mode_ok = true;
      break;
    }
  }
  if (!config_mode_ok) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU into config mode");
    return false;
  }

  // Set normal power mode
  std::vector<uint8_t> power_mode = {POWER_MODE_NORMAL};
  if (!write_register(BNO055_PWR_MODE_ADDR, power_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU normal power mode");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Set register page 0
  std::vector<uint8_t> page = {0x00};
  if (!write_register(BNO055_PAGE_ID_ADDR, page)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU register page 0");
  }

  // System trigger - clear reset
  std::vector<uint8_t> trigger = {0x00};
  if (!write_register(BNO055_SYS_TRIGGER_ADDR, trigger)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to start IMU");
  }
  // Adafruit: delay(10) after clearing SYS_TRIGGER
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Verify self-test result
  std::vector<uint8_t> self_test;
  if (read_register(BNO055_SELFTEST_RESULT_ADDR, self_test, 1)) {
    if ((self_test[0] & SELFTEST_ALL_PASSED) != SELFTEST_ALL_PASSED) {
      RCLCPP_WARN(node_->get_logger(),
        "Self-test incomplete: 0x%02X (expected 0x%02X) - "
        "ACC:%s MAG:%s GYR:%s MCU:%s",
        self_test[0], SELFTEST_ALL_PASSED,
        (self_test[0] & 0x01) ? "OK" : "FAIL",
        (self_test[0] & 0x02) ? "OK" : "FAIL",
        (self_test[0] & 0x04) ? "OK" : "FAIL",
        (self_test[0] & 0x08) ? "OK" : "FAIL");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Self-test passed (all sensors OK)");
    }
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
    // Adafruit: delay(10) after axis remap write
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Set calibration offsets if configured
  if (config_.set_offsets) {
    RCLCPP_INFO(node_->get_logger(), "Writing calibration offsets to sensor registers");

    if (config_.offset_acc.size() >= 3) {
      write_offset(ACCEL_OFFSET_X_LSB_ADDR, config_.offset_acc[0]);
      write_offset(ACCEL_OFFSET_Y_LSB_ADDR, config_.offset_acc[1]);
      write_offset(ACCEL_OFFSET_Z_LSB_ADDR, config_.offset_acc[2]);
      RCLCPP_INFO(node_->get_logger(), "  Accel offsets: [%d, %d, %d]",
        config_.offset_acc[0], config_.offset_acc[1], config_.offset_acc[2]);
    }

    if (config_.offset_mag.size() >= 3) {
      write_offset(MAG_OFFSET_X_LSB_ADDR, config_.offset_mag[0]);
      write_offset(MAG_OFFSET_Y_LSB_ADDR, config_.offset_mag[1]);
      write_offset(MAG_OFFSET_Z_LSB_ADDR, config_.offset_mag[2]);
      RCLCPP_INFO(node_->get_logger(), "  Mag offsets: [%d, %d, %d]",
        config_.offset_mag[0], config_.offset_mag[1], config_.offset_mag[2]);
    }

    if (config_.offset_gyr.size() >= 3) {
      write_offset(GYRO_OFFSET_X_LSB_ADDR, config_.offset_gyr[0]);
      write_offset(GYRO_OFFSET_Y_LSB_ADDR, config_.offset_gyr[1]);
      write_offset(GYRO_OFFSET_Z_LSB_ADDR, config_.offset_gyr[2]);
      RCLCPP_INFO(node_->get_logger(), "  Gyro offsets: [%d, %d, %d]",
        config_.offset_gyr[0], config_.offset_gyr[1], config_.offset_gyr[2]);
    }

    // Write accelerometer and magnetometer radius
    write_offset(ACCEL_RADIUS_LSB_ADDR, config_.radius_acc);
    write_offset(MAG_RADIUS_LSB_ADDR, config_.radius_mag);
    RCLCPP_INFO(node_->get_logger(), "  Accel radius: %d, Mag radius: %d",
      config_.radius_acc, config_.radius_mag);
  }

  // Set operation mode
  if (!set_mode(config_.operation_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode");
    return false;
  }
  // Adafruit: delay(20) after final setMode
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  RCLCPP_INFO(node_->get_logger(), "Bosch BNO055 IMU configuration complete");
  return true;
}

void SensorService::activate_publishers()
{
  pub_imu_->on_activate();
  pub_imu_raw_->on_activate();
  pub_mag_->on_activate();
  pub_grav_->on_activate();
  pub_euler_->on_activate();
  pub_temp_->on_activate();
  pub_calib_status_->on_activate();
  pub_imu_ok_->on_activate();

  // Publish initial imu_ok state so subscribers receive a value immediately
  auto msg = std_msgs::msg::Bool();
  msg.data = imu_ok_;
  pub_imu_ok_->publish(msg);
}

void SensorService::deactivate_publishers()
{
  pub_imu_->on_deactivate();
  pub_imu_raw_->on_deactivate();
  pub_mag_->on_deactivate();
  pub_grav_->on_deactivate();
  pub_euler_->on_deactivate();
  pub_temp_->on_deactivate();
  pub_calib_status_->on_deactivate();
  pub_imu_ok_->on_deactivate();
}

void SensorService::get_sensor_data()
{
  // Read 46 bytes starting from accelerometer data register
  // 45 bytes = sensor data (accel through temp), +1 byte = CALIB_STAT (0x35)
  // Reading calibration inline avoids a separate UART round-trip per cycle
  // (matches h4r_bosch_bno055_uart's 46-byte bulk read)
  std::vector<uint8_t> buf;
  if (!read_register(BNO055_ACCEL_DATA_X_LSB_ADDR, buf, 46)) {
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

  // Publish calibration status inline from buf[45] (CALIB_STAT register)
  // This avoids a separate UART read transaction for calibration
  {
    uint8_t calib_byte = buf[45];
    uint8_t sys = (calib_byte >> 6) & 0x03;
    uint8_t gyro = (calib_byte >> 4) & 0x03;
    uint8_t accel = (calib_byte >> 2) & 0x03;
    uint8_t mag = calib_byte & 0x03;

    bool fully_calibrated = is_fully_calibrated(sys, gyro, accel, mag);

    std::stringstream ss;
    ss << "{\"sys\": " << static_cast<int>(sys)
       << ", \"gyro\": " << static_cast<int>(gyro)
       << ", \"accel\": " << static_cast<int>(accel)
       << ", \"mag\": " << static_cast<int>(mag)
       << ", \"fully_calibrated\": " << (fully_calibrated ? "true" : "false") << "}";

    auto calib_msg = std_msgs::msg::String();
    calib_msg.data = ss.str();
    pub_calib_status_->publish(calib_msg);

    RCLCPP_DEBUG(node_->get_logger(), "Calibration (inline): %s", ss.str().c_str());
  }

  // Publish Euler angles (indices 18-23: heading, roll, pitch)
  auto euler_msg = geometry_msgs::msg::Vector3();
  euler_msg.x = bytes_to_int16(buf, 18) / 16.0;  // heading (degrees)
  euler_msg.y = bytes_to_int16(buf, 20) / 16.0;  // roll (degrees)
  euler_msg.z = bytes_to_int16(buf, 22) / 16.0;  // pitch (degrees)
  pub_euler_->publish(euler_msg);

  // IMU anomaly detection (matching Python implementation)
  check_imu_anomalies(imu_msg);
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

  bool fully_calibrated = is_fully_calibrated(sys, gyro, accel, mag);

  std::stringstream ss;
  ss << "{\"sys\": " << static_cast<int>(sys)
     << ", \"gyro\": " << static_cast<int>(gyro)
     << ", \"accel\": " << static_cast<int>(accel)
     << ", \"mag\": " << static_cast<int>(mag)
     << ", \"fully_calibrated\": " << (fully_calibrated ? "true" : "false") << "}";

  auto calib_msg = std_msgs::msg::String();
  calib_msg.data = ss.str();
  pub_calib_status_->publish(calib_msg);

  // Log at DEBUG level to avoid spam at 10Hz, or use throttle
  RCLCPP_DEBUG(node_->get_logger(), "Calibration: %s", ss.str().c_str());
}

void SensorService::check_watchdog()
{
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_successful_read_).count() / 1000.0;

  // Check imu_ok timeout - set imu_ok to false if no data for too long
  if (config_.imu_ok_timeout > 0.0 && elapsed >= config_.imu_ok_timeout) {
    if (imu_ok_) {
      set_imu_ok(false);
      std::stringstream ss;
      ss << "[Watchdog] IMU data timeout: No data received for " << std::fixed
         << std::setprecision(2) << elapsed << " seconds (threshold: "
         << config_.imu_ok_timeout << "s). Setting imu_ok to false.";
      log_throttled("watchdog_imu_ok_timeout", ss.str(), 1, 5.0);
    }
  }

  // Check serial reset timeout - reset connection and reconfigure sensor
  // Persistent retry: keeps trying until data flows again or ROS shuts down.
  // connector_->reset() itself also retries forever internally.
  if (config_.serial_reset_timeout > 0.0 && elapsed >= config_.serial_reset_timeout
      && !reset_in_progress_)
  {
    reset_in_progress_ = true;
    set_imu_ok(false);

    RCLCPP_WARN(node_->get_logger(),
      "[Watchdog] Serial timeout: No data for %.2fs (threshold: %.2fs). "
      "Starting persistent reconnection...", elapsed, config_.serial_reset_timeout);

    bool recovered = false;
    for (int attempt = 1; rclcpp::ok() && !recovered; attempt++) {
      try {
        if (attempt > 1) {
          // 1s delay between attempts (connector_->reset() already has internal delays)
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        RCLCPP_INFO(node_->get_logger(),
          "[Watchdog] Reconnection attempt %d...", attempt);

        // Reset the connector (close + reopen + BNO055 hardware reset)
        if (connector_ && connector_->reset()) {
          // Reconfigure the sensor after reset
          if (configure()) {
            last_successful_read_ = std::chrono::steady_clock::now();
            consecutive_error_count_ = 0;
            // Clear anomaly detection state after recovery
            imu_constant_count_ = 0;
            has_prev_imu_data_ = false;
            accel_x_history_.clear();
            accel_y_history_.clear();
            accel_z_history_.clear();
            RCLCPP_INFO(node_->get_logger(),
              "[Watchdog] Sensor recovered on attempt %d", attempt);
            recovered = true;
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "[Watchdog] Reset OK but reconfigure failed (attempt %d)", attempt);
          }
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "[Watchdog] Serial reset failed (attempt %d)", attempt);
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "[Watchdog] Exception on attempt %d: %s", attempt, e.what());
      }
    }

    if (!recovered) {
      // Only reached when rclcpp::ok() becomes false (ROS shutdown)
      RCLCPP_ERROR(node_->get_logger(),
        "[Watchdog] Reconnection aborted - ROS shutdown requested");
    }

    reset_in_progress_ = false;
  }

  // Check for connection issues based on system status and self test
  // Only check if we're not in a timeout state (no point reading registers if connection is down)
  if (elapsed < config_.serial_reset_timeout) {
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
      if (self_test[0] != SELFTEST_ALL_PASSED) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
          "Self-test failed: 0x%02X (expected 0x%02X)", self_test[0], SELFTEST_ALL_PASSED);
      }
    }
  }

  // Check for timeout
  if (elapsed > 2.0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No data received for %.2f seconds - possible connection loss", elapsed);
  }
}

void SensorService::check_imu_anomalies(const sensor_msgs::msg::Imu & imu_msg)
{
  if (has_prev_imu_data_) {
    // Check if IMU data is constant (sensor stuck)
    bool is_constant = (
      std::abs(imu_msg.linear_acceleration.x - prev_accel_x_) < config_.imu_change_epsilon &&
      std::abs(imu_msg.linear_acceleration.y - prev_accel_y_) < config_.imu_change_epsilon &&
      std::abs(imu_msg.linear_acceleration.z - prev_accel_z_) < config_.imu_change_epsilon &&
      std::abs(imu_msg.angular_velocity.x - prev_gyro_x_) < config_.imu_change_epsilon &&
      std::abs(imu_msg.angular_velocity.y - prev_gyro_y_) < config_.imu_change_epsilon &&
      std::abs(imu_msg.angular_velocity.z - prev_gyro_z_) < config_.imu_change_epsilon);

    if (is_constant) {
      imu_constant_count_++;
      if (imu_constant_count_ >= config_.imu_constant_threshold) {
        double duration_sec = imu_constant_count_ / config_.data_query_frequency;
        std::stringstream ss;
        ss << "IMU anomaly detected: Data constant for " << std::fixed << std::setprecision(2)
           << duration_sec << " seconds"
           << " (accel: " << imu_msg.linear_acceleration.x
           << ", " << imu_msg.linear_acceleration.y
           << ", " << imu_msg.linear_acceleration.z
           << " | gyro: " << imu_msg.angular_velocity.x
           << ", " << imu_msg.angular_velocity.y
           << ", " << imu_msg.angular_velocity.z << ")";
        log_throttled("imu_constant", ss.str(), 1, 5.0);
        set_imu_ok(false);
      }
    } else {
      imu_constant_count_ = 0;
      set_imu_ok(true);
    }
  }

  // Check for abnormal acceleration values using standard deviation
  accel_x_history_.push_back(imu_msg.linear_acceleration.x);
  accel_y_history_.push_back(imu_msg.linear_acceleration.y);
  accel_z_history_.push_back(imu_msg.linear_acceleration.z);

  if (accel_x_history_.size() > config_.imu_history_size) {
    accel_x_history_.erase(accel_x_history_.begin());
    accel_y_history_.erase(accel_y_history_.begin());
    accel_z_history_.erase(accel_z_history_.begin());
  }

  if (accel_x_history_.size() >= config_.imu_history_size / 2) {
    size_t n = accel_x_history_.size();
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    for (size_t i = 0; i < n; ++i) {
      mean_x += accel_x_history_[i];
      mean_y += accel_y_history_[i];
      mean_z += accel_z_history_[i];
    }
    mean_x /= n;
    mean_y /= n;
    mean_z /= n;

    double var_x = 0.0, var_y = 0.0, var_z = 0.0;
    for (size_t i = 0; i < n; ++i) {
      var_x += std::pow(accel_x_history_[i] - mean_x, 2);
      var_y += std::pow(accel_y_history_[i] - mean_y, 2);
      var_z += std::pow(accel_z_history_[i] - mean_z, 2);
    }
    var_x /= n;
    var_y /= n;
    var_z /= n;

    double std_x = std::sqrt(var_x);
    double std_y = std::sqrt(var_y);
    double std_z = std::sqrt(var_z);

    if (std_x > config_.max_std_dev_threshold ||
        std_y > config_.max_std_dev_threshold ||
        std_z > config_.max_std_dev_threshold)
    {
      std::stringstream ss;
      ss << "IMU anomaly detected: Standard deviation too high (noisy/broken data). "
         << "Std Dev: (" << std::fixed << std::setprecision(4)
         << std_x << ", " << std_y << ", " << std_z
         << ") | Threshold: " << std::setprecision(2) << config_.max_std_dev_threshold
         << " m/s² | Mean: (" << std::setprecision(3)
         << mean_x << ", " << mean_y << ", " << mean_z << ")";
      log_throttled("imu_std_dev", ss.str(), 1, 5.0);
      set_imu_ok(false);
    }
  }

  // Update previous values
  prev_accel_x_ = imu_msg.linear_acceleration.x;
  prev_accel_y_ = imu_msg.linear_acceleration.y;
  prev_accel_z_ = imu_msg.linear_acceleration.z;
  prev_gyro_x_ = imu_msg.angular_velocity.x;
  prev_gyro_y_ = imu_msg.angular_velocity.y;
  prev_gyro_z_ = imu_msg.angular_velocity.z;
  has_prev_imu_data_ = true;
}

void SensorService::set_imu_ok(bool value)
{
  if (imu_ok_ != value) {
    imu_ok_ = value;
    auto msg = std_msgs::msg::Bool();
    msg.data = value;
    pub_imu_ok_->publish(msg);
    prev_imu_ok_ = value;

    if (value) {
      RCLCPP_INFO(node_->get_logger(), "IMU status recovered: imu_ok = true");
    } else {
      RCLCPP_WARN(node_->get_logger(), "IMU status degraded: imu_ok = false");
    }
  }
}

void SensorService::log_throttled(
  const std::string & key, const std::string & msg, int level, double timeout_sec)
{
  auto now = std::chrono::steady_clock::now();
  auto it = log_throttle_map_.find(key);

  if (it == log_throttle_map_.end() ||
      std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count() / 1000.0 >= timeout_sec)
  {
    switch (level) {
      case 0:  // INFO
        RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
        break;
      case 1:  // WARN
        RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
        break;
      case 2:  // ERROR
        RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
        break;
      default:
        RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
        break;
    }
    log_throttle_map_[key] = now;
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
  bool result = write_register(BNO055_OPR_MODE_ADDR, data);
  // BNO055 requires a delay after mode transition (per datasheet: 19ms typical)
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  return result;
}

void SensorService::write_offset(uint8_t reg_lsb, int16_t value)
{
  std::vector<uint8_t> data = {
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF)
  };
  if (!write_register(reg_lsb, data)) {
    RCLCPP_WARN(node_->get_logger(),
      "Failed to write offset at register 0x%02X (value: %d)", reg_lsb, value);
  }
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

bool SensorService::is_fully_calibrated(
  uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) const
{
  switch (config_.operation_mode) {
    case OPERATION_MODE_ACCONLY:
      return (accel == CALIBRATION_FULLY_CALIBRATED);
    case OPERATION_MODE_MAGONLY:
      return (mag == CALIBRATION_FULLY_CALIBRATED);
    case OPERATION_MODE_GYRONLY:
    case OPERATION_MODE_M4G:
      return (gyro == CALIBRATION_FULLY_CALIBRATED);
    case OPERATION_MODE_ACCMAG:
    case OPERATION_MODE_COMPASS:
      return (accel == CALIBRATION_FULLY_CALIBRATED && mag == CALIBRATION_FULLY_CALIBRATED);
    case OPERATION_MODE_ACCGYRO:
    case OPERATION_MODE_IMUPLUS:
      return (accel == CALIBRATION_FULLY_CALIBRATED && gyro == CALIBRATION_FULLY_CALIBRATED);
    case OPERATION_MODE_MAGGYRO:
      return (mag == CALIBRATION_FULLY_CALIBRATED && gyro == CALIBRATION_FULLY_CALIBRATED);
    default:
      return (sys == CALIBRATION_FULLY_CALIBRATED && gyro == CALIBRATION_FULLY_CALIBRATED &&
              accel == CALIBRATION_FULLY_CALIBRATED && mag == CALIBRATION_FULLY_CALIBRATED);
  }
}

int16_t SensorService::bytes_to_int16(const std::vector<uint8_t> & data, size_t offset)
{
  if (offset + 1 >= data.size()) {
    return 0;
  }
  return static_cast<int16_t>((data[offset + 1] << 8) | data[offset]);
}

}  // namespace bno055
