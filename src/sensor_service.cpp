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

  // ── Match Python SensorService.configure() exactly ──
  // Python flow: chip_id → CONFIG mode → power → page → trigger → units → remap
  //              → print_calib_data → set_calib_offsets → operation mode
  // No retries, no delays between writes, no self-test check.

  // Step 1: Verify chip ID (single attempt, like Python)
  std::vector<uint8_t> chip_id;
  if (!read_register(BNO055_CHIP_ID_ADDR, chip_id, 1) ||
      chip_id.empty() || chip_id[0] != BNO055_ID)
  {
    RCLCPP_ERROR(node_->get_logger(), "Communication error: Device ID=%s is incorrect",
      chip_id.empty() ? "(no response)" : std::to_string(chip_id[0]).c_str());
    return false;
  }

  // Step 2: Set CONFIG mode (direct write, no retry, no delay — like Python)
  std::vector<uint8_t> config_mode = {OPERATION_MODE_CONFIG};
  if (!write_register(BNO055_OPR_MODE_ADDR, config_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU into config mode.");
  }

  // Step 3: Set normal power mode
  std::vector<uint8_t> power_mode = {POWER_MODE_NORMAL};
  if (!write_register(BNO055_PWR_MODE_ADDR, power_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU normal power mode.");
  }

  // Step 4: Set register page 0
  std::vector<uint8_t> page = {0x00};
  if (!write_register(BNO055_PAGE_ID_ADDR, page)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU register page 0.");
  }

  // Step 5: SYS_TRIGGER = 0x00 (start, NO reset)
  std::vector<uint8_t> trigger = {0x00};
  if (!write_register(BNO055_SYS_TRIGGER_ADDR, trigger)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to start IMU.");
  }

  // Step 6: Set units
  std::vector<uint8_t> units = {0x83};
  if (!write_register(BNO055_UNIT_SEL_ADDR, units)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU units.");
  }

  // Step 7: Axis remapping (2 bytes)
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
      RCLCPP_WARN(node_->get_logger(), "Unable to set sensor placement configuration.");
    }
  }

  // Step 8: Show current sensor offsets (like Python print_calib_data)
  print_calib_data();

  // Step 9: Set calibration offsets if configured
  if (config_.set_offsets) {
    if (set_calib_offsets()) {
      RCLCPP_INFO(node_->get_logger(), "Successfully configured sensor offsets to:");
      print_calib_data();
    } else {
      RCLCPP_WARN(node_->get_logger(), "setting offsets failed");
    }
  }

  // Step 10: Set operation mode (direct write, no delay — like Python)
  RCLCPP_INFO(node_->get_logger(), "Setting device_mode to %d", config_.operation_mode);
  std::vector<uint8_t> op_mode = {config_.operation_mode};
  if (!write_register(BNO055_OPR_MODE_ADDR, op_mode)) {
    RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode into operation mode.");
  }

  RCLCPP_INFO(node_->get_logger(), "Bosch BNO055 IMU configuration complete.");
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

  // Quaternion data (indices 24-31) - raw int16 like Python
  double qw = static_cast<double>(bytes_to_int16(buf, 24));
  double qx = static_cast<double>(bytes_to_int16(buf, 26));
  double qy = static_cast<double>(bytes_to_int16(buf, 28));
  double qz = static_cast<double>(bytes_to_int16(buf, 30));

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
  // ── Match Python check_watchdog() exactly ──
  // Python: single reset attempt, no extra register reads
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_successful_read_).count() / 1000.0;

  // Check imu_ok timeout
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

  // Check serial reset timeout (single attempt like Python)
  if (config_.serial_reset_timeout > 0.0 && elapsed >= config_.serial_reset_timeout
      && !reset_in_progress_)
  {
    reset_in_progress_ = true;

    try {
      std::stringstream ss;
      ss << "[Watchdog] Serial timeout detected: No data received for "
         << std::fixed << std::setprecision(2) << elapsed << " seconds (threshold: "
         << config_.serial_reset_timeout << "s). Resetting serial connection...";
      log_throttled("watchdog_serial_reset", ss.str(), 2, 10.0);

      // Reset the connector (single attempt like Python)
      if (connector_ && connector_->reset()) {
        try {
          if (configure()) {
            last_successful_read_ = std::chrono::steady_clock::now();
            consecutive_error_count_ = 0;
            // Clear anomaly detection state after recovery
            imu_constant_count_ = 0;
            has_prev_imu_data_ = false;
            accel_x_history_.clear();
            accel_y_history_.clear();
            accel_z_history_.clear();
            log_throttled("watchdog_reset_success",
              "Sensor reconfigured after watchdog reset", 0, 10.0);
          } else {
            set_imu_ok(false);
            log_throttled("watchdog_reset_fail",
              "Failed to reconfigure sensor after watchdog reset", 2, 10.0);
          }
        } catch (const std::exception & e) {
          set_imu_ok(false);
          std::stringstream err;
          err << "Failed to reconfigure sensor after watchdog reset: " << e.what();
          log_throttled("watchdog_reset_fail", err.str(), 2, 10.0);
        }
      } else {
        set_imu_ok(false);
        log_throttled("watchdog_reset_fail",
          "Failed to reset serial connection", 2, 10.0);
      }
    } catch (...) {
      // ensure reset_in_progress_ is cleared
    }

    reset_in_progress_ = false;
  }
}

void SensorService::check_imu_anomalies(const sensor_msgs::msg::Imu & imu_msg)
{
  // ── Match Python _check_imu_anomalies() exactly ──
  // Python sets imu_ok as a plain variable, then publishes ONLY at the end
  // if the value changed. This avoids rapid true/false oscillation.

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
        imu_ok_ = false;
      }
    } else {
      imu_constant_count_ = 0;
      imu_ok_ = true;
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
      imu_ok_ = false;
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

  // Publish imu_ok status only when it changes (matching Python exactly)
  if (imu_ok_ != prev_imu_ok_) {
    auto msg = std_msgs::msg::Bool();
    msg.data = imu_ok_;
    pub_imu_ok_->publish(msg);
    prev_imu_ok_ = imu_ok_;
  }
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

void SensorService::print_calib_data()
{
  RCLCPP_INFO(node_->get_logger(), "Current sensor offsets:");

  std::vector<uint8_t> accel_offset, mag_offset, gyro_offset, accel_radius, mag_radius;

  if (read_register(ACCEL_OFFSET_X_LSB_ADDR, accel_offset, 6)) {
    uint16_t x = (accel_offset[1] << 8) | accel_offset[0];
    uint16_t y = (accel_offset[3] << 8) | accel_offset[2];
    uint16_t z = (accel_offset[5] << 8) | accel_offset[4];
    RCLCPP_INFO(node_->get_logger(), "\tAccel offsets (x y z): %d %d %d", x, y, z);
  }

  if (read_register(ACCEL_RADIUS_LSB_ADDR, accel_radius, 2)) {
    uint16_t r = (accel_radius[1] << 8) | accel_radius[0];
    RCLCPP_INFO(node_->get_logger(), "\tAccel radius: %d", r);
  }

  if (read_register(MAG_OFFSET_X_LSB_ADDR, mag_offset, 6)) {
    uint16_t x = (mag_offset[1] << 8) | mag_offset[0];
    uint16_t y = (mag_offset[3] << 8) | mag_offset[2];
    uint16_t z = (mag_offset[5] << 8) | mag_offset[4];
    RCLCPP_INFO(node_->get_logger(), "\tMag offsets (x y z): %d %d %d", x, y, z);
  }

  if (read_register(MAG_RADIUS_LSB_ADDR, mag_radius, 2)) {
    uint16_t r = (mag_radius[1] << 8) | mag_radius[0];
    RCLCPP_INFO(node_->get_logger(), "\tMag radius: %d", r);
  }

  if (read_register(GYRO_OFFSET_X_LSB_ADDR, gyro_offset, 6)) {
    uint16_t x = (gyro_offset[1] << 8) | gyro_offset[0];
    uint16_t y = (gyro_offset[3] << 8) | gyro_offset[2];
    uint16_t z = (gyro_offset[5] << 8) | gyro_offset[4];
    RCLCPP_INFO(node_->get_logger(), "\tGyro offsets (x y z): %d %d %d", x, y, z);
  }
}

bool SensorService::set_calib_offsets()
{
  // Python: Must switch to config mode + 25ms delay before writing offsets
  std::vector<uint8_t> config_mode = {OPERATION_MODE_CONFIG};
  if (!write_register(BNO055_OPR_MODE_ADDR, config_mode)) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to set IMU into config mode");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // Python: "Seems to only work when writing 1 register at a time"
  auto write_byte = [this](uint8_t reg, uint8_t value) -> bool {
    std::vector<uint8_t> data = {value};
    return write_register(reg, data);
  };

  try {
    if (config_.offset_acc.size() >= 3) {
      write_byte(ACCEL_OFFSET_X_LSB_ADDR, config_.offset_acc[0] & 0xFF);
      write_byte(ACCEL_OFFSET_X_MSB_ADDR, (config_.offset_acc[0] >> 8) & 0xFF);
      write_byte(ACCEL_OFFSET_Y_LSB_ADDR, config_.offset_acc[1] & 0xFF);
      write_byte(ACCEL_OFFSET_Y_MSB_ADDR, (config_.offset_acc[1] >> 8) & 0xFF);
      write_byte(ACCEL_OFFSET_Z_LSB_ADDR, config_.offset_acc[2] & 0xFF);
      write_byte(ACCEL_OFFSET_Z_MSB_ADDR, (config_.offset_acc[2] >> 8) & 0xFF);
    }

    write_byte(ACCEL_RADIUS_LSB_ADDR, config_.radius_acc & 0xFF);
    write_byte(ACCEL_RADIUS_MSB_ADDR, (config_.radius_acc >> 8) & 0xFF);

    if (config_.offset_mag.size() >= 3) {
      write_byte(MAG_OFFSET_X_LSB_ADDR, config_.offset_mag[0] & 0xFF);
      write_byte(MAG_OFFSET_X_MSB_ADDR, (config_.offset_mag[0] >> 8) & 0xFF);
      write_byte(MAG_OFFSET_Y_LSB_ADDR, config_.offset_mag[1] & 0xFF);
      write_byte(MAG_OFFSET_Y_MSB_ADDR, (config_.offset_mag[1] >> 8) & 0xFF);
      write_byte(MAG_OFFSET_Z_LSB_ADDR, config_.offset_mag[2] & 0xFF);
      write_byte(MAG_OFFSET_Z_MSB_ADDR, (config_.offset_mag[2] >> 8) & 0xFF);
    }

    write_byte(MAG_RADIUS_LSB_ADDR, config_.radius_mag & 0xFF);
    write_byte(MAG_RADIUS_MSB_ADDR, (config_.radius_mag >> 8) & 0xFF);

    if (config_.offset_gyr.size() >= 3) {
      write_byte(GYRO_OFFSET_X_LSB_ADDR, config_.offset_gyr[0] & 0xFF);
      write_byte(GYRO_OFFSET_X_MSB_ADDR, (config_.offset_gyr[0] >> 8) & 0xFF);
      write_byte(GYRO_OFFSET_Y_LSB_ADDR, config_.offset_gyr[1] & 0xFF);
      write_byte(GYRO_OFFSET_Y_MSB_ADDR, (config_.offset_gyr[1] >> 8) & 0xFF);
      write_byte(GYRO_OFFSET_Z_LSB_ADDR, config_.offset_gyr[2] & 0xFF);
      write_byte(GYRO_OFFSET_Z_MSB_ADDR, (config_.offset_gyr[2] >> 8) & 0xFF);
    }

    return true;
  } catch (const std::exception &) {
    return false;
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
