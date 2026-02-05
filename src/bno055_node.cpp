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

#include "bno055/bno055_node.hpp"
#include "bno055/i2c_connector.hpp"
#include "bno055/uart_connector.hpp"
#include "bno055/registers.hpp"

namespace bno055
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

BNO055LifecycleNode::BNO055LifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bno055", options)
{
  RCLCPP_INFO(get_logger(), "BNO055 Lifecycle Node constructed");
  declare_parameters();
}

void BNO055LifecycleNode::declare_parameters()
{
  // Connection parameters
  declare_parameter("connection_type", "uart");
  declare_parameter("i2c_bus", 0);
  declare_parameter("i2c_addr", static_cast<int>(BNO055_ADDRESS_A));
  declare_parameter("uart_port", "/dev/ttyUSB0");
  declare_parameter("uart_baudrate", 115200);
  declare_parameter("uart_timeout", 0.1);

  // General parameters
  declare_parameter("frame_id", "bno055");
  declare_parameter("ros_topic_prefix", "bno055/");
  declare_parameter("data_query_frequency", 10.0);
  declare_parameter("calib_status_frequency", 0.1);

  // Sensor configuration
  declare_parameter("operation_mode", static_cast<int>(OPERATION_MODE_NDOF));
  declare_parameter("placement_axis_remap", "P1");
  declare_parameter("set_offsets", false);

  // Scaling factors
  declare_parameter("acc_factor", 100.0);
  declare_parameter("mag_factor", 16000000.0);
  declare_parameter("gyr_factor", 900.0);
  declare_parameter("grav_factor", 100.0);

  // Offset parameters
  declare_parameter("offset_acc", std::vector<int64_t>{-20, 165, -24});
  declare_parameter("offset_mag", std::vector<int64_t>{-76, -354, 637});
  declare_parameter("offset_gyr", std::vector<int64_t>{2, -1, -1});
  declare_parameter("radius_acc", 1000);
  declare_parameter("radius_mag", 0);

  // Variance parameters
  declare_parameter("variance_acc", std::vector<double>{0.017, 0.017, 0.017});
  declare_parameter("variance_angular_vel", std::vector<double>{0.04, 0.04, 0.04});
  declare_parameter("variance_orientation", std::vector<double>{0.0159, 0.0159, 0.0159});
  declare_parameter("variance_mag", std::vector<double>{0.0, 0.0, 0.0});
}

bool BNO055LifecycleNode::load_parameters()
{
  try {
    connection_type_ = get_parameter("connection_type").as_string();
    i2c_bus_ = get_parameter("i2c_bus").as_int();
    i2c_addr_ = static_cast<uint8_t>(get_parameter("i2c_addr").as_int());
    uart_port_ = get_parameter("uart_port").as_string();
    uart_baudrate_ = get_parameter("uart_baudrate").as_int();
    uart_timeout_ = get_parameter("uart_timeout").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    ros_topic_prefix_ = get_parameter("ros_topic_prefix").as_string();
    data_query_frequency_ = get_parameter("data_query_frequency").as_double();
    calib_status_frequency_ = get_parameter("calib_status_frequency").as_double();
    operation_mode_ = static_cast<uint8_t>(get_parameter("operation_mode").as_int());
    placement_axis_remap_ = get_parameter("placement_axis_remap").as_string();
    set_offsets_ = get_parameter("set_offsets").as_bool();
    acc_factor_ = get_parameter("acc_factor").as_double();
    mag_factor_ = get_parameter("mag_factor").as_double();
    gyr_factor_ = get_parameter("gyr_factor").as_double();
    grav_factor_ = get_parameter("grav_factor").as_double();

    // Load offset vectors
    auto offset_acc_int64 = get_parameter("offset_acc").as_integer_array();
    auto offset_mag_int64 = get_parameter("offset_mag").as_integer_array();
    auto offset_gyr_int64 = get_parameter("offset_gyr").as_integer_array();
    
    offset_acc_.resize(3);
    offset_mag_.resize(3);
    offset_gyr_.resize(3);
    
    for (size_t i = 0; i < 3; ++i) {
      offset_acc_[i] = static_cast<int16_t>(offset_acc_int64[i]);
      offset_mag_[i] = static_cast<int16_t>(offset_mag_int64[i]);
      offset_gyr_[i] = static_cast<int16_t>(offset_gyr_int64[i]);
    }

    radius_acc_ = static_cast<int16_t>(get_parameter("radius_acc").as_int());
    radius_mag_ = static_cast<int16_t>(get_parameter("radius_mag").as_int());

    variance_acc_ = get_parameter("variance_acc").as_double_array();
    variance_angular_vel_ = get_parameter("variance_angular_vel").as_double_array();
    variance_orientation_ = get_parameter("variance_orientation").as_double_array();
    variance_mag_ = get_parameter("variance_mag").as_double_array();

    RCLCPP_INFO(get_logger(), "Parameters loaded successfully");
    RCLCPP_INFO(get_logger(), "  connection_type: %s", connection_type_.c_str());
    RCLCPP_INFO(get_logger(), "  frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "  data_query_frequency: %.1f Hz", data_query_frequency_);

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load parameters: %s", e.what());
    return false;
  }
}

bool BNO055LifecycleNode::create_connector()
{
  try {
    if (connection_type_ == "i2c") {
      RCLCPP_INFO(get_logger(), "Creating I2C connector (bus=%d, addr=0x%02x)",
        i2c_bus_, i2c_addr_);
      connector_ = std::make_unique<I2CConnector>(i2c_bus_, i2c_addr_);
    } else if (connection_type_ == "uart") {
      RCLCPP_INFO(get_logger(), "Creating UART connector (port=%s, baud=%d)",
        uart_port_.c_str(), uart_baudrate_);
      connector_ = std::make_unique<UARTConnector>(uart_port_, uart_baudrate_, uart_timeout_);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported connection type: %s", connection_type_.c_str());
      return false;
    }

    if (!connector_->connect()) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to BNO055 sensor");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Successfully connected to BNO055 sensor");
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception while creating connector: %s", e.what());
    return false;
  }
}

bool BNO055LifecycleNode::configure_sensor()
{
  SensorConfig config;
  config.frame_id = frame_id_;
  config.acc_factor = acc_factor_;
  config.mag_factor = mag_factor_;
  config.gyr_factor = gyr_factor_;
  config.grav_factor = grav_factor_;
  config.operation_mode = operation_mode_;
  config.placement_axis_remap = placement_axis_remap_;
  config.set_offsets = set_offsets_;
  config.offset_acc = offset_acc_;
  config.offset_mag = offset_mag_;
  config.offset_gyr = offset_gyr_;
  config.radius_acc = radius_acc_;
  config.radius_mag = radius_mag_;
  config.variance_acc = variance_acc_;
  config.variance_angular_vel = variance_angular_vel_;
  config.variance_orientation = variance_orientation_;
  config.variance_mag = variance_mag_;
  config.topic_prefix = ros_topic_prefix_;

  sensor_service_ = std::make_unique<SensorService>(
    shared_from_this(), connector_, config);

  if (!sensor_service_->configure()) {
    RCLCPP_ERROR(get_logger(), "Failed to configure sensor");
    return false;
  }

  return true;
}

CallbackReturn BNO055LifecycleNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  // Load parameters
  if (!load_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to load parameters");
    return CallbackReturn::FAILURE;
  }

  // Create hardware connector
  if (!create_connector()) {
    RCLCPP_ERROR(get_logger(), "Failed to create connector");
    return CallbackReturn::FAILURE;
  }

  // Configure sensor
  if (!configure_sensor()) {
    RCLCPP_ERROR(get_logger(), "Failed to configure sensor");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Configuration complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO055LifecycleNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  // Activate publishers
  if (sensor_service_) {
    sensor_service_->activate_publishers();
  }

  // Create and start data query timer
  if (data_query_frequency_ > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / data_query_frequency_);
    data_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BNO055LifecycleNode::read_data_callback, this));
    RCLCPP_INFO(get_logger(), "Data timer started at %.1f Hz", data_query_frequency_);
  }

  // Create and start calibration status timer
  if (calib_status_frequency_ > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / calib_status_frequency_);
    calib_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BNO055LifecycleNode::log_calibration_status_callback, this));
    RCLCPP_INFO(get_logger(), "Calibration timer started at %.1f Hz", calib_status_frequency_);
  }

  // Create and start watchdog timer (runs every 0.5 seconds)
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&BNO055LifecycleNode::watchdog_check_callback, this));
  RCLCPP_INFO(get_logger(), "Watchdog timer started");

  RCLCPP_INFO(get_logger(), "Activation complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO055LifecycleNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  // Stop timers
  if (data_timer_) {
    data_timer_->cancel();
    data_timer_.reset();
  }

  if (calib_timer_) {
    calib_timer_->cancel();
    calib_timer_.reset();
  }

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }

  // Deactivate publishers
  if (sensor_service_) {
    sensor_service_->deactivate_publishers();
  }

  RCLCPP_INFO(get_logger(), "Deactivation complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO055LifecycleNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning up...");

  // Release sensor service
  sensor_service_.reset();

  // Disconnect hardware
  if (connector_) {
    connector_->disconnect();
    connector_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO055LifecycleNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting down...");

  // Stop timers
  if (data_timer_) {
    data_timer_->cancel();
    data_timer_.reset();
  }

  if (calib_timer_) {
    calib_timer_->cancel();
    calib_timer_.reset();
  }

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }

  // Release resources
  sensor_service_.reset();

  if (connector_) {
    connector_->disconnect();
    connector_.reset();
  }

  RCLCPP_INFO(get_logger(), "Shutdown complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO055LifecycleNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_ERROR(get_logger(), "Error state reached - performing cleanup");

  // Attempt to stop timers safely
  try {
    if (data_timer_) {
      data_timer_->cancel();
      data_timer_.reset();
    }
    if (calib_timer_) {
      calib_timer_->cancel();
      calib_timer_.reset();
    }
    if (watchdog_timer_) {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception while stopping timers: %s", e.what());
  }

  // Attempt to disconnect hardware safely
  try {
    if (connector_ && connector_->is_connected()) {
      connector_->disconnect();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception while disconnecting: %s", e.what());
  }

  return CallbackReturn::SUCCESS;
}

void BNO055LifecycleNode::read_data_callback()
{
  if (!sensor_service_) {
    return;
  }

  try {
    sensor_service_->get_sensor_data();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Exception in read_data_callback: %s", e.what());
  }
}

void BNO055LifecycleNode::log_calibration_status_callback()
{
  if (!sensor_service_) {
    return;
  }

  try {
    sensor_service_->get_calib_status();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Exception in log_calibration_status_callback: %s", e.what());
  }
}

void BNO055LifecycleNode::watchdog_check_callback()
{
  if (!sensor_service_) {
    return;
  }

  try {
    sensor_service_->check_watchdog();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Exception in watchdog_check_callback: %s", e.what());
  }
}

}  // namespace bno055
