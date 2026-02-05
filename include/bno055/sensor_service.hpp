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

#ifndef BNO055__SENSOR_SERVICE_HPP_
#define BNO055__SENSOR_SERVICE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/trigger.hpp"

#include "bno055/connector.hpp"

namespace bno055
{

struct SensorConfig
{
  std::string frame_id;
  double acc_factor;
  double mag_factor;
  double gyr_factor;
  double grav_factor;
  uint8_t operation_mode;
  std::string placement_axis_remap;
  bool set_offsets;
  std::vector<int16_t> offset_acc;
  std::vector<int16_t> offset_mag;
  std::vector<int16_t> offset_gyr;
  int16_t radius_acc;
  int16_t radius_mag;
  std::vector<double> variance_acc;
  std::vector<double> variance_angular_vel;
  std::vector<double> variance_orientation;
  std::vector<double> variance_mag;
  std::string topic_prefix;
};

class SensorService
{
public:
  SensorService(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<Connector> connector,
    const SensorConfig & config);

  ~SensorService() = default;

  bool configure();
  void get_sensor_data();
  void get_calib_status();
  void check_watchdog();
  
  // Publisher activation
  void activate_publishers();
  void deactivate_publishers();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<Connector> connector_;
  SensorConfig config_;

  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> pub_imu_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> pub_imu_raw_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>> pub_mag_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>> pub_grav_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>> pub_temp_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_calib_status_;

  // Service
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr calibration_service_;

  // State tracking for connection monitoring
  std::chrono::steady_clock::time_point last_successful_read_;
  int consecutive_error_count_;

  // Helper methods
  bool set_mode(uint8_t mode);
  bool read_register(uint8_t reg, std::vector<uint8_t> & data, size_t length);
  bool write_register(uint8_t reg, const std::vector<uint8_t> & data);
  int16_t bytes_to_int16(const std::vector<uint8_t> & data, size_t offset);
  void calibration_request_callback(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response);
};

}  // namespace bno055

#endif  // BNO055__SENSOR_SERVICE_HPP_
