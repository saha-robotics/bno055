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

#ifndef BNO055__BNO055_NODE_HPP_
#define BNO055__BNO055_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "bno055/connector.hpp"
#include "bno055/sensor_service.hpp"

namespace bno055
{

class BNO055LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BNO055LifecycleNode(const rclcpp::NodeOptions & options);
  ~BNO055LifecycleNode() override = default;

protected:
  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  // Parameters
  std::string connection_type_;
  int i2c_bus_;
  uint8_t i2c_addr_;
  std::string uart_port_;
  int uart_baudrate_;
  double uart_timeout_;
  std::string frame_id_;
  double data_query_frequency_;
  double calib_status_frequency_;
  uint8_t operation_mode_;
  std::string placement_axis_remap_;
  bool set_offsets_;
  std::vector<int16_t> offset_acc_;
  std::vector<int16_t> offset_mag_;
  std::vector<int16_t> offset_gyr_;
  int16_t radius_acc_;
  int16_t radius_mag_;
  std::vector<double> variance_acc_;
  std::vector<double> variance_angular_vel_;
  std::vector<double> variance_orientation_;
  std::vector<double> variance_mag_;
  std::string ros_topic_prefix_;
  double acc_factor_;
  double mag_factor_;
  double gyr_factor_;
  double grav_factor_;

  // Hardware connector
  std::unique_ptr<Connector> connector_;
  
  // Sensor service
  std::unique_ptr<SensorService> sensor_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::TimerBase::SharedPtr calib_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Timer callbacks
  void read_data_callback();
  void log_calibration_status_callback();
  void watchdog_check_callback();

  // Helper methods
  void declare_parameters();
  bool load_parameters();
  bool create_connector();
  bool configure_sensor();
};

}  // namespace bno055

#endif  // BNO055__BNO055_NODE_HPP_
