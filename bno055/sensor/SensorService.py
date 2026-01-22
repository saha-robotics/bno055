# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import json
from math import sqrt
import struct
import sys
from time import sleep, time

from bno055 import registers
from bno055.connectors.Connector import Connector
from bno055.params.NodeParameters import NodeParameters

from geometry_msgs.msg import Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String, Bool
from example_interfaces.srv import Trigger
from robot_msgs.msg import Log, RosTopicsConfig


class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node: Node, connector: Connector, param: NodeParameters):
        self.node = node
        self.con = connector
        self.param = param

        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)
        LatchQoSProf = QoSProfile(
            depth=1,  
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # create topic publishers:
        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', QoSProf)
        self.pub_imu = node.create_publisher(Imu, prefix + 'imu', QoSProf)
        self.pub_mag = node.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_grav = node.create_publisher(Vector3, prefix + 'grav', QoSProf)
        self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', QoSProf)
        self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', QoSProf)
        self.pub_imu_ok = node.create_publisher(Bool, prefix + 'imu_ok', LatchQoSProf)
        self.pub_global_log = node.create_publisher(Log, RosTopicsConfig.LOGGING, 10)
        self.srv = self.node.create_service(Trigger, prefix + 'calibration_request', self.calibration_request_callback)

        # IMU anomaly detection state variables
        self.prev_accel_x = 0.0
        self.prev_accel_y = 0.0
        self.prev_accel_z = 0.0
        self.prev_gyro_x = 0.0
        self.prev_gyro_y = 0.0
        self.prev_gyro_z = 0.0
        self.prev_imu_time = None
        self.imu_constant_count = 0
        self.imu_ok = True
        self.prev_imu_ok = False
        
        # History buffers for std dev calculation
        self.accel_x_history = []
        self.accel_y_history = []
        self.accel_z_history = []
        
        # Log throttle map
        self.log_throttle_map = {}
        
        # Serial reset timeout tracking
        self.last_successful_data_time = time()
        self.reset_in_progress = False

    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info('Configuring device...')
        try:
            data = self.con.receive(registers.BNO055_CHIP_ID_ADDR, 1)
            if data[0] != registers.BNO055_ID:
                raise IOError('Device ID=%s is incorrect' % data)
            # print("device sent ", binascii.hexlify(data))
        except Exception as e:  # noqa: B902
            # This is the first communication - exit if it does not work
            self.node.get_logger().error('Communication error: %s' % e)
            self.node.get_logger().error('Shutting down ROS node...')
            sys.exit(1)

        # IMU connected => apply IMU Configuration:
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')

        if not (self.con.transmit(registers.BNO055_PWR_MODE_ADDR, 1, bytes([registers.POWER_MODE_NORMAL]))):
            self.node.get_logger().warn('Unable to set IMU normal power mode.')

        if not (self.con.transmit(registers.BNO055_PAGE_ID_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to set IMU register page 0.')

        if not (self.con.transmit(registers.BNO055_SYS_TRIGGER_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to start IMU.')

        if not (self.con.transmit(registers.BNO055_UNIT_SEL_ADDR, 1, bytes([0x83]))):
            self.node.get_logger().warn('Unable to set IMU units.')

        # The sensor placement configuration (Axis remapping) defines the
        # position and orientation of the sensor mount.
        # See also Bosch BNO055 datasheet section Axis Remap
        mount_positions = {
            'P0': bytes(b'\x21\x04'),
            'P1': bytes(b'\x24\x00'),
            'P2': bytes(b'\x24\x06'),
            'P3': bytes(b'\x21\x02'),
            'P4': bytes(b'\x24\x03'),
            'P5': bytes(b'\x21\x02'),
            'P6': bytes(b'\x21\x07'),
            'P7': bytes(b'\x24\x05')
        }
        if not (self.con.transmit(registers.BNO055_AXIS_MAP_CONFIG_ADDR, 2,
                                  mount_positions[self.param.placement_axis_remap.value])):
            self.node.get_logger().warn('Unable to set sensor placement configuration.')

        # Show the current sensor offsets
        self.node.get_logger().info('Current sensor offsets:')
        self.print_calib_data()
        if self.param.set_offsets.value:
            configured_offsets = \
                self.set_calib_offsets(
                    self.param.offset_acc,
                    self.param.offset_mag,
                    self.param.offset_gyr,
                    self.param.radius_mag,
                    self.param.radius_acc)
            if configured_offsets:
                self.node.get_logger().info('Successfully configured sensor offsets to:')
                self.print_calib_data()
            else:
                self.node.get_logger().warn('setting offsets failed')


        # Set Device mode
        device_mode = self.param.operation_mode.value
        self.node.get_logger().info(f"Setting device_mode to {device_mode}")

        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([device_mode]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')

        self.node.get_logger().info('Bosch BNO055 IMU configuration complete.')

    def _check_serial_timeout(self):
        """Check if serial connection should be reset due to data timeout.
        
        :return: True if reset was triggered, False otherwise
        """
        timeout = self.param.serial_reset_timeout.value
        if timeout <= 0.0:
            return False  # Feature disabled
        
        current_time = time()
        elapsed = current_time - self.last_successful_data_time
        
        if elapsed >= timeout and not self.reset_in_progress:
            self.reset_in_progress = True
            
            # Set imu_ok to false when timeout occurs
            if self.imu_ok:
                self.imu_ok = False
                imu_ok_msg = Bool()
                imu_ok_msg.data = False
                self.pub_imu_ok.publish(imu_ok_msg)
                self.prev_imu_ok = False
            
            msg = (
                f"Serial timeout detected: No data received for {elapsed:.2f} seconds "
                f"(threshold: {timeout:.1f}s). Resetting serial connection..."
            )
            self.publish_log_throttled("serial_timeout", msg, Log.ERROR, 10.0)
            
            # Reset the connector
            if self.con.reset():
                # Reconfigure the sensor after reset
                try:
                    self.configure()
                    self.last_successful_data_time = time()
                    self.node.get_logger().info('Sensor reconfigured after serial reset')
                except Exception as e:
                    self.node.get_logger().error(f'Failed to reconfigure sensor after reset: {e}')
            
            self.reset_in_progress = False
            return True
        
        return False

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        # Check for serial reset timeout
        if self._check_serial_timeout():
            return  # Skip this cycle if reset was triggered
        
        # Initialize ROS msgs
        imu_raw_msg = Imu()
        imu_msg = Imu()
        mag_msg = MagneticField()
        grav_msg = Vector3()
        temp_msg = Temperature()

        # read from sensor
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        
        # Update last successful data time
        self.last_successful_data_time = time()
        # Publish raw data
        imu_raw_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_raw_msg.header.frame_id = self.param.frame_id.value
        # TODO: do headers need sequence counters now?
        # imu_raw_msg.header.seq = seq

        # TODO: make this an option to publish?
        imu_raw_msg.orientation_covariance = [
            self.param.variance_orientation.value[0], 0.0, 0.0,
            0.0, self.param.variance_orientation.value[1], 0.0,
            0.0, 0.0, self.param.variance_orientation.value[2]
        ]

        imu_raw_msg.linear_acceleration.x = \
            self.unpackBytesToFloat(buf[0], buf[1]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.y = \
            self.unpackBytesToFloat(buf[2], buf[3]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.z = \
            self.unpackBytesToFloat(buf[4], buf[5]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration_covariance = [
            self.param.variance_acc.value[0], 0.0, 0.0,
            0.0, self.param.variance_acc.value[1], 0.0,
            0.0, 0.0, self.param.variance_acc.value[2]
        ]
        imu_raw_msg.angular_velocity.x = \
            self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.y = \
            self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.z = \
            self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity_covariance = [
            self.param.variance_angular_vel.value[0], 0.0, 0.0,
            0.0, self.param.variance_angular_vel.value[1], 0.0,
            0.0, 0.0, self.param.variance_angular_vel.value[2]
        ]
        # node.get_logger().info('Publishing imu message')
        self.pub_imu_raw.publish(imu_raw_msg)

        # TODO: make this an option to publish?
        # Publish filtered data
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.param.frame_id.value

        q = Quaternion()
        # imu_msg.header.seq = seq
        q.w = self.unpackBytesToFloat(buf[24], buf[25])
        q.x = self.unpackBytesToFloat(buf[26], buf[27])
        q.y = self.unpackBytesToFloat(buf[28], buf[29])
        q.z = self.unpackBytesToFloat(buf[30], buf[31])
        # TODO(flynneva): replace with standard normalize() function
        # normalize
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        imu_msg.orientation.x = q.x / norm
        imu_msg.orientation.y = q.y / norm
        imu_msg.orientation.z = q.z / norm
        imu_msg.orientation.w = q.w / norm

        imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance

        imu_msg.linear_acceleration.x = \
            self.unpackBytesToFloat(buf[32], buf[33]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.y = \
            self.unpackBytesToFloat(buf[34], buf[35]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.z = \
            self.unpackBytesToFloat(buf[36], buf[37]) / self.param.acc_factor.value
        imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
        imu_msg.angular_velocity.x = \
            self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.y = \
            self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.z = \
            self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor.value
        imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data
        mag_msg.header.stamp = self.node.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.param.frame_id.value
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = \
            self.unpackBytesToFloat(buf[6], buf[7]) / self.param.mag_factor.value
        mag_msg.magnetic_field.y = \
            self.unpackBytesToFloat(buf[8], buf[9]) / self.param.mag_factor.value
        mag_msg.magnetic_field.z = \
            self.unpackBytesToFloat(buf[10], buf[11]) / self.param.mag_factor.value
        mag_msg.magnetic_field_covariance = [
            self.param.variance_mag.value[0], 0.0, 0.0,
            0.0, self.param.variance_mag.value[1], 0.0,
            0.0, 0.0, self.param.variance_mag.value[2]
        ]
        self.pub_mag.publish(mag_msg)

        grav_msg.x = \
            self.unpackBytesToFloat(buf[38], buf[39]) / self.param.grav_factor.value
        grav_msg.y = \
            self.unpackBytesToFloat(buf[40], buf[41]) / self.param.grav_factor.value
        grav_msg.z = \
            self.unpackBytesToFloat(buf[42], buf[43]) / self.param.grav_factor.value
        self.pub_grav.publish(grav_msg)

        # Publish temperature
        temp_msg.header.stamp = self.node.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.param.frame_id.value
        # temp_msg.header.seq = seq
        temp_msg.temperature = float(buf[44])
        self.pub_temp.publish(temp_msg)

        # IMU anomaly detection
        self._check_imu_anomalies(imu_msg)

    def publish_log_throttled(self, log_key, log_message, log_type, timeout_sec):
        current_time = time()
        if log_key not in self.log_throttle_map or (current_time - self.log_throttle_map[log_key]) >= timeout_sec:
            # Console logging
            if log_type == Log.WARN:
                self.node.get_logger().warn(log_message)
            elif log_type == Log.ERROR:
                self.node.get_logger().error(log_message)
            else:
                self.node.get_logger().info(log_message)
            
            # Topic logging
            log_msg = Log()
            log_msg.node = "bno055" # or self.node.get_name() but self.node.get_name() might be fully qualified
            log_msg.type = log_type
            log_msg.log = log_message
            self.pub_global_log.publish(log_msg)
            
            self.log_throttle_map[log_key] = current_time

    def _check_imu_anomalies(self, imu_msg):
        """Check for IMU anomalies and publish imu_ok status."""
        current_time = time()
        
        if self.prev_imu_time is not None:
            # Check if IMU data is constant
            is_constant = (
                abs(imu_msg.linear_acceleration.x - self.prev_accel_x) < self.param.imu_change_epsilon.value and
                abs(imu_msg.linear_acceleration.y - self.prev_accel_y) < self.param.imu_change_epsilon.value and
                abs(imu_msg.linear_acceleration.z - self.prev_accel_z) < self.param.imu_change_epsilon.value and
                abs(imu_msg.angular_velocity.x - self.prev_gyro_x) < self.param.imu_change_epsilon.value and
                abs(imu_msg.angular_velocity.y - self.prev_gyro_y) < self.param.imu_change_epsilon.value and
                abs(imu_msg.angular_velocity.z - self.prev_gyro_z) < self.param.imu_change_epsilon.value
            )

            if is_constant:
                self.imu_constant_count += 1
                if self.imu_constant_count >= self.param.imu_constant_threshold.value:
                    freq = self.param.data_query_frequency.value if hasattr(self.param, 'data_query_frequency') else 100.0
                    msg = (
                        f"IMU anomaly detected: Data constant for {self.imu_constant_count / freq:.2f} seconds "
                        f"(accel: {imu_msg.linear_acceleration.x:.3f}, {imu_msg.linear_acceleration.y:.3f}, {imu_msg.linear_acceleration.z:.3f} | "
                        f"gyro: {imu_msg.angular_velocity.x:.3f}, {imu_msg.angular_velocity.y:.3f}, {imu_msg.angular_velocity.z:.3f})"
                    )
                    self.publish_log_throttled("imu_constant", msg, Log.WARN, 5.0)

                    self.imu_ok = False
            else:
                self.imu_constant_count = 0
                self.imu_ok = True

        # Check for abnormal acceleration values using standard deviation
        self.accel_x_history.append(imu_msg.linear_acceleration.x)
        self.accel_y_history.append(imu_msg.linear_acceleration.y)
        self.accel_z_history.append(imu_msg.linear_acceleration.z)

        if len(self.accel_x_history) > self.param.imu_history_size.value:
            self.accel_x_history.pop(0)
            self.accel_y_history.pop(0)
            self.accel_z_history.pop(0)

        if len(self.accel_x_history) >= self.param.imu_history_size.value // 2:
            # Calculate mean
            mean_x = sum(self.accel_x_history) / len(self.accel_x_history)
            mean_y = sum(self.accel_y_history) / len(self.accel_y_history)
            mean_z = sum(self.accel_z_history) / len(self.accel_z_history)

            # Calculate variance
            variance_x = sum((x - mean_x) ** 2 for x in self.accel_x_history) / len(self.accel_x_history)
            variance_y = sum((y - mean_y) ** 2 for y in self.accel_y_history) / len(self.accel_y_history)
            variance_z = sum((z - mean_z) ** 2 for z in self.accel_z_history) / len(self.accel_z_history)

            # Calculate standard deviation
            std_dev_x = sqrt(variance_x)
            std_dev_y = sqrt(variance_y)
            std_dev_z = sqrt(variance_z)

            if (std_dev_x > self.param.max_std_dev_threshold.value or
                std_dev_y > self.param.max_std_dev_threshold.value or
                std_dev_z > self.param.max_std_dev_threshold.value):
                
                msg = (
                    f"IMU anomaly detected: Standard deviation too high (noisy/broken data). "
                    f"Std Dev: ({std_dev_x:.4f}, {std_dev_y:.4f}, {std_dev_z:.4f}) | "
                    f"Threshold: {self.param.max_std_dev_threshold.value:.2f} m/s² | "
                    f"Mean: ({mean_x:.3f}, {mean_y:.3f}, {mean_z:.3f})"
                )
                self.publish_log_throttled("imu_std_dev", msg, Log.WARN, 5.0)
                self.imu_ok = False

        # Update previous values
        self.prev_accel_x = imu_msg.linear_acceleration.x
        self.prev_accel_y = imu_msg.linear_acceleration.y
        self.prev_accel_z = imu_msg.linear_acceleration.z
        self.prev_gyro_x = imu_msg.angular_velocity.x
        self.prev_gyro_y = imu_msg.angular_velocity.y
        self.prev_gyro_z = imu_msg.angular_velocity.z
        self.prev_imu_time = current_time

        # Publish imu_ok status only when it changes
        self._publish_imu_ok()

    def _publish_imu_ok(self):
        """Publish imu_ok status if it has changed."""
        if self.imu_ok != self.prev_imu_ok:
            imu_ok_msg = Bool()
            imu_ok_msg.data = self.imu_ok
            self.pub_imu_ok.publish(imu_ok_msg)
            self.prev_imu_ok = self.imu_ok

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        calib_status_str = String()
        calib_status_str.data = json.dumps(calib_status_dict)

        # Publish via ROS topic:
        self.pub_calib_status.publish(calib_status_str)

    def get_calib_data(self):
        """Read all calibration data."""

        accel_offset_read = self.con.receive(registers.ACCEL_OFFSET_X_LSB_ADDR, 6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        accel_radius_read = self.con.receive(registers.ACCEL_RADIUS_LSB_ADDR, 2)
        accel_radius_read_value = (accel_radius_read[1] << 8) | accel_radius_read[0]

        mag_offset_read = self.con.receive(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_radius_read = self.con.receive(registers.MAG_RADIUS_LSB_ADDR, 2)
        mag_radius_read_value = (mag_radius_read[1] << 8) | mag_radius_read[0]

        gyro_offset_read = self.con.receive(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        calib_data = {'accel_offset': {'x': accel_offset_read_x, 'y': accel_offset_read_y, 'z': accel_offset_read_z}, 'accel_radius': accel_radius_read_value,
                      'mag_offset': {'x': mag_offset_read_x, 'y': mag_offset_read_y, 'z': mag_offset_read_z}, 'mag_radius': mag_radius_read_value,
                      'gyro_offset': {'x': gyro_offset_read_x, 'y': gyro_offset_read_y, 'z': gyro_offset_read_z}}

        return calib_data

    def print_calib_data(self):
        """Read all calibration data and print to screen."""
        calib_data = self.get_calib_data()
        self.node.get_logger().info(
            '\tAccel offsets (x y z): %d %d %d' % (
                calib_data['accel_offset']['x'],
                calib_data['accel_offset']['y'],
                calib_data['accel_offset']['z']))

        self.node.get_logger().info(
            '\tAccel radius: %d' % (
                calib_data['accel_radius'],
            )
        )

        self.node.get_logger().info(
            '\tMag offsets (x y z): %d %d %d' % (
                calib_data['mag_offset']['x'],
                calib_data['mag_offset']['y'],
                calib_data['mag_offset']['z']))

        self.node.get_logger().info(
            '\tMag radius: %d' % (
                calib_data['mag_radius'],
            )
        )

        self.node.get_logger().info(
            '\tGyro offsets (x y z): %d %d %d' % (
                calib_data['gyro_offset']['x'],
                calib_data['gyro_offset']['y'],
                calib_data['gyro_offset']['z']))

    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset, mag_radius, acc_radius):
        """
        Write calibration data (define as 16 bit signed hex).

        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        :param mag_radius:
        :param acc_radius:
        """
        # Must switch to config mode to write out
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().error('Unable to set IMU into config mode')
        sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.con.transmit(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset.value[0] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset.value[1] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset.value[2] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.ACCEL_RADIUS_LSB_ADDR, 1, bytes([acc_radius.value & 0xFF]))
            self.con.transmit(registers.ACCEL_RADIUS_MSB_ADDR, 1, bytes([(acc_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset.value[0] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset.value[1] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset.value[2] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_RADIUS_LSB_ADDR, 1, bytes([mag_radius.value & 0xFF]))
            self.con.transmit(registers.MAG_RADIUS_MSB_ADDR, 1, bytes([(mag_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset.value[0] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset.value[1] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset.value[2] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset.value[2] >> 8) & 0xFF]))

            return True
        except Exception:  # noqa: B902
            return False

    def calibration_request_callback(self, request, response):
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')
        sleep(0.025)
        calib_data = self.get_calib_data()
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')
        response.success = True
        response.message = str(calib_data)
        return response

    def unpackBytesToFloat(self, start, end):
        return float(struct.unpack('h', struct.pack('BB', start, end))[0])
