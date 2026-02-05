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

#ifndef BNO055__REGISTERS_HPP_
#define BNO055__REGISTERS_HPP_

#include <cstdint>
#include <array>

namespace bno055
{

// I2C addresses
constexpr uint8_t BNO055_ADDRESS_A = 0x28;
constexpr uint8_t BNO055_ADDRESS_B = 0x29;
constexpr uint8_t BNO055_ID = 0xA0;

// Page id register definition
constexpr uint8_t BNO055_PAGE_ID_ADDR = 0x07;

// PAGE0 REGISTER DEFINITION START
constexpr uint8_t BNO055_CHIP_ID_ADDR = 0x00;
constexpr uint8_t BNO055_ACCEL_REV_ID_ADDR = 0x01;
constexpr uint8_t BNO055_MAG_REV_ID_ADDR = 0x02;
constexpr uint8_t BNO055_GYRO_REV_ID_ADDR = 0x03;
constexpr uint8_t BNO055_SW_REV_ID_LSB_ADDR = 0x04;
constexpr uint8_t BNO055_SW_REV_ID_MSB_ADDR = 0x05;
constexpr uint8_t BNO055_BL_REV_ID_ADDR = 0x06;

// Accel data register
constexpr uint8_t BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08;
constexpr uint8_t BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09;
constexpr uint8_t BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A;
constexpr uint8_t BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0B;
constexpr uint8_t BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C;
constexpr uint8_t BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0D;

// Mag data register
constexpr uint8_t BNO055_MAG_DATA_X_LSB_ADDR = 0x0E;
constexpr uint8_t BNO055_MAG_DATA_X_MSB_ADDR = 0x0F;
constexpr uint8_t BNO055_MAG_DATA_Y_LSB_ADDR = 0x10;
constexpr uint8_t BNO055_MAG_DATA_Y_MSB_ADDR = 0x11;
constexpr uint8_t BNO055_MAG_DATA_Z_LSB_ADDR = 0x12;
constexpr uint8_t BNO055_MAG_DATA_Z_MSB_ADDR = 0x13;

// Gyro data registers
constexpr uint8_t BNO055_GYRO_DATA_X_LSB_ADDR = 0x14;
constexpr uint8_t BNO055_GYRO_DATA_X_MSB_ADDR = 0x15;
constexpr uint8_t BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16;
constexpr uint8_t BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17;
constexpr uint8_t BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18;
constexpr uint8_t BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19;

// Euler data registers
constexpr uint8_t BNO055_EULER_H_LSB_ADDR = 0x1A;
constexpr uint8_t BNO055_EULER_H_MSB_ADDR = 0x1B;
constexpr uint8_t BNO055_EULER_R_LSB_ADDR = 0x1C;
constexpr uint8_t BNO055_EULER_R_MSB_ADDR = 0x1D;
constexpr uint8_t BNO055_EULER_P_LSB_ADDR = 0x1E;
constexpr uint8_t BNO055_EULER_P_MSB_ADDR = 0x1F;

// Quaternion data registers
constexpr uint8_t BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20;
constexpr uint8_t BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21;
constexpr uint8_t BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22;
constexpr uint8_t BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23;
constexpr uint8_t BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24;
constexpr uint8_t BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25;
constexpr uint8_t BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26;
constexpr uint8_t BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27;

// Linear acceleration data registers
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29;
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2A;
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2B;
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2C;
constexpr uint8_t BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2D;

// Gravity data registers
constexpr uint8_t BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2E;
constexpr uint8_t BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2F;
constexpr uint8_t BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30;
constexpr uint8_t BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31;
constexpr uint8_t BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32;
constexpr uint8_t BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33;

// Temperature data register
constexpr uint8_t BNO055_TEMP_ADDR = 0x34;

// Status registers
constexpr uint8_t BNO055_CALIB_STAT_ADDR = 0x35;
constexpr uint8_t BNO055_SELFTEST_RESULT_ADDR = 0x36;
constexpr uint8_t BNO055_INTR_STAT_ADDR = 0x37;
constexpr uint8_t BNO055_SYS_CLK_STAT_ADDR = 0x38;
constexpr uint8_t BNO055_SYS_STAT_ADDR = 0x39;
constexpr uint8_t BNO055_SYS_ERR_ADDR = 0x3A;

// Unit selection register
constexpr uint8_t BNO055_UNIT_SEL_ADDR = 0x3B;
constexpr uint8_t BNO055_DATA_SELECT_ADDR = 0x3C;

// Mode registers
constexpr uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
constexpr uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
constexpr uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
constexpr uint8_t BNO055_TEMP_SOURCE_ADDR = 0x40;

// Axis remap registers
constexpr uint8_t BNO055_AXIS_MAP_CONFIG_ADDR = 0x41;
constexpr uint8_t BNO055_AXIS_MAP_SIGN_ADDR = 0x42;

// Axis remap values
constexpr uint8_t AXIS_REMAP_X = 0x00;
constexpr uint8_t AXIS_REMAP_Y = 0x01;
constexpr uint8_t AXIS_REMAP_Z = 0x02;
constexpr uint8_t AXIS_REMAP_POSITIVE = 0x00;
constexpr uint8_t AXIS_REMAP_NEGATIVE = 0x01;

// Offset registers
constexpr uint8_t ACCEL_OFFSET_X_LSB_ADDR = 0x55;
constexpr uint8_t ACCEL_OFFSET_X_MSB_ADDR = 0x56;
constexpr uint8_t ACCEL_OFFSET_Y_LSB_ADDR = 0x57;
constexpr uint8_t ACCEL_OFFSET_Y_MSB_ADDR = 0x58;
constexpr uint8_t ACCEL_OFFSET_Z_LSB_ADDR = 0x59;
constexpr uint8_t ACCEL_OFFSET_Z_MSB_ADDR = 0x5A;

constexpr uint8_t MAG_OFFSET_X_LSB_ADDR = 0x5B;
constexpr uint8_t MAG_OFFSET_X_MSB_ADDR = 0x5C;
constexpr uint8_t MAG_OFFSET_Y_LSB_ADDR = 0x5D;
constexpr uint8_t MAG_OFFSET_Y_MSB_ADDR = 0x5E;
constexpr uint8_t MAG_OFFSET_Z_LSB_ADDR = 0x5F;
constexpr uint8_t MAG_OFFSET_Z_MSB_ADDR = 0x60;

constexpr uint8_t GYRO_OFFSET_X_LSB_ADDR = 0x61;
constexpr uint8_t GYRO_OFFSET_X_MSB_ADDR = 0x62;
constexpr uint8_t GYRO_OFFSET_Y_LSB_ADDR = 0x63;
constexpr uint8_t GYRO_OFFSET_Y_MSB_ADDR = 0x64;
constexpr uint8_t GYRO_OFFSET_Z_LSB_ADDR = 0x65;
constexpr uint8_t GYRO_OFFSET_Z_MSB_ADDR = 0x66;

// Radius registers
constexpr uint8_t ACCEL_RADIUS_LSB_ADDR = 0x67;
constexpr uint8_t ACCEL_RADIUS_MSB_ADDR = 0x68;
constexpr uint8_t MAG_RADIUS_LSB_ADDR = 0x69;
constexpr uint8_t MAG_RADIUS_MSB_ADDR = 0x6A;

// Power modes
constexpr uint8_t POWER_MODE_NORMAL = 0x00;
constexpr uint8_t POWER_MODE_LOWPOWER = 0x01;
constexpr uint8_t POWER_MODE_SUSPEND = 0x02;

// Operation mode settings
constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
constexpr uint8_t OPERATION_MODE_ACCONLY = 0x01;
constexpr uint8_t OPERATION_MODE_MAGONLY = 0x02;
constexpr uint8_t OPERATION_MODE_GYRONLY = 0x03;
constexpr uint8_t OPERATION_MODE_ACCMAG = 0x04;
constexpr uint8_t OPERATION_MODE_ACCGYRO = 0x05;
constexpr uint8_t OPERATION_MODE_MAGGYRO = 0x06;
constexpr uint8_t OPERATION_MODE_AMG = 0x07;
constexpr uint8_t OPERATION_MODE_IMUPLUS = 0x08;
constexpr uint8_t OPERATION_MODE_COMPASS = 0x09;
constexpr uint8_t OPERATION_MODE_M4G = 0x0A;
constexpr uint8_t OPERATION_MODE_NDOF_FMC_OFF = 0x0B;
constexpr uint8_t OPERATION_MODE_NDOF = 0x0C;

// Communication constants
constexpr uint8_t COM_START_BYTE_WR = 0xAA;
constexpr uint8_t COM_START_BYTE_RESP = 0xBB;
constexpr uint8_t COM_START_BYTE_ERROR_RESP = 0xEE;
constexpr uint8_t COM_READ = 0x01;
constexpr uint8_t COM_WRITE = 0x00;

// Default calibration values
constexpr std::array<int16_t, 3> DEFAULT_OFFSET_ACC = {-20, 165, -24};
constexpr std::array<int16_t, 3> DEFAULT_OFFSET_MAG = {-76, -354, 637};
constexpr std::array<int16_t, 3> DEFAULT_OFFSET_GYR = {2, -1, -1};
constexpr int16_t DEFAULT_RADIUS_MAG = 0x0;
constexpr int16_t DEFAULT_RADIUS_ACC = 0x3E8;

// Sensor standard deviation squared (variance) defaults
constexpr std::array<double, 3> DEFAULT_VARIANCE_ACC = {0.017, 0.017, 0.017};
constexpr std::array<double, 3> DEFAULT_VARIANCE_ANGULAR_VEL = {0.04, 0.04, 0.04};
constexpr std::array<double, 3> DEFAULT_VARIANCE_ORIENTATION = {0.0159, 0.0159, 0.0159};
constexpr std::array<double, 3> DEFAULT_VARIANCE_MAG = {0.0, 0.0, 0.0};

}  // namespace bno055

#endif  // BNO055__REGISTERS_HPP_
