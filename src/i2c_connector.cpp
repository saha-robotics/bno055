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

#include "bno055/i2c_connector.hpp"
#include "bno055/registers.hpp"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

namespace bno055
{

I2CConnector::I2CConnector(int bus, uint8_t address)
: bus_(bus), address_(address), fd_(-1)
{
}

I2CConnector::~I2CConnector()
{
  disconnect();
}

bool I2CConnector::connect()
{
  std::string device = "/dev/i2c-" + std::to_string(bus_);
  fd_ = open(device.c_str(), O_RDWR);
  
  if (fd_ < 0) {
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Verify chip ID
  std::vector<uint8_t> chip_id(1);
  if (!read(BNO055_CHIP_ID_ADDR, chip_id, 1)) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  if (chip_id[0] != BNO055_ID) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void I2CConnector::disconnect()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool I2CConnector::read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length)
{
  if (fd_ < 0) {
    return false;
  }

  data.resize(length);
  size_t bytes_left = length;
  size_t offset = 0;

  while (bytes_left > 0) {
    size_t read_len = std::min(bytes_left, size_t(32));
    
    // Write register address
    if (write(fd_, &reg_addr, 1) != 1) {
      return false;
    }

    // Read data
    if (::read(fd_, data.data() + offset, read_len) != static_cast<ssize_t>(read_len)) {
      return false;
    }

    bytes_left -= read_len;
    offset += read_len;
    reg_addr += read_len;
  }

  return true;
}

bool I2CConnector::write(uint8_t reg_addr, const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }

  size_t bytes_left = data.size();
  size_t offset = 0;

  while (bytes_left > 0) {
    size_t write_len = std::min(bytes_left, size_t(32));
    
    // Prepare buffer with register address followed by data
    std::vector<uint8_t> buffer(write_len + 1);
    buffer[0] = reg_addr + offset;
    std::copy(data.begin() + offset, data.begin() + offset + write_len, buffer.begin() + 1);

    if (::write(fd_, buffer.data(), buffer.size()) != static_cast<ssize_t>(buffer.size())) {
      return false;
    }

    bytes_left -= write_len;
    offset += write_len;
  }

  return true;
}

}  // namespace bno055
