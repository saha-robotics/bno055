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

#include "bno055/uart_connector.hpp"
#include "bno055/registers.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <algorithm>

namespace bno055
{

UARTConnector::UARTConnector(const std::string & port, int baudrate, double timeout)
: port_(port), baudrate_(baudrate), timeout_(timeout), fd_(-1)
{
}

UARTConnector::~UARTConnector()
{
  disconnect();
}

bool UARTConnector::connect()
{
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  
  if (fd_ < 0) {
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baudrate_) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B115200; break;
  }
  
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 mode
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Non-canonical mode
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Timeout configuration
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_ * 10);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Verify connection by reading chip ID
  std::vector<uint8_t> chip_id;
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

void UARTConnector::disconnect()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool UARTConnector::read_response(std::vector<uint8_t> & data, size_t expected_length)
{
  // Read response header (2 bytes: start byte + length)
  std::vector<uint8_t> header(2);
  if (::read(fd_, header.data(), 2) != 2) {
    return false;
  }

  if (header[0] == COM_START_BYTE_ERROR_RESP) {
    return false;
  }

  if (header[0] != COM_START_BYTE_RESP) {
    return false;
  }

  uint8_t response_length = header[1];
  if (response_length != expected_length) {
    return false;
  }

  // Read data
  data.resize(expected_length);
  if (::read(fd_, data.data(), expected_length) != static_cast<ssize_t>(expected_length)) {
    return false;
  }

  return true;
}

bool UARTConnector::read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length)
{
  if (fd_ < 0) {
    return false;
  }

  // Build read command: [start_byte, read_cmd, reg_addr, length]
  std::vector<uint8_t> cmd = {COM_START_BYTE_WR, COM_READ, reg_addr, static_cast<uint8_t>(length)};
  
  if (::write(fd_, cmd.data(), cmd.size()) != static_cast<ssize_t>(cmd.size())) {
    return false;
  }

  return read_response(data, length);
}

bool UARTConnector::write(uint8_t reg_addr, const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }

  // Build write command: [start_byte, write_cmd, reg_addr, length, data...]
  std::vector<uint8_t> cmd;
  cmd.push_back(COM_START_BYTE_WR);
  cmd.push_back(COM_WRITE);
  cmd.push_back(reg_addr);
  cmd.push_back(static_cast<uint8_t>(data.size()));
  cmd.insert(cmd.end(), data.begin(), data.end());

  if (::write(fd_, cmd.data(), cmd.size()) != static_cast<ssize_t>(cmd.size())) {
    return false;
  }

  // Read write response (1 byte status)
  std::vector<uint8_t> response;
  return read_response(response, 1);
}

}  // namespace bno055
