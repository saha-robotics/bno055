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
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <thread>
#include <chrono>

namespace bno055
{

UARTConnector::UARTConnector(const std::string & port, int baudrate, double timeout)
: port_(port), baudrate_(baudrate), timeout_(timeout)
{
}

UARTConnector::~UARTConnector()
{
  disconnect();
}

bool UARTConnector::connect()
{
  try {
    // Timeout matches Python pyserial: serial.Serial(port, baud, timeout=X)
    // simpleTimeout sets the total read timeout in milliseconds
    uint32_t timeout_ms = static_cast<uint32_t>(timeout_ * 1000);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);

    serial_ = std::make_unique<serial::Serial>(
      port_,
      static_cast<uint32_t>(baudrate_),
      to,
      serial::eightbits,
      serial::parity_none,
      serial::stopbits_one,
      serial::flowcontrol_none
    );

    if (!serial_->isOpen()) {
      serial_.reset();
      return false;
    }

    // Flush stale data from previous session
    serial_->flush();

    // Verify connection by reading chip ID
    std::vector<uint8_t> chip_id;
    bool chip_id_ok = false;
    for (int attempt = 0; attempt < 5; attempt++) {
      if (attempt > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        serial_->flush();
      }
      chip_id.clear();
      if (read(BNO055_CHIP_ID_ADDR, chip_id, 1) &&
          chip_id.size() == 1 && chip_id[0] == BNO055_ID)
      {
        chip_id_ok = true;
        break;
      }
    }

    if (!chip_id_ok) {
      serial_->close();
      serial_.reset();
      return false;
    }

    return true;
  } catch (const serial::IOException & e) {
    fprintf(stderr, "[BNO055] serial::IOException: %s\n", e.what());
    serial_.reset();
    return false;
  } catch (const std::exception & e) {
    fprintf(stderr, "[BNO055] Exception during connect: %s\n", e.what());
    serial_.reset();
    return false;
  }
}

void UARTConnector::disconnect()
{
  if (serial_ && serial_->isOpen()) {
    try {
      serial_->close();
    } catch (...) {}
  }
  serial_.reset();
}

bool UARTConnector::is_connected() const
{
  return serial_ && serial_->isOpen();
}

void UARTConnector::flush_buffers()
{
  if (is_connected()) {
    try {
      serial_->flush();
    } catch (...) {}
  }
}

bool UARTConnector::reset()
{
  const int max_reset_attempts = 10;
  const int base_delay_ms = 200;

  for (int attempt = 0; attempt < max_reset_attempts; attempt++) {
    disconnect();

    int delay_ms = base_delay_ms * (attempt + 1);
    if (attempt > 0) {
      delay_ms += 300 * attempt;
    }
    delay_ms = std::min(delay_ms, 3000);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    if (connect()) {
      return true;
    }

    if (attempt > 0 && (attempt % 5) == 0) {
      fprintf(stderr, "[BNO055] Still trying to reconnect (attempt %d)...\n", attempt + 1);
    }
  }

  return false;
}

int UARTConnector::read_response(std::vector<uint8_t> & data, size_t expected_length)
{
  // BNO055 read response  = [0xBB, len, data...]
  // BNO055 error response = [0xEE, error_code]
  // Returns: 0 = success, >0 = BNO055 error code, -1 = comm failure
  try {
    uint8_t byte = 0;
    const int max_header_attempts = 10;

    // Search for a valid response header byte
    for (int i = 0; i < max_header_attempts; i++) {
      size_t n = serial_->read(&byte, 1);
      if (n != 1) {
        return -1;  // Timeout
      }
      if (byte == COM_START_BYTE_RESP || byte == COM_START_BYTE_ERROR_RESP) {
        break;
      }
      if (i == max_header_attempts - 1) {
        return -1;
      }
    }

    if (byte == COM_START_BYTE_ERROR_RESP) {
      uint8_t err = 0;
      if (serial_->read(&err, 1) != 1) {
        return -1;
      }
      return static_cast<int>(err);
    }

    if (byte != COM_START_BYTE_RESP) {
      return -1;
    }

    // Read length byte
    uint8_t response_length = 0;
    if (serial_->read(&response_length, 1) != 1) {
      return -1;
    }

    if (response_length != expected_length) {
      // Drain remaining bytes
      if (response_length > 0) {
        std::vector<uint8_t> drain(response_length);
        serial_->read(drain.data(), response_length);
      }
      return -1;
    }

    // Read data payload
    data.resize(expected_length);
    size_t total_read = 0;
    while (total_read < expected_length) {
      size_t n = serial_->read(data.data() + total_read, expected_length - total_read);
      if (n == 0) {
        return -1;  // Timeout
      }
      total_read += n;
    }

    return 0;  // Success
  } catch (const std::exception &) {
    return -1;
  }
}

bool UARTConnector::read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length)
{
  if (!is_connected()) {
    return false;
  }

  const int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; attempt++) {
    if (!is_connected()) {
      return false;
    }

    try {
      if (attempt > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        serial_->flushInput();
      } else {
        serial_->flushInput();
      }

      // Build read command: [start_byte, read_cmd, reg_addr, length]
      uint8_t cmd[] = {COM_START_BYTE_WR, COM_READ, reg_addr, static_cast<uint8_t>(length)};
      size_t written = serial_->write(cmd, sizeof(cmd));
      if (written != sizeof(cmd)) {
        continue;
      }

      int result = read_response(data, length);
      if (result == 0) {
        return true;
      }
      // Transient errors worth retrying
      if (result != 0x07 && result != 0x0A) {
        return false;
      }
    } catch (const serial::PortNotOpenedException &) {
      return false;
    } catch (const serial::IOException &) {
      return false;
    }
  }
  return false;
}

bool UARTConnector::write(uint8_t reg_addr, const std::vector<uint8_t> & data)
{
  if (!is_connected()) {
    return false;
  }

  try {
    serial_->flushInput();

    // Build write command: [start_byte, write_cmd, reg_addr, length, data...]
    std::vector<uint8_t> cmd;
    cmd.push_back(COM_START_BYTE_WR);
    cmd.push_back(COM_WRITE);
    cmd.push_back(reg_addr);
    cmd.push_back(static_cast<uint8_t>(data.size()));
    cmd.insert(cmd.end(), data.begin(), data.end());

    size_t written = serial_->write(cmd);
    if (written != cmd.size()) {
      return false;
    }

    // BNO055 write response: [0xEE, status] — status 0x01 = success
    uint8_t response[2] = {0, 0};
    size_t n = serial_->read(response, 2);
    if (n != 2) {
      return false;
    }
    return (response[0] == COM_START_BYTE_ERROR_RESP && response[1] == 0x01);
  } catch (const serial::PortNotOpenedException &) {
    return false;
  } catch (const serial::IOException &) {
    return false;
  }
}

}  // namespace bno055
