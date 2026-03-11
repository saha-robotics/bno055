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
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
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
  // Open with non-blocking flag to prevent blocking on port availability
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (fd_ < 0) {
    return false;
  }

  // Clear O_NONBLOCK after opening to allow blocking reads with timeout
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags != -1) {
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  // ── Termios: start from cfmakeraw() baseline ──
  // cfmakeraw() zeroes all processing flags, giving a clean slate.
  // Previous code used tcgetattr + manual flag clearing, which could
  // inherit stale flags left by a previous process.
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);  // Clean baseline: no echo, no signals, no processing

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

  // 8N1 mode, no hardware flow control, enable receiver
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= CREAD | CLOCAL;

  // Timeout configuration
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_ * 10);

  if (tcsetattr(fd_, TCSAFLUSH, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  // ── Modem control: match pyserial behavior ──
  // pyserial asserts DTR+RTS HIGH on every open(). On many USB-serial
  // adapters (CP2104, CH340, FTDI), DTR is wired to the device's reset
  // or power-enable pin. Without this, the BNO055 may not be powered
  // or may stay in reset.
  int modem_bits = TIOCM_DTR | TIOCM_RTS;
  ioctl(fd_, TIOCMBIS, &modem_bits);  // Assert DTR+RTS HIGH

  // ── Clear any lingering BREAK condition from a previous opener ──
  // If a previous process crashed while holding BREAK (TX LOW), the
  // BNO055 UART RX line stays low and the sensor cannot receive commands.
  ioctl(fd_, TIOCCBRK, nullptr);

  // Flush any stale data from previous session or partial BNO055 responses
  tcflush(fd_, TCIOFLUSH);

  // Send a blind hardware reset to BNO055 via protocol-level commands.
  // Even if BNO055 is in a confused UART state, the reset trigger bit
  // may reach the sensor and bring it to a known state.
  {
    // Set page 0 first (BNO055_PAGE_ID_ADDR=0x07, value=0x00)
    uint8_t page_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_PAGE_ID_ADDR, 0x01, 0x00};
    ::write(fd_, page_cmd, sizeof(page_cmd));
    usleep(10000);  // 10ms
    // Drain any response (don't care if it fails)
    uint8_t drain[8];
    ::read(fd_, drain, sizeof(drain));

    // Send reset trigger (BNO055_SYS_TRIGGER_ADDR=0x3F, value=0x20)
    uint8_t reset_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_SYS_TRIGGER_ADDR, 0x01, 0x20};
    ::write(fd_, reset_cmd, sizeof(reset_cmd));
    // BNO055 needs ~650ms to boot after reset
    usleep(800000);  // 800ms
    // Flush everything after reset
    tcflush(fd_, TCIOFLUSH);
  }

  // Verify connection by reading chip ID with retries
  std::vector<uint8_t> chip_id;
  bool chip_id_ok = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (attempt > 0) {
      usleep(200000);  // 200ms between retries
      tcflush(fd_, TCIOFLUSH);
    }
    chip_id.clear();
    if (read(BNO055_CHIP_ID_ADDR, chip_id, 1) && chip_id.size() == 1 && chip_id[0] == BNO055_ID) {
      chip_id_ok = true;
      break;
    }
  }

  if (!chip_id_ok) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void UARTConnector::disconnect()
{
  if (fd_ >= 0) {
    // Flush kernel UART buffers so no partial command is left in the pipe
    tcflush(fd_, TCIOFLUSH);

    // Best-effort: send BNO055 hardware reset so the sensor's UART state
    // machine returns to a known idle state.
    {
      uint8_t page_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_PAGE_ID_ADDR, 0x01, 0x00};
      ::write(fd_, page_cmd, sizeof(page_cmd));
      usleep(5000);  // 5ms

      uint8_t reset_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_SYS_TRIGGER_ADDR, 0x01, 0x20};
      ::write(fd_, reset_cmd, sizeof(reset_cmd));
      usleep(5000);  // 5ms
    }

    // Restore default termios (best-effort)
    struct termios tty;
    if (tcgetattr(fd_, &tty) == 0) {
      cfmakeraw(&tty);
      tcsetattr(fd_, TCSANOW, &tty);
    }

    close(fd_);
    fd_ = -1;
  }
}

void UARTConnector::flush_buffers()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

bool UARTConnector::reset()
{
  // Multi-attempt reset with increasing delays.
  // When the cable between UART converter and BNO055 had intermittent contact,
  // the BNO055 UART state machine may be confused. We need:
  // 1) Close and reopen the serial port (clears kernel buffers)
  // 2) Send hardware reset to BNO055 (done inside connect())
  // 3) Retry multiple times with increasing delays
  const int max_reset_attempts = 3;
  const int base_delay_ms = 200;

  for (int attempt = 0; attempt < max_reset_attempts; attempt++) {
    disconnect();

    // Increasing delay: 200ms, 500ms, 1000ms
    int delay_ms = base_delay_ms * (attempt + 1);
    if (attempt > 0) {
      delay_ms += 300 * attempt;  // Extra time for BNO055 to settle
    }
    usleep(delay_ms * 1000);

    if (connect()) {
      return true;
    }

    if (attempt > 0 && (attempt % 10) == 0) {
      fprintf(stderr, "[UARTConnector::reset] Still trying to reconnect (attempt %d)...\n",
        attempt + 1);
    }
  }

  return false;
}

ssize_t UARTConnector::timed_read(void * buf, size_t count, int timeout_ms)
{
  // Use select() with explicit timeout to prevent indefinite blocking.
  // The kernel VTIME mechanism is unreliable on some USB-serial adapters —
  // select() gives a hard upper bound on how long we wait.
  if (fd_ < 0) {
    return -1;
  }

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd_, &rfds);

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int sel = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
  if (sel < 0) {
    // select() error — check if device was removed
    if (errno == EIO || errno == ENXIO || errno == EBADF || errno == ENODEV) {
      close(fd_);
      fd_ = -1;  // Mark as disconnected so watchdog can trigger reconnect
    }
    return -1;
  }
  if (sel == 0) {
    // Timeout — no data available
    return 0;
  }

  ssize_t n = ::read(fd_, buf, count);
  if (n < 0) {
    // Read error — check for device removal
    if (errno == EIO || errno == ENXIO || errno == EBADF || errno == ENODEV) {
      close(fd_);
      fd_ = -1;
    }
  }
  return n;
}

int UARTConnector::read_response(std::vector<uint8_t> & data, size_t expected_length)
{
  // Read all available bytes: BNO055 read response = [0xBB, len, data...]
  // BNO055 error response = [0xEE, error_code]
  // Returns: 0 = success, >0 = BNO055 error code, -1 = comm failure
  //
  // Use timed_read() with select() to guarantee we never block forever.
  // Per-byte timeout: 150ms — generous enough for 115200 baud but prevents
  // the executor thread from hanging when the sensor stops responding.
  constexpr int BYTE_TIMEOUT_MS = 150;

  uint8_t byte = 0;
  int max_attempts = 10;
  
  // Search for a valid response header byte (0xBB or 0xEE)
  for (int i = 0; i < max_attempts; i++) {
    ssize_t n = timed_read(&byte, 1, BYTE_TIMEOUT_MS);
    if (n <= 0) {
      return -1;  // Timeout or error
    }
    if (byte == COM_START_BYTE_RESP || byte == COM_START_BYTE_ERROR_RESP) {
      break;
    }
    if (i == max_attempts - 1) {
      return -1;
    }
  }

  if (byte == COM_START_BYTE_ERROR_RESP) {
    // Read the error code byte
    uint8_t err = 0;
    ssize_t n = timed_read(&err, 1, BYTE_TIMEOUT_MS);
    if (n <= 0) {
      return -1;  // Couldn't read error code
    }
    return static_cast<int>(err);  // Return BNO055 error code (0x07 = bus overrun, etc.)
  }

  if (byte != COM_START_BYTE_RESP) {
    return -1;
  }

  // Read length byte
  uint8_t response_length;
  if (timed_read(&response_length, 1, BYTE_TIMEOUT_MS) != 1) {
    return -1;
  }

  if (response_length != expected_length) {
    // Drain remaining bytes (best-effort, don't block long)
    std::vector<uint8_t> drain(response_length);
    timed_read(drain.data(), response_length, BYTE_TIMEOUT_MS * 2);
    return -1;
  }

  // Read data payload
  data.resize(expected_length);
  size_t total_read = 0;
  // Overall timeout for payload: generous but finite
  int remaining_timeout_ms = BYTE_TIMEOUT_MS * static_cast<int>(expected_length + 1);
  while (total_read < expected_length) {
    ssize_t n = timed_read(
      data.data() + total_read,
      expected_length - total_read,
      remaining_timeout_ms);
    if (n <= 0) {
      return -1;  // Timeout or error mid-payload
    }
    total_read += static_cast<size_t>(n);
    remaining_timeout_ms -= BYTE_TIMEOUT_MS;  // Decrease budget
    if (remaining_timeout_ms <= 0) {
      remaining_timeout_ms = BYTE_TIMEOUT_MS;  // Keep at least one timeout unit
    }
  }

  return 0;  // Success
}

bool UARTConnector::read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length)
{
  if (fd_ < 0) {
    return false;
  }

  // Retry loop: BNO055 can return 0x07 (BUS_OVER_RUN_ERROR) under high-frequency reads
  // or 0x0A (RECEIVE_CHARACTER_TIMEOUT) when fusion is not ready yet.
  const int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; attempt++) {
    // Check fd_ at each attempt — timed_read() may have invalidated it
    if (fd_ < 0) {
      return false;
    }

    if (attempt > 0) {
      // Wait before retry — give sensor time to recover from bus overrun.
      // Also wait long enough for BNO055 to finish sending any partial
      // response from the previous failed attempt (prevents UART desync).
      usleep(10000);  // 10ms (was 2ms — too short for 45-byte responses)
      if (fd_ >= 0) {
        tcflush(fd_, TCIOFLUSH);  // Flush BOTH directions to resync
      }
    } else {
      // First attempt: flush input only
      tcflush(fd_, TCIFLUSH);
    }

    // Build read command: [start_byte, read_cmd, reg_addr, length]
    uint8_t cmd[] = {COM_START_BYTE_WR, COM_READ, reg_addr, static_cast<uint8_t>(length)};
    
    ssize_t written = ::write(fd_, cmd, sizeof(cmd));
    if (written != static_cast<ssize_t>(sizeof(cmd))) {
      // Write failed — check if device was physically removed
      if (written < 0 && (errno == EIO || errno == ENXIO || errno == EBADF || errno == ENODEV)) {
        close(fd_);
        fd_ = -1;
        return false;
      }
      continue;
    }
    // Small delay to ensure command bytes are physically on the wire.
    // tcdrain() would be ideal (h4r uses serial::flushOutput) but it can
    // block indefinitely on some USB-serial adapters when the BNO055 UART
    // is in a broken state. At 115200 baud, 4 bytes take ~0.35ms.
    usleep(500);

    int result = read_response(data, length);
    if (result == 0) {
      return true;  // Success
    }
    // fd_ may have been invalidated inside read_response→timed_read
    if (fd_ < 0) {
      return false;
    }
    // Error 0x07 = BUS_OVER_RUN_ERROR — transient, worth retrying
    // Error 0x0A = RECEIVE_CHARACTER_TIMEOUT — also transient
    if (result != 0x07 && result != 0x0A) {
      return false;  // Non-transient error, don't retry
    }
  }
  return false;
}

bool UARTConnector::write(uint8_t reg_addr, const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }

  // Flush input buffer before sending command to discard stale data
  tcflush(fd_, TCIFLUSH);

  // Build write command: [start_byte, write_cmd, reg_addr, length, data...]
  std::vector<uint8_t> cmd;
  cmd.push_back(COM_START_BYTE_WR);
  cmd.push_back(COM_WRITE);
  cmd.push_back(reg_addr);
  cmd.push_back(static_cast<uint8_t>(data.size()));
  cmd.insert(cmd.end(), data.begin(), data.end());

  ssize_t written = ::write(fd_, cmd.data(), cmd.size());
  if (written != static_cast<ssize_t>(cmd.size())) {
    if (written < 0 && (errno == EIO || errno == ENXIO || errno == EBADF || errno == ENODEV)) {
      close(fd_);
      fd_ = -1;
    }
    return false;
  }
  // Small delay to ensure command bytes are physically on the wire.
  // tcdrain() can block indefinitely on some USB-serial adapters.
  usleep(500);

  // BNO055 write response is always 2 bytes: [0xEE, status]
  // Status 0x01 = success, anything else = error
  // Use timed_read to prevent blocking if sensor doesn't respond
  uint8_t response[2] = {0, 0};
  ssize_t n = timed_read(response, 2, 300);  // 300ms timeout for write ACK
  if (n != 2) {
    return false;
  }
  return (response[0] == COM_START_BYTE_ERROR_RESP && response[1] == 0x01);
}

}  // namespace bno055
