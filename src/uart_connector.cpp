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
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
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

// ── Helper: select()-based timed read ──────────────────────────────────
// Returns bytes actually read, 0 on timeout, -1 on error.
static ssize_t timed_read(int fd, void * buf, size_t count, double timeout_sec)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  struct timeval tv;
  tv.tv_sec  = static_cast<long>(timeout_sec);
  tv.tv_usec = static_cast<long>((timeout_sec - tv.tv_sec) * 1e6);

  int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    return ret;  // 0 = timeout, -1 = error
  }
  return ::read(fd, buf, count);
}

// ── Helper: drain all pending bytes from fd ────────────────────────────
static void drain_fd(int fd)
{
  uint8_t junk[64];
  // Non-blocking drain: use a very short select timeout
  for (int i = 0; i < 10; i++) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    struct timeval tv = {0, 5000};  // 5ms
    if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
      break;
    }
    if (::read(fd, junk, sizeof(junk)) <= 0) {
      break;
    }
  }
}

// ── Helper: configure port with stty ───────────────────────────────────
// IMPORTANT: must be called AFTER open() while the fd is held open.
// On USB-serial (cp210x, ch340, etc.), when the last fd is closed the
// kernel frees tty_struct and the next open() starts with default settings
// (9600 baud).  Running stty while our fd keeps the tty alive ensures
// the settings actually stick.
static bool configure_port_stty(const std::string & port, int baudrate)
{
  char cmd[256];
  snprintf(cmd, sizeof(cmd),
    "stty -F %s %d raw -echo -echoe -echok -echoctl -echoke "
    "cs8 -cstopb -parenb -crtscts -ixon -ixoff -ixany "
    "cread clocal 2>&1",
    port.c_str(), baudrate);

  int ret = system(cmd);
  if (ret != 0) {
    fprintf(stderr, "[BNO055-DBG] stty failed (ret=%d) cmd: %s\n", ret, cmd);
    return false;
  }
  return true;
}

bool UARTConnector::connect()
{
  // Open the port FIRST — this keeps the tty alive so stty settings persist.
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    fprintf(stderr, "[BNO055-DBG] open(%s) failed: errno=%d (%s)\n",
            port_.c_str(), errno, strerror(errno));
    return false;
  }
  fprintf(stderr, "[BNO055-DBG] Opened %s fd=%d\n", port_.c_str(), fd_);

  // Configure port AFTER opening (fd keeps tty_struct alive, settings stick)
  if (!configure_port_stty(port_, baudrate_)) {
    fprintf(stderr, "[BNO055-DBG] Failed to configure port %s with stty\n", port_.c_str());
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Drain any stale data from previous session
  drain_fd(fd_);

  // ── First try: read chip ID WITHOUT resetting ──
  // If sensor is already running fine, a reset would break the connection.
  std::vector<uint8_t> chip_id;
  bool chip_id_ok = false;
  fprintf(stderr, "[BNO055-DBG] Starting chip ID verification on port=%s fd=%d\n",
          port_.c_str(), fd_);

  // Attempt 1-3: just read chip ID directly (no reset)
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      usleep(100000);  // 100ms between retries
      drain_fd(fd_);
    }
    chip_id.clear();
    bool read_ok = read(BNO055_CHIP_ID_ADDR, chip_id, 1);
    fprintf(stderr, "[BNO055-DBG] Chip ID attempt %d/6 (no reset): read_ok=%d, size=%zu",
            attempt + 1, read_ok, chip_id.size());
    if (!chip_id.empty()) {
      fprintf(stderr, ", chip_id[0]=0x%02X (expected 0x%02X)", chip_id[0], BNO055_ID);
    }
    fprintf(stderr, "\n");
    if (read_ok && chip_id.size() == 1 && chip_id[0] == BNO055_ID) {
      chip_id_ok = true;
      break;
    }
  }

  // ── Second try: if direct read failed, do hardware reset then retry ──
  if (!chip_id_ok) {
    fprintf(stderr, "[BNO055-DBG] Direct read failed, sending hardware reset...\n");
    drain_fd(fd_);

    // Set page 0 (BNO055_PAGE_ID_ADDR=0x07, value=0x00)
    uint8_t page_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_PAGE_ID_ADDR, 0x01, 0x00};
    ::write(fd_, page_cmd, sizeof(page_cmd));
    usleep(10000);  // 10ms
    drain_fd(fd_);

    // Send reset trigger (BNO055_SYS_TRIGGER_ADDR=0x3F, value=0x20)
    uint8_t reset_cmd[] = {COM_START_BYTE_WR, COM_WRITE, BNO055_SYS_TRIGGER_ADDR, 0x01, 0x20};
    ::write(fd_, reset_cmd, sizeof(reset_cmd));
    usleep(1000000);  // 1000ms — generous boot time after reset
    drain_fd(fd_);

    // Attempt 4-6: read chip ID after reset
    for (int attempt = 0; attempt < 3; attempt++) {
      if (attempt > 0) {
        usleep(200000);  // 200ms between retries
        drain_fd(fd_);
      }
      chip_id.clear();
      bool read_ok = read(BNO055_CHIP_ID_ADDR, chip_id, 1);
      fprintf(stderr, "[BNO055-DBG] Chip ID attempt %d/6 (after reset): read_ok=%d, size=%zu",
              attempt + 4, read_ok, chip_id.size());
      if (!chip_id.empty()) {
        fprintf(stderr, ", chip_id[0]=0x%02X (expected 0x%02X)", chip_id[0], BNO055_ID);
      }
      fprintf(stderr, "\n");
      if (read_ok && chip_id.size() == 1 && chip_id[0] == BNO055_ID) {
        chip_id_ok = true;
        break;
      }
    }
  }

  if (!chip_id_ok) {
    fprintf(stderr, "[BNO055-DBG] FAIL: All chip ID attempts failed, closing fd=%d\n", fd_);
    close(fd_);
    fd_ = -1;
    return false;
  }
  fprintf(stderr, "[BNO055-DBG] SUCCESS: Chip ID verified\n");

  return true;
}

void UARTConnector::disconnect()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void UARTConnector::flush_buffers()
{
  if (fd_ >= 0) {
    drain_fd(fd_);
  }
}

bool UARTConnector::reset()
{
  const int max_reset_attempts = 3;
  const int base_delay_ms = 200;

  for (int attempt = 0; attempt < max_reset_attempts; attempt++) {
    disconnect();

    int delay_ms = base_delay_ms * (attempt + 1);
    if (attempt > 0) {
      delay_ms += 300 * attempt;
    }
    usleep(delay_ms * 1000);

    if (connect()) {
      return true;
    }
  }

  return false;
}

int UARTConnector::read_response(std::vector<uint8_t> & data, size_t expected_length)
{
  // BNO055 read response  = [0xBB, len, data...]
  // BNO055 error response = [0xEE, error_code]
  // Returns: 0 = success, >0 = BNO055 error code, -1 = comm failure
  uint8_t byte;
  int max_attempts = 10;

  // Search for a valid response header byte (0xBB or 0xEE)
  for (int i = 0; i < max_attempts; i++) {
    ssize_t n = timed_read(fd_, &byte, 1, timeout_);
    if (n != 1) {
      fprintf(stderr, "[BNO055-DBG] read_response: header read failed "
              "(n=%zd, attempt %d/%d, errno=%d: %s)\n",
              n, i + 1, max_attempts, errno, strerror(errno));
      return -1;
    }
    fprintf(stderr, "[BNO055-DBG] read_response: header byte[%d]=0x%02X\n", i, byte);
    if (byte == COM_START_BYTE_RESP || byte == COM_START_BYTE_ERROR_RESP) {
      break;
    }
    if (i == max_attempts - 1) {
      fprintf(stderr, "[BNO055-DBG] read_response: no valid header after %d attempts\n",
              max_attempts);
      return -1;
    }
  }

  if (byte == COM_START_BYTE_ERROR_RESP) {
    uint8_t err = 0;
    timed_read(fd_, &err, 1, timeout_);
    fprintf(stderr, "[BNO055-DBG] read_response: BNO055 error code=0x%02X\n", err);
    return static_cast<int>(err);
  }

  if (byte != COM_START_BYTE_RESP) {
    return -1;
  }

  // Read length byte
  uint8_t response_length;
  if (timed_read(fd_, &response_length, 1, timeout_) != 1) {
    return -1;
  }

  if (response_length != expected_length) {
    // Drain remaining bytes
    std::vector<uint8_t> junk(response_length);
    timed_read(fd_, junk.data(), response_length, timeout_);
    return -1;
  }

  // Read data
  data.resize(expected_length);
  size_t total_read = 0;
  while (total_read < expected_length) {
    ssize_t n = timed_read(fd_, data.data() + total_read,
                           expected_length - total_read, timeout_);
    if (n <= 0) {
      return -1;
    }
    total_read += static_cast<size_t>(n);
  }

  return 0;  // Success
}

bool UARTConnector::read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length)
{
  if (fd_ < 0) {
    return false;
  }

  const int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; attempt++) {
    if (attempt > 0) {
      usleep(2000);  // 2ms
    }

    // Drain stale input before sending command
    drain_fd(fd_);

    // Build read command: [start_byte, read_cmd, reg_addr, length]
    std::vector<uint8_t> cmd = {
      COM_START_BYTE_WR, COM_READ, reg_addr, static_cast<uint8_t>(length)};

    if (::write(fd_, cmd.data(), cmd.size()) != static_cast<ssize_t>(cmd.size())) {
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
  }
  return false;
}

bool UARTConnector::write(uint8_t reg_addr, const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }

  // Drain stale input
  drain_fd(fd_);

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

  // BNO055 write response is always 2 bytes: [0xEE, status]
  // Status 0x01 = success, anything else = error
  uint8_t response[2];
  ssize_t n = timed_read(fd_, response, 2, timeout_);
  if (n != 2) {
    return false;
  }
  return (response[0] == COM_START_BYTE_ERROR_RESP && response[1] == 0x01);
}

}  // namespace bno055
