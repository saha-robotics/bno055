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

#ifndef BNO055__UART_CONNECTOR_HPP_
#define BNO055__UART_CONNECTOR_HPP_

#include "bno055/connector.hpp"
#include <string>

namespace bno055
{

class UARTConnector : public Connector
{
public:
  UARTConnector(const std::string & port, int baudrate, double timeout);
  ~UARTConnector() override;

  bool connect() override;
  void disconnect() override;
  bool read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t length) override;
  bool write(uint8_t reg_addr, const std::vector<uint8_t> & data) override;
  bool is_connected() const override { return fd_ >= 0; }

private:
  std::string port_;
  int baudrate_;
  double timeout_;
  int fd_;
  
  bool read_response(std::vector<uint8_t> & data, size_t expected_length);
};

}  // namespace bno055

#endif  // BNO055__UART_CONNECTOR_HPP_
