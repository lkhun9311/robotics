// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include "canopen_codec/codec.hpp"

namespace canopen_backend {

// The path between the CANopen backend and the bus. The backend calls both from the one thread
// that steps it (in E3c the I/O worker; in logic-time tests, the test). An in-memory
// implementation connects to drive_core drives; the socket one (E3c Task 6) to SocketCAN.
class Transport {
 public:
  virtual ~Transport() = default;
  // Puts one frame on the bus. Never blocks.
  virtual void send(const canopen_codec::Frame& frame) = 0;
  // Every frame received since the last call, oldest first; empty when none. Never blocks.
  virtual std::vector<canopen_codec::Frame> poll() = 0;
};

}  // namespace canopen_backend
