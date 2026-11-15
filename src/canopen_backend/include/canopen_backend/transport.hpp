// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

#include "canopen_codec/codec.hpp"

namespace canopen_backend {

// What a transport counted of its own I/O (spec §4 IPC policy: a failed send is counted, never
// retried). An in-memory transport counts nothing.
struct TransportCounts {
  std::uint64_t tx{};
  std::uint64_t tx_eagain{};
  std::uint64_t tx_enobufs{};
  std::uint64_t tx_other_error{};
  std::uint64_t rx{};           // Every frame received, accepted or not.
  std::uint64_t rx_rejected{};  // Received but not a classic data frame (B11).
  std::uint64_t rx_error{};     // Receive failures other than "nothing pending".
  int last_rx_errno{0};         // Of the latest receive failure; 0 if none.
};

// The path between the CANopen backend and the bus. The backend calls it from the one thread
// that steps it (in E3c the I/O worker; in logic-time tests, the test). An in-memory
// implementation connects to drive_core drives; SocketTransport to SocketCAN.
class Transport {
 public:
  virtual ~Transport() = default;
  // Puts one frame on the bus. Never blocks.
  virtual void send(const canopen_codec::Frame& frame) = 0;
  // Frames received since the last call, oldest first; empty when none. A transport may return
  // a bounded batch and the rest at the next call. Never blocks.
  virtual std::vector<canopen_codec::Frame> poll() = 0;
  // A descriptor that turns readable when poll() has frames, for the worker's ppoll; -1 for a
  // transport without one (the worker then polls at every timer wake).
  virtual int wait_fd() const { return -1; }
  virtual TransportCounts counts() const { return {}; }
  // Gives up the bus (closes the socket); send and poll do nothing afterwards.
  virtual void release() {}
};

}  // namespace canopen_backend
