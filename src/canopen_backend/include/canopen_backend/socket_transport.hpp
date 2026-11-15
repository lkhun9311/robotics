// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "canopen_backend/transport.hpp"
#include "canopen_codec/codec.hpp"
#include "socketcan_io/socket.hpp"

namespace canopen_backend {

// The backend's transport on a SocketCAN raw socket (socketcan_io::CanSocket, non-blocking).
// A send that fails (ENOBUFS, EAGAIN, other) is counted by the socket and not retried: a late
// CANopen frame is worse than a lost one. Rejected frames (RTR, extended, error, FD) are counted
// and never reach the backend (B11). Single-threaded: only the I/O worker calls it.
class SocketTransport final : public Transport {
 public:
  // Frames one poll() reads at most; the worker's per-wake budget is a multiple of it.
  static constexpr std::size_t kPollBatch = 16;

  explicit SocketTransport(socketcan_io::CanSocket socket);

  void send(const canopen_codec::Frame& frame) override;
  std::vector<canopen_codec::Frame> poll() override;
  int wait_fd() const override;
  TransportCounts counts() const override;
  void release() override;

 private:
  std::optional<socketcan_io::CanSocket> socket_;
  TransportCounts released_{};  // The counts at release, kept for the exit record.
};

}  // namespace canopen_backend
