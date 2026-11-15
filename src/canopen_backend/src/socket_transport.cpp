// SPDX-License-Identifier: Apache-2.0
#include "canopen_backend/socket_transport.hpp"

#include <numeric>
#include <utility>

namespace canopen_backend {

namespace {

TransportCounts convert(const socketcan_io::CanSocket& s) {
  const socketcan_io::IoCounts& io = s.counts();
  TransportCounts c;
  c.tx = io.tx;
  c.tx_eagain = io.tx_eagain;
  c.tx_enobufs = io.tx_enobufs;
  c.tx_other_error = io.tx_other_error;
  c.rx = io.rx;
  c.rx_rejected = std::accumulate(io.rx_rejected.begin(), io.rx_rejected.end(), std::uint64_t{0});
  c.rx_error = io.rx_error;
  c.last_rx_errno = s.last_rx_errno();
  return c;
}

}  // namespace

SocketTransport::SocketTransport(socketcan_io::CanSocket socket) : socket_(std::move(socket)) {}

void SocketTransport::send(const canopen_codec::Frame& frame) {
  if (socket_) {
    socket_->send(frame);  // Counted in IoCounts on failure; never retried.
  }
}

std::vector<canopen_codec::Frame> SocketTransport::poll() {
  std::vector<canopen_codec::Frame> out;
  if (!socket_) {
    return out;
  }
  while (out.size() < kPollBatch) {
    std::optional<socketcan_io::RxFrame> rx = socket_->read_one();
    if (!rx) {
      break;  // Drained, a receive error or the reject budget: the counters tell which.
    }
    out.push_back(rx->frame);
  }
  return out;
}

int SocketTransport::wait_fd() const { return socket_ ? socket_->fd() : -1; }

TransportCounts SocketTransport::counts() const { return socket_ ? convert(*socket_) : released_; }

void SocketTransport::release() {
  if (socket_) {
    released_ = convert(*socket_);
    socket_.reset();  // Closes the socket.
  }
}

}  // namespace canopen_backend
