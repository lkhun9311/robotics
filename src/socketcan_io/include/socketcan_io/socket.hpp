// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <sys/socket.h>
#include <sys/types.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "canopen_codec/codec.hpp"
#include "runtime_core/time.hpp"
#include "socketcan_io/classify.hpp"

namespace socketcan_io {

// A frame that passed classification.
// `rx_mono` is CLOCK_MONOTONIC read right after recvmsg returned and is the only time a Drive may
// be given; `kernel_rt_ns` (CLOCK_REALTIME, from SO_TIMESTAMPNS) is a diagnostic and is never mixed
// with it.
struct RxFrame {
  canopen_codec::Frame frame;
  runtime_core::MonoTime rx_mono;
  std::optional<std::int64_t> kernel_rt_ns;
};

// The three system calls the socket makes, replaceable in tests. errno carries the failure.
struct SysOps {
  std::function<ssize_t(int, msghdr*, int)> recvmsg;
  std::function<ssize_t(int, const void*, std::size_t)> write;
  std::function<runtime_core::MonoTime()> now;
};

// Real system calls and CLOCK_MONOTONIC.
SysOps system_ops();

struct IoCounts {
  std::uint64_t rx{};                                      // Every frame received, accepted or not.
  std::array<std::uint64_t, kRxClassCount> rx_rejected{};  // Indexed by RxClass; kData stays 0.
  std::uint64_t rx_error{};               // recvmsg failures other than EAGAIN/EWOULDBLOCK/EINTR.
  std::uint64_t rx_reject_budget_hits{};  // read_one calls cut short by kMaxRejectedPerRead.
  std::uint64_t tx{};
  std::uint64_t tx_eagain{};
  std::uint64_t tx_enobufs{};
  std::uint64_t tx_other_error{};
};

// One non-blocking raw CAN socket.
// Receives classic and FD frames and error frames so that they are counted rather than silently
// filtered by the kernel (B11).
class CanSocket {
 public:
  static std::optional<CanSocket> open(std::string_view ifname, std::string* error);

  CanSocket(int fd, SysOps ops);  // Test seam; takes ownership of fd (a negative fd is not closed).
  CanSocket(CanSocket&& other) noexcept;
  CanSocket& operator=(CanSocket&& other) noexcept;
  CanSocket(const CanSocket&) = delete;
  CanSocket& operator=(const CanSocket&) = delete;
  ~CanSocket();

  // Rejected frames one read_one call skips before it gives the caller control back, so an
  // error-frame storm cannot starve a loop that drains with a per-wake budget.
  static constexpr std::uint64_t kMaxRejectedPerRead = 64;

  // The next frame the codec may see; rejected frames are counted and skipped.
  // std::nullopt means one of three things, told apart by the counters: nothing pending
  // (EAGAIN: no counter moves), a receive error (counts().rx_error rises and last_rx_errno()
  // holds the errno), or the reject budget was spent (counts().rx_reject_budget_hits rises;
  // frames may still be pending, so the caller reads again on its next turn).
  std::optional<RxFrame> read_one();
  bool send(const canopen_codec::Frame& frame);

  const IoCounts& counts() const { return counts_; }
  int fd() const { return fd_; }
  int last_rx_errno() const { return last_rx_errno_; }  // Of the latest rx_error; 0 if none.

 private:
  void close_fd();

  int fd_;
  SysOps ops_;
  IoCounts counts_{};
  int last_rx_errno_{0};
};

}  // namespace socketcan_io
