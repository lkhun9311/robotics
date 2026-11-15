// SPDX-License-Identifier: Apache-2.0
#include "socketcan_io/socket.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <utility>

namespace socketcan_io {

namespace {

std::string errno_text(const char* what) { return std::string(what) + ": " + std::strerror(errno); }

}  // namespace

SysOps system_ops() {
  SysOps ops;
  ops.recvmsg = [](int fd, msghdr* msg, int flags) { return ::recvmsg(fd, msg, flags); };
  ops.write = [](int fd, const void* buf, std::size_t n) { return ::write(fd, buf, n); };
  ops.now = [] {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return runtime_core::MonoTime{static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 +
                                  ts.tv_nsec};
  };
  return ops;
}

std::optional<CanSocket> CanSocket::open(std::string_view ifname, std::string* error) {
  auto fail = [&](const char* what) -> std::optional<CanSocket> {
    if (error != nullptr) {
      *error = errno_text(what);
    }
    return std::nullopt;
  };
  const std::string name(ifname);
  if (name.empty() || name.size() >= IFNAMSIZ) {
    if (error != nullptr) {
      *error = "invalid interface name '" + name + "'";
    }
    return std::nullopt;
  }
  const unsigned index = ::if_nametoindex(name.c_str());
  if (index == 0) {
    return fail(("if_nametoindex " + name).c_str());
  }
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    return fail("socket");
  }
  CanSocket sock(fd, system_ops());  // Closes fd on every failure below.
  const int on = 1;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof on) < 0) {
    return fail("CAN_RAW_FD_FRAMES");
  }
  const can_err_mask_t all_errors = CAN_ERR_MASK;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &all_errors, sizeof all_errors) < 0) {
    return fail("CAN_RAW_ERR_FILTER");
  }
  if (::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof on) < 0) {
    return fail("SO_TIMESTAMPNS");
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(index);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) < 0) {
    return fail("bind");
  }
  return sock;
}

CanSocket::CanSocket(int fd, SysOps ops) : fd_(fd), ops_(std::move(ops)) {}

CanSocket::CanSocket(CanSocket&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)),
      ops_(std::move(other.ops_)),
      counts_(other.counts_),
      last_rx_errno_(other.last_rx_errno_) {}

CanSocket& CanSocket::operator=(CanSocket&& other) noexcept {
  if (this != &other) {
    close_fd();
    fd_ = std::exchange(other.fd_, -1);
    ops_ = std::move(other.ops_);
    counts_ = other.counts_;
    last_rx_errno_ = other.last_rx_errno_;
  }
  return *this;
}

CanSocket::~CanSocket() { close_fd(); }

void CanSocket::close_fd() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = -1;
}

std::optional<RxFrame> CanSocket::read_one() {
  std::uint64_t rejected = 0;
  for (;;) {
    canfd_frame frame{};
    alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))> control{};
    iovec iov{&frame, sizeof frame};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data();
    msg.msg_controllen = control.size();

    const ssize_t n = ops_.recvmsg(fd_, &msg, 0);
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        ++counts_.rx_error;
        last_rx_errno_ = errno;
      }
      return std::nullopt;
    }
    // Stamp first: nothing between the syscall's return and this read may take time.
    const runtime_core::MonoTime rx_mono = ops_.now();
    ++counts_.rx;
    const auto nbytes = static_cast<std::size_t>(n);
    const std::optional<canopen_codec::Frame> codec = to_codec(frame, nbytes);
    if (!codec.has_value()) {
      ++counts_.rx_rejected[static_cast<std::size_t>(classify(frame, nbytes))];
      if (++rejected >= kMaxRejectedPerRead) {
        ++counts_.rx_reject_budget_hits;
        return std::nullopt;
      }
      continue;
    }
    RxFrame out;
    out.frame = *codec;
    out.rx_mono = rx_mono;
    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
      if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS &&
          c->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
        timespec ts{};
        std::memcpy(&ts, CMSG_DATA(c), sizeof ts);
        out.kernel_rt_ns = static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
      }
    }
    return out;
  }
}

bool CanSocket::send(const canopen_codec::Frame& frame) {
  if (frame.dlc > 8) {
    ++counts_.tx_other_error;
    return false;
  }
  can_frame out{};
  out.can_id = frame.cob_id;
  out.len = frame.dlc;
  std::memcpy(out.data, frame.data.data(), frame.dlc);
  const ssize_t n = ops_.write(fd_, &out, sizeof out);
  if (n == static_cast<ssize_t>(sizeof out)) {
    ++counts_.tx;
    return true;
  }
  if (n < 0 && errno == EAGAIN) {
    ++counts_.tx_eagain;
  } else if (n < 0 && errno == ENOBUFS) {
    ++counts_.tx_enobufs;
  } else {
    ++counts_.tx_other_error;
  }
  return false;
}

}  // namespace socketcan_io
