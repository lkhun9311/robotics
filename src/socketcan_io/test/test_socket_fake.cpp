// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <linux/can.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>
#include <ctime>

#include "socketcan_io/socket.hpp"

namespace {

using socketcan_io::CanSocket;
using socketcan_io::SysOps;

canopen_codec::Frame sample() {
  canopen_codec::Frame f;
  f.cob_id = 0x203;
  f.dlc = 2;
  f.data[0] = 1;
  return f;
}

SysOps no_ops() {
  SysOps ops;
  ops.recvmsg = [](int, msghdr*, int) -> ssize_t {
    errno = EAGAIN;
    return -1;
  };
  ops.write = [](int, const void*, std::size_t) -> ssize_t {
    errno = EIO;
    return -1;
  };
  ops.now = [] { return runtime_core::MonoTime{0}; };
  return ops;
}

// Fills the caller's iovec with a classic frame; optionally attaches a kernel timestamp.
ssize_t deliver(msghdr* msg, canid_t id, bool with_stamp) {
  canfd_frame f;
  std::memset(&f, 0, sizeof f);
  f.can_id = id;
  f.len = 2;
  f.data[0] = 0xAB;
  std::memcpy(msg->msg_iov[0].iov_base, &f, CAN_MTU);
  if (!with_stamp) {
    msg->msg_controllen = 0;
  } else {
    timespec ts{3, 7};
    cmsghdr* c = CMSG_FIRSTHDR(msg);
    c->cmsg_level = SOL_SOCKET;
    c->cmsg_type = SCM_TIMESTAMPNS;
    c->cmsg_len = CMSG_LEN(sizeof ts);
    std::memcpy(CMSG_DATA(c), &ts, sizeof ts);
    msg->msg_controllen = CMSG_SPACE(sizeof ts);
  }
  return static_cast<ssize_t>(CAN_MTU);
}

}  // namespace

TEST(SocketCan, WriteErrorIsCounted) {
  SysOps ops = no_ops();
  int calls = 0;
  const int errors[] = {EAGAIN, ENOBUFS, EIO};
  ops.write = [&](int, const void*, std::size_t) -> ssize_t {
    errno = errors[calls++];
    return -1;
  };
  CanSocket sock(-1, ops);
  EXPECT_FALSE(sock.send(sample()));
  EXPECT_FALSE(sock.send(sample()));
  EXPECT_FALSE(sock.send(sample()));
  EXPECT_EQ(calls, 3);
  EXPECT_EQ(sock.counts().tx_eagain, 1U);
  EXPECT_EQ(sock.counts().tx_enobufs, 1U);
  EXPECT_EQ(sock.counts().tx_other_error, 1U);
  EXPECT_EQ(sock.counts().tx, 0U);
}

TEST(SocketCan, SendWritesClassicFrame) {
  SysOps ops = no_ops();
  can_frame seen{};
  std::size_t seen_len = 0;
  ops.write = [&](int, const void* p, std::size_t n) -> ssize_t {
    std::memcpy(&seen, p, sizeof seen);
    seen_len = n;
    return static_cast<ssize_t>(n);
  };
  CanSocket sock(-1, ops);
  EXPECT_TRUE(sock.send(sample()));
  EXPECT_EQ(seen_len, static_cast<std::size_t>(CAN_MTU));
  EXPECT_EQ(seen.can_id, 0x203U);
  EXPECT_EQ(seen.len, 2);
  EXPECT_EQ(seen.data[0], 1);
  EXPECT_EQ(sock.counts().tx, 1U);
}

TEST(SocketCan, ReadStampsMonotonicAfterRecv) {
  SysOps ops = no_ops();
  // The clock reads 1 ms until recvmsg has returned and 5 ms after, so a stamp taken before
  // the receive would show 1 ms.
  bool received = false;
  ops.now = [&received] { return runtime_core::MonoTime{received ? 5'000'000 : 1'000'000}; };
  ops.recvmsg = [&received](int, msghdr* m, int) {
    received = true;
    return deliver(m, 0x203, true);
  };
  CanSocket sock(-1, ops);
  const auto rx = sock.read_one();
  ASSERT_TRUE(rx.has_value());
  EXPECT_EQ(rx->rx_mono.ns, 5'000'000);
  ASSERT_TRUE(rx->kernel_rt_ns.has_value());
  EXPECT_EQ(*rx->kernel_rt_ns, 3'000'000'007);
  EXPECT_EQ(rx->frame.cob_id, 0x203);
  EXPECT_EQ(rx->frame.data[0], 0xAB);
  EXPECT_EQ(sock.counts().rx, 1U);
}

TEST(SocketCan, ReadWithoutKernelStampHasNone) {
  SysOps ops = no_ops();
  ops.recvmsg = [](int, msghdr* m, int) { return deliver(m, 0x203, false); };
  CanSocket sock(-1, ops);
  const auto rx = sock.read_one();
  ASSERT_TRUE(rx.has_value());
  EXPECT_FALSE(rx->kernel_rt_ns.has_value());
}

TEST(SocketCan, ReadReturnsNulloptOnEagain) {
  CanSocket sock(-1, no_ops());
  EXPECT_FALSE(sock.read_one().has_value());
  EXPECT_EQ(sock.counts().rx, 0U);
}

TEST(SocketCan, RejectedFrameIsCountedAndSkipped) {
  SysOps ops = no_ops();
  int n = 0;
  ops.recvmsg = [&](int, msghdr* m, int) -> ssize_t {
    if (n++ == 0) {
      return deliver(m, 0x203 | CAN_RTR_FLAG, false);
    }
    errno = EAGAIN;
    return -1;
  };
  CanSocket sock(-1, ops);
  EXPECT_FALSE(sock.read_one().has_value());
  EXPECT_EQ(sock.counts().rx, 1U);
  EXPECT_EQ(sock.counts().rx_rejected[static_cast<std::size_t>(socketcan_io::RxClass::kRtr)], 1U);
}

TEST(SocketCan, OpenFailsOnUnknownInterface) {
  std::string error;
  EXPECT_FALSE(CanSocket::open("nonexistent_can0", &error).has_value());
  EXPECT_FALSE(error.empty());
}
