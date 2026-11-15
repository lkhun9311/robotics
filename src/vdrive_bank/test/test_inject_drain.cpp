// SPDX-License-Identifier: Apache-2.0
// The inject socket drain of the bank loop (B1, B6) on a real UNIX datagram socket pair: a
// client that keeps the socket full must not hold the control thread. The CAN socket is a fake
// that is never readable; the clock is the test's.
#include <gtest/gtest.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include "socketcan_io/socket.hpp"
#include "vdrive_bank/loop.hpp"
#include "vdrive_bank/trace_ring.hpp"

namespace {

using runtime_core::Duration;
using runtime_core::microseconds;
using runtime_core::milliseconds;
using runtime_core::MonoTime;
using vdrive_bank::Bank;
using vdrive_bank::BankLoop;
using vdrive_bank::BankRecord;
using vdrive_bank::TraceRecord;
using vdrive_bank::TraceRing;

constexpr MonoTime kT0{7'000'000'000};
constexpr std::array<std::uint32_t, vdrive_bank::kNodeCount> kBootIds{11, 22, 33, 44, 55, 66};

struct Rig {
  MonoTime now{kT0};
  Duration per_inject{0};  // Monotonic time one inject takes.
  // The CAN socket: never readable, or (can_flood) always, each read taking per_read.
  bool can_flood{false};
  Duration per_read{microseconds(100)};
  std::uint64_t can_reads{0};
  socketcan_io::CanSocket socket{-1, ops()};
  TraceRing ring;
  Bank bank{kBootIds, kT0};
  BankLoop loop{bank, socket, ring, [this] { return now; }, kT0};
  std::array<int, 2> fds{-1, -1};  // [0] the bank's end, [1] the client's.

  Rig() {
    loop.set_time_observer([this](BankLoop::Call c, MonoTime) {
      if (c == BankLoop::Call::kInject) {
        now = now + per_inject;
      }
    });
    loop.start(kBootIds);
    EXPECT_EQ(::socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, fds.data()), 0)
        << std::strerror(errno);
  }
  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;
  ~Rig() {
    for (const int fd : fds) {
      if (fd >= 0) {
        ::close(fd);
      }
    }
  }

  socketcan_io::SysOps ops() {
    socketcan_io::SysOps o;
    o.recvmsg = [this](int, msghdr* msg, int) -> ssize_t {
      if (!can_flood) {
        errno = EAGAIN;
        return -1;
      }
      // An SDO upload request of 0x1000 to node 1 (CiA 301: 0x601, ccs 2, index LE).
      canfd_frame raw;
      std::memset(&raw, 0, sizeof raw);
      raw.can_id = 0x601;
      raw.len = 8;
      raw.data[0] = 0x40;
      raw.data[2] = 0x10;
      std::memcpy(msg->msg_iov[0].iov_base, &raw, CAN_MTU);
      msg->msg_controllen = 0;
      ++can_reads;
      now = now + per_read;
      return static_cast<ssize_t>(CAN_MTU);
    };
    o.write = [](int, const void*, std::size_t n) -> ssize_t { return static_cast<ssize_t>(n); };
    o.now = [this] { return now; };
    return o;
  }

  // Queues `count` commands c0, c1, ... on the client end; all must fit in the socket.
  void fill(int count) {
    for (int i = 0; i < count; ++i) {
      const std::string command = "c" + std::to_string(i);
      ASSERT_EQ(::send(fds[1], command.data(), command.size(), MSG_DONTWAIT),
                static_cast<ssize_t>(command.size()))
          << "datagram " << i << ": " << std::strerror(errno);
    }
  }

  // The commands the bank applied, in order, from the inject records of the trace ring.
  std::vector<std::string> applied() {
    std::vector<std::string> out;
    while (auto r = ring.pop()) {
      const auto* b = std::get_if<BankRecord>(&*r);
      if (b == nullptr || b->kind != BankRecord::Kind::kInject) {
        continue;
      }
      const std::string text = b->text;  // "err unknown command: c<i>"
      out.push_back(text.substr(text.rfind(' ') + 1));
    }
    return out;
  }
};

TEST(InjectDrain, OneCallServesAtMostTheBudgetAndKeepsTheRestInOrder) {
  Rig rig;
  constexpr int kQueued = 100;
  static_assert(kQueued > 2 * vdrive_bank::kInjectBudget);
  rig.fill(kQueued);
  int total = 0;
  for (int call = 0; total < kQueued; ++call) {
    ASSERT_LT(call, kQueued) << "the drain makes no progress";
    const int served = rig.loop.serve_inject(rig.fds[0]);
    ASSERT_GE(served, 1);
    ASSERT_LE(served, vdrive_bank::kInjectBudget) << "call " << call;
    total += served;
  }
  EXPECT_EQ(total, kQueued);
  EXPECT_EQ(rig.loop.serve_inject(rig.fds[0]), 0);  // Drained.
  std::vector<std::string> expected;
  for (int i = 0; i < kQueued; ++i) {
    expected.push_back("c" + std::to_string(i));
  }
  EXPECT_EQ(rig.applied(), expected);  // Every datagram, once, in the order sent.
  EXPECT_EQ(rig.loop.inject_counts().recv_errors, 0U);
}

TEST(InjectDrain, OneCallStopsAtTheReleaseDeadline) {
  Rig rig;
  // 600 us per inject against W = 2 ms: the fourth inject reaches the release deadline and is
  // the last one of the call, as the first frame that reaches it ends the CAN drain.
  rig.per_inject = microseconds(600);
  rig.fill(vdrive_bank::kInjectBudget);
  const int served = rig.loop.serve_inject(rig.fds[0]);
  EXPECT_EQ(served, 4);
  EXPECT_GE(rig.now, rig.loop.next_release());
  EXPECT_LT(rig.now, rig.loop.next_release() + rig.per_inject);
}

TEST(InjectDrain, ALateCallStillServesOne) {
  Rig rig;
  // A wake that starts after the release deadline (a long dispatch, an overload): one datagram,
  // so a backlog is never held off for as long as the loop stays late.
  rig.now = rig.loop.next_release() + microseconds(500);
  rig.fill(3);
  EXPECT_EQ(rig.loop.serve_inject(rig.fds[0]), 1);
  EXPECT_EQ(rig.applied(), (std::vector<std::string>{"c0"}));
}

TEST(InjectDrain, SustainedInjectAndCanTrafficKeepTheReleases) {
  Rig rig;
  // The CAN drain runs to every release deadline and the inject socket is refilled before every
  // wake: each release still runs on time, each wake serves its inject budget (the drain sits
  // between the release and the CAN drain), and every command is applied once, in order.
  rig.can_flood = true;
  rig.per_inject = microseconds(10);
  std::vector<std::string> expected;
  int sent = 0;
  for (int wake = 0; rig.now <= kT0 + milliseconds(100); ++wake) {
    ASSERT_LT(wake, 1000) << "the loop does not advance";
    for (int i = 0; i < vdrive_bank::kInjectBudget; ++i, ++sent) {
      const std::string command = "c" + std::to_string(sent);
      ASSERT_EQ(::send(rig.fds[1], command.data(), command.size(), MSG_DONTWAIT),
                static_cast<ssize_t>(command.size()))
          << "wake " << wake << ": " << std::strerror(errno)
          << " (the backlog grew until the socket was full)";
      expected.push_back(command);
    }
    rig.loop.run_once(rig.fds[0]);
  }
  const vdrive_bank::LoopCounts& c = rig.loop.counts();
  EXPECT_EQ(c.releases_executed, 50U);
  EXPECT_EQ(c.releases_skipped, 0U);
  EXPECT_GT(rig.can_reads, 500U);
  EXPECT_EQ(rig.applied(), expected);
}

}  // namespace
