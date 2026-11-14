// SPDX-License-Identifier: Apache-2.0
// The bank loop (B1) against a fake socket that is always readable and a fake clock that the
// socket advances on every read: a dispatch that takes `per_read` of monotonic time.
#include <gtest/gtest.h>
#include <linux/can.h>
#include <sys/socket.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <vector>

#include "drive_core/cia402.hpp"
#include "socketcan_io/socket.hpp"
#include "vdrive_bank/loop.hpp"
#include "vdrive_bank/trace_ring.hpp"

namespace {

namespace cc = canopen_codec;
using runtime_core::Duration;
using runtime_core::microseconds;
using runtime_core::milliseconds;
using runtime_core::MonoTime;
using vdrive_bank::Bank;
using vdrive_bank::BankLoop;
using vdrive_bank::TraceRecord;
using vdrive_bank::TraceRing;

constexpr MonoTime kT0{7'000'000'000};
constexpr std::array<std::uint32_t, vdrive_bank::kNodeCount> kBootIds{11, 22, 33, 44, 55, 66};
// Objects every node serves by SDO upload in Pre-operational.
constexpr std::array<std::uint16_t, 3> kIndices{0x1000, 0x1001, 0x1017};

// SDO upload request (CiA 301: ccs 2, index LE, sub-index) for node `node`.
cc::Frame upload(std::uint8_t node, std::uint16_t index) {
  cc::Frame f{static_cast<std::uint16_t>(0x600U + node), 8, {}};
  f.data[0] = 0x40;
  f.data[1] = static_cast<std::uint8_t>(index & 0xFFU);
  f.data[2] = static_cast<std::uint8_t>(index >> 8U);
  return f;
}

// The clock, the frames the socket delivers (round robin over nodes and indices) and the frames
// written.
struct Sim {
  MonoTime now{kT0};
  Duration per_read{microseconds(100)};
  std::uint64_t reads{0};
  std::map<std::uint8_t, std::vector<std::uint16_t>> requested;  // Per node, in send order.
  std::vector<cc::Frame> written;

  cc::Frame next_input() {
    const auto node = static_cast<std::uint8_t>(reads % 6U + 1U);
    const std::uint16_t index = kIndices[(reads / 6U) % kIndices.size()];
    requested[node].push_back(index);
    ++reads;
    return upload(node, index);
  }

  socketcan_io::SysOps ops() {
    socketcan_io::SysOps o;
    o.recvmsg = [this](int, msghdr* msg, int) -> ssize_t {
      const cc::Frame f = next_input();
      canfd_frame raw;
      std::memset(&raw, 0, sizeof raw);
      raw.can_id = f.cob_id;
      raw.len = f.dlc;
      std::memcpy(raw.data, f.data.data(), f.dlc);
      std::memcpy(msg->msg_iov[0].iov_base, &raw, CAN_MTU);
      msg->msg_controllen = 0;
      now = now + per_read;
      return static_cast<ssize_t>(CAN_MTU);
    };
    o.write = [this](int, const void* buf, std::size_t n) -> ssize_t {
      can_frame out;
      std::memcpy(&out, buf, sizeof out);
      cc::Frame f{static_cast<std::uint16_t>(out.can_id), out.len, {}};
      std::memcpy(f.data.data(), out.data, out.len);
      written.push_back(f);
      return static_cast<ssize_t>(n);
    };
    o.now = [this] { return now; };
    return o;
  }
};

struct Rig {
  Sim sim;
  socketcan_io::CanSocket socket{-1, sim.ops()};
  TraceRing ring;
  Bank bank{kBootIds, kT0};
  BankLoop loop{bank, socket, ring, [this] { return sim.now; }, kT0};
  std::vector<std::pair<BankLoop::Call, MonoTime>> calls;

  Rig() {
    loop.set_time_observer([this](BankLoop::Call c, MonoTime t) { calls.emplace_back(c, t); });
    loop.start(kBootIds);
  }

  std::vector<TraceRecord> drain_ring() {
    std::vector<TraceRecord> out;
    while (auto r = ring.pop()) {
      out.push_back(*r);
    }
    return out;
  }

  std::size_t ticks() const {
    return static_cast<std::size_t>(std::count_if(calls.begin(), calls.end(), [](const auto& c) {
      return c.first == BankLoop::Call::kTick;
    }));
  }
};

TEST(BankLoop, TicksContinueUnderContinuousTraffic) {
  Rig rig;
  for (int wakes = 0; rig.sim.now <= kT0 + milliseconds(100); ++wakes) {
    ASSERT_LT(wakes, 1000) << "the loop does not advance";
    rig.loop.run_once();
    // The drain stops at the release deadline: the first read that reaches it is the last.
    ASSERT_GE(rig.sim.now, rig.loop.next_release());
    ASSERT_LT(rig.sim.now, rig.loop.next_release() + rig.sim.per_read);
  }
  const vdrive_bank::LoopCounts& c = rig.loop.counts();
  EXPECT_EQ(c.releases_executed, 50U);
  EXPECT_EQ(c.releases_planned, 50U);
  EXPECT_EQ(c.releases_skipped, 0U);
  EXPECT_EQ(c.rx_budget_hits, 0U);
  EXPECT_EQ(rig.ticks(), 50U);
  EXPECT_EQ(c.frames_dispatched, rig.sim.reads);
  EXPECT_GT(rig.sim.reads, 900U);  // ~20 frames per 2 ms release.

  // Per-node order: each node's SDO responses name the indices in the order they were requested.
  std::map<std::uint8_t, std::vector<std::uint16_t>> answered;
  for (const cc::Frame& f : rig.sim.written) {
    if (f.cob_id > 0x580 && f.cob_id <= 0x586) {
      answered[static_cast<std::uint8_t>(f.cob_id - 0x580)].push_back(
          static_cast<std::uint16_t>(f.data[1] | (f.data[2] << 8U)));
    }
  }
  for (std::uint8_t node = 1; node <= 6; ++node) {
    EXPECT_EQ(answered[node], rig.sim.requested[node]) << "node " << int{node};
  }
}

TEST(BankLoop, SlowDispatchSkipsAndThreeSkipsOverload) {
  Rig rig;
  // A dispatch of 2.5 ms against W = 2 ms: the loop falls behind and skips releases. Skipped
  // releases are never caught up: one tick per executed release, and the drive model advances by
  // exactly one W per tick.
  rig.sim.per_read = microseconds(2500);
  for (int i = 0; i < 40; ++i) {
    rig.loop.run_once();
  }
  const vdrive_bank::LoopCounts& c = rig.loop.counts();
  EXPECT_GT(c.releases_skipped, 5U);
  EXPECT_EQ(c.releases_executed + c.releases_skipped, c.releases_planned);
  EXPECT_EQ(rig.ticks(), c.releases_executed);
  EXPECT_LT(c.releases_executed, c.releases_planned);
  // One skip per late wake here: below the overload threshold.
  EXPECT_EQ(c.max_consecutive_skipped, 1U);
  EXPECT_EQ(c.overloads, 0U);
  for (std::uint8_t node = 1; node <= 6; ++node) {
    EXPECT_NE(rig.bank.drive(node).cia402().state(), drive_core::Cia402State::kFault);
  }

  // One dispatch of 8.5 ms: at least three releases pass without a tick -> overload.
  rig.sim.per_read = microseconds(8500);
  rig.loop.run_once();  // Tick, then one 8.5 ms read.
  rig.sim.per_read = microseconds(100);
  const std::size_t written_before = rig.sim.written.size();
  rig.loop.run_once();  // The late release: three or more skipped.
  EXPECT_GE(c.consecutive_skipped, vdrive_bank::kOverloadSkips);
  EXPECT_EQ(c.overloads, 1U);
  for (std::uint8_t node = 1; node <= 6; ++node) {
    const drive_core::Cia402State s = rig.bank.drive(node).cia402().state();
    EXPECT_TRUE(s == drive_core::Cia402State::kFaultReactionActive ||
                s == drive_core::Cia402State::kFault)
        << "node " << int{node};
  }
  // Each node's EMCY 0x6100 (Pre-operational allows EMCY).
  std::vector<std::uint16_t> emcy;
  for (std::size_t i = written_before; i < rig.sim.written.size(); ++i) {
    const cc::Frame& f = rig.sim.written[i];
    if (f.cob_id > 0x80 && f.cob_id <= 0x86 && (f.data[0] | (f.data[1] << 8U)) == 0x6100) {
      emcy.push_back(f.cob_id);
    }
  }
  EXPECT_EQ(emcy, (std::vector<std::uint16_t>{0x81, 0x82, 0x83, 0x84, 0x85, 0x86}));

  const std::vector<TraceRecord> records = rig.drain_ring();
  const auto overloads = std::count_if(records.begin(), records.end(), [](const TraceRecord& r) {
    const auto* o = std::get_if<vdrive_bank::trace::Overload>(&r);
    return o != nullptr && o->error_code == 0x6100 && o->consecutive_skipped >= 3;
  });
  EXPECT_EQ(overloads, 1);
}

TEST(BankLoop, InjectSocketFailuresAreCountedAndTraced) {
  Rig rig;
  rig.drain_ring();
  using Kind = vdrive_bank::trace::InjectIo::Kind;
  rig.loop.note_inject_io(Kind::kTruncated, 300);
  rig.loop.note_inject_io(Kind::kRecvError, 104);
  rig.loop.note_inject_io(Kind::kSendError, 111);
  rig.loop.note_inject_io(Kind::kTruncated, 129);
  const vdrive_bank::trace::InjectCounts& c = rig.loop.inject_counts();
  EXPECT_EQ(c.truncated, 2U);
  EXPECT_EQ(c.recv_errors, 1U);
  EXPECT_EQ(c.send_errors, 1U);
  std::vector<vdrive_bank::trace::InjectIo> traced;
  for (const TraceRecord& r : rig.drain_ring()) {
    if (const auto* io = std::get_if<vdrive_bank::trace::InjectIo>(&r)) {
      traced.push_back(*io);
    }
  }
  ASSERT_EQ(traced.size(), 4U);
  EXPECT_EQ(traced[0].value, 300);
  EXPECT_EQ(traced[3].totals.truncated, 2U);
  const auto exit = std::get<vdrive_bank::trace::Exit>(rig.loop.exit_record(rig.sim.now, 15));
  EXPECT_EQ(exit.inject.truncated, 2U);
  EXPECT_EQ(exit.inject.send_errors, 1U);
}

TEST(BankLoop, TimeNeverDecreases) {
  Rig rig;
  // Irregular dispatch times including zero (the budget path) and long stalls.
  std::uint64_t lcg = 12345;
  for (int i = 0; i < 2000; ++i) {
    lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
    rig.sim.per_read = Duration{static_cast<std::int64_t>((lcg >> 33U) % 3'000'001U)};
    if (i % 50 == 0) {
      EXPECT_TRUE(rig.loop.inject(i % 100 == 0 ? "stuck 3 1" : "bogus").ok == (i % 100 == 0));
    }
    rig.loop.run_once();
  }
  ASSERT_GT(rig.calls.size(), 2000U);
  EXPECT_EQ(rig.calls.front().first, BankLoop::Call::kPowerOn);
  for (std::size_t i = 1; i < rig.calls.size(); ++i) {
    ASSERT_GE(rig.calls[i].second, rig.calls[i - 1].second) << "call " << i;
  }
  const vdrive_bank::LoopCounts& c = rig.loop.counts();
  EXPECT_EQ(rig.calls.size(), 1U + c.releases_executed + c.frames_dispatched + 40U);
  EXPECT_GT(c.rx_budget_hits, 0U);
}

}  // namespace
