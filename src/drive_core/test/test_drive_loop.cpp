// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/drive.hpp"
#include "drive_core/validity.hpp"
#include "runtime_core/time.hpp"

// The drive loop (contract D7-D13): SYNC timeline, guards, quick stop, policy latch, faults,
// joint model and TPDO feedback. Logic time only: a FakeClock moves in W steps, the W tick runs
// before the frames of the same instant, and SYNC comes every T_bus.
namespace {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;
using drive_core::Cia402State;
using drive_core::Drive;
using drive_core::DriveEvent;
using drive_core::DriveEventKind;
using drive_core::DropReason;
using drive_core::QuickStopReason;
using drive_core::to_wire_us;
using runtime_core::Duration;
using runtime_core::FakeClock;
using runtime_core::microseconds;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

const cc::NodeId kNode = cc::NodeId::make(3).value();
const cc::NodeId kOther = cc::NodeId::make(4).value();
constexpr std::uint32_t kBootId = 0x1234'5678;
constexpr MonoTime kBoot{2'000'000'000};
constexpr Duration kW = drive_core::kTickPeriod;
constexpr std::int32_t kFar = 100'000'000;  // µrad: a target the joint cannot reach soon.
constexpr std::int64_t kVMax = drive_core::kVMaxNradS;
constexpr std::int64_t kDvPerTick = 20'000'000;       // a_stop * W, nrad/s
constexpr std::int64_t kEAllowNrad = 100'000'000;     // e_allow 0.1 rad
constexpr std::int64_t kBrakeBoundNrad = 50'000'000;  // v_max^2 / (2 a_stop)

// A drive after power-on, past the boot quarantine, NMT Operational, with an event log.
struct Loop {
  FakeClock clock{kBoot};
  Drive drive{kNode, kBootId, kBoot};
  std::vector<DriveEvent> events;
  std::uint16_t tag{0};
  std::uint16_t seq{0};

  Loop() {
    drive.set_event_sink([this](const DriveEvent& e) { events.push_back(e); });
    drive.power_on();
    clock.advance(milliseconds(100));
    frame(cc::encode_nmt({cc::NmtCommand::kStart, kNode}));
  }

  MonoTime now() const { return clock.now(); }
  std::vector<cc::Frame> frame(const cc::Frame& f) { return drive.on_frame(f, now()); }
  void cw(std::uint16_t value) { frame(cc::pack_rpdo3(kNode, od::Rpdo3{value})); }
  void nmt(cc::NmtCommand command) { frame(cc::encode_nmt({command, kNode})); }
  std::uint16_t commit() {
    const auto t = drive.commit(1, 1, now());
    EXPECT_TRUE(t.has_value());
    tag = t.value_or(0);
    return tag;
  }
  void enable() {
    cw(0x06);
    cw(0x07);
    commit();
    cw(0x0F);
    EXPECT_EQ(drive.cia402().state(), Cia402State::kOperationEnabled);
  }
  void rpdo1(std::uint16_t s, std::int32_t target) {
    frame(cc::pack_rpdo1(kNode, od::Rpdo1{target, s, tag}));
  }
  void rpdo2(std::uint16_t s, std::uint32_t until) {
    frame(cc::pack_rpdo2(kNode, od::Rpdo2{until, s, tag}));
  }
  // Both halves of the next command, valid for `validity` from now.
  void command(std::int32_t target, Duration validity = milliseconds(48)) {
    ++seq;
    rpdo1(seq, target);
    rpdo2(seq, to_wire_us(now() + validity));
  }
  std::vector<cc::Frame> sync() { return frame(cc::encode_sync()); }
  void tick() {
    clock.advance(kW);
    drive.tick(now());
  }
  void ticks(int n) {
    for (int i = 0; i < n; ++i) {
      tick();
    }
  }
  // One bus period: five W ticks, then the SYNC, then (optionally) the next command.
  std::vector<cc::Frame> period(bool with_command = true, std::int32_t target = kFar) {
    ticks(5);
    auto frames = sync();
    if (with_command) {
      command(target);
    }
    return frames;
  }
  // Enabled, joint at v_max towards `target` (one command actuated per SYNC).
  void run_at_vmax(std::int32_t target = kFar) {
    enable();
    sync();
    command(target);
    for (int i = 0; i < 3; ++i) {
      period(true, target);
    }
    EXPECT_EQ(std::llabs(drive.joint().velocity_nrad_s), kVMax);
  }

  std::size_t count(DriveEventKind kind) const {
    return static_cast<std::size_t>(std::count_if(
        events.begin(), events.end(), [kind](const DriveEvent& e) { return e.kind == kind; }));
  }
  std::optional<DriveEvent> last(DriveEventKind kind) const {
    for (auto it = events.rbegin(); it != events.rend(); ++it) {
      if (it->kind == kind) {
        return *it;
      }
    }
    return std::nullopt;
  }
  std::uint32_t dropped(DropReason reason) const {
    std::uint32_t n = 0;
    for (const auto& e : events) {
      if (e.kind == DriveEventKind::kDropped && e.drop == reason) {
        n += e.count;
      }
    }
    return n;
  }
  Cia402State state() const { return drive.cia402().state(); }
  std::optional<QuickStopReason> stop_reason() const {
    const auto e = last(DriveEventKind::kQuickStop);
    return e ? std::optional<QuickStopReason>{e->reason} : std::nullopt;
  }
};

struct Feedback {
  od::Tpdo1 t1;
  od::Tpdo2 t2;
  od::Tpdo3 t3;
};

Feedback unpack(const std::vector<cc::Frame>& frames) {
  EXPECT_EQ(frames.size(), 3U);
  Feedback fb{};
  for (const auto& f : frames) {
    if (const auto t1 = cc::unpack_tpdo1(kNode, f)) {
      fb.t1 = *t1;
    } else if (const auto t2 = cc::unpack_tpdo2(kNode, f)) {
      fb.t2 = *t2;
    } else if (const auto t3 = cc::unpack_tpdo3(kNode, f)) {
      fb.t3 = *t3;
    } else {
      ADD_FAILURE() << "unexpected frame 0x" << std::hex << f.cob_id;
    }
  }
  return fb;
}

TEST(Sync, RpdoActuatesAtNextSync) {
  Loop l;
  l.enable();
  l.sync();  // SYNC k
  l.command(1'000);
  l.ticks(5);
  EXPECT_FALSE(l.drive.actuated().has_value());  // Received, not yet actuated.
  EXPECT_EQ(l.drive.joint().position_nrad, 0);
  l.sync();  // SYNC k+1
  ASSERT_TRUE(l.drive.actuated().has_value());
  EXPECT_EQ(l.drive.actuated()->command_seq, l.seq);
  EXPECT_EQ(l.drive.actuated()->target_position, 1'000);
  EXPECT_EQ(l.count(DriveEventKind::kActuated), 1U);
  l.tick();
  EXPECT_EQ(l.drive.joint().position_nrad, 1'000'000);  // 1 mrad, reached within one W.
}

TEST(Sync, TpdoSampledAtSync) {
  Loop l;
  // Pre-operational: SYNC is consumed, but no PDO is sent.
  l.nmt(cc::NmtCommand::kEnterPreOperational);
  l.ticks(5);
  EXPECT_TRUE(l.sync().empty());
  l.nmt(cc::NmtCommand::kStart);

  l.enable();
  l.ticks(5);
  l.sync();
  l.command(kFar);
  l.ticks(5);
  l.sync();  // Actuated here.
  l.ticks(5);
  const MonoTime at = l.now();
  const auto fb = unpack(l.sync());
  // Five W at v_max since the actuation: 10 mrad.
  EXPECT_EQ(fb.t1.position_actual_value, 10'000);
  EXPECT_EQ(fb.t2.velocity_actual_value, 1'000'000);
  EXPECT_EQ(fb.t1.statusword, l.drive.cia402().statusword());
  EXPECT_EQ(fb.t3.sample_time_us, to_wire_us(at));
  // The joint moves on after the SYNC; the sample stays the one taken at the SYNC.
  l.tick();
  EXPECT_EQ(l.drive.joint().position_nrad, 12'000'000);
  EXPECT_EQ(fb.t1.position_actual_value, 10'000);
}

TEST(Sync, LossStartsQuickStop) {
  Loop l;
  l.run_at_vmax();
  // SYNC stops; the master keeps sending RPDOs, which never actuate without SYNC.
  const MonoTime last_sync = l.now();
  for (int i = 0; i < 14; ++i) {
    l.tick();
    l.command(kFar);
  }
  EXPECT_EQ(l.now() - last_sync, milliseconds(28));
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
  l.tick();  // 3 x T_bus without SYNC.
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kSyncLoss);
  EXPECT_TRUE(l.drive.policy_latched());
}

TEST(Sync, DuplicateIsIgnored) {
  Loop l;
  l.enable();
  l.sync();
  l.command(1'000);
  l.ticks(5);
  const auto first = unpack(l.sync());  // Actuates seq 1.
  l.command(2'000);
  l.tick();
  EXPECT_TRUE(l.sync().empty());  // 2 ms after the previous SYNC: counted and ignored.
  EXPECT_EQ(l.drive.sync_duplicates(), 1U);
  EXPECT_EQ(l.dropped(DropReason::kDuplicateSync), 1U);
  ASSERT_TRUE(l.drive.actuated().has_value());
  EXPECT_EQ(l.drive.actuated()->command_seq, 1);  // The duplicate actuated nothing.
  l.ticks(4);
  const auto next = unpack(l.sync());
  EXPECT_EQ(l.drive.actuated()->command_seq, 2);
  EXPECT_EQ(next.t1.sample_seq, static_cast<std::uint16_t>(first.t1.sample_seq + 1));
}

TEST(Sync, DelayedWithinBoundIsTolerated) {
  Loop l;
  l.run_at_vmax();
  // The next SYNC comes 28 ms late; two commands wait in the window meanwhile.
  l.ticks(5);
  l.command(kFar);
  l.ticks(9);
  l.sync();
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
  EXPECT_EQ(l.drive.actuated()->command_seq, l.seq);  // The newest pair.
  EXPECT_EQ(l.dropped(DropReason::kSuperseded), 1U);
  l.command(kFar);
  for (int i = 0; i < 3; ++i) {
    l.period();
  }
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
  EXPECT_EQ(l.count(DriveEventKind::kQuickStop), 0U);
}

TEST(DriveGuard, NeverActuatedAtExpiry) {
  Loop l;
  l.enable();
  l.sync();
  // A pair whose deadline is exactly the next SYNC is never actuated.
  l.command(kFar, milliseconds(10));
  l.ticks(5);
  l.sync();
  EXPECT_FALSE(l.drive.actuated().has_value());
  EXPECT_EQ(l.count(DriveEventKind::kActuated), 0U);
  EXPECT_EQ(l.dropped(DropReason::kNotValid), 1U);

  // An actuated command stops being tracked at the tick where now == valid_until.
  l.command(kFar, milliseconds(20));
  const MonoTime until = l.now() + milliseconds(20);
  l.ticks(5);
  l.sync();
  ASSERT_TRUE(l.drive.actuated().has_value());
  while (l.now() + kW < until) {
    l.tick();
    EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
    EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax);
  }
  l.tick();
  ASSERT_EQ(l.now(), until);
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kExpiry);
  EXPECT_FALSE(l.drive.actuated().has_value());
  EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax - kDvPerTick);  // Braked, not tracked.
}

TEST(DriveGuard, ExpiredDetectedWithinW) {
  Loop l;
  l.enable();
  l.sync();
  l.command(kFar, microseconds(20'500));  // Deadline between two W ticks.
  const MonoTime until = l.now() + microseconds(20'500);
  l.ticks(5);
  l.sync();
  ASSERT_TRUE(l.drive.actuated().has_value());
  for (int guard = 0; l.state() == Cia402State::kOperationEnabled; ++guard) {
    ASSERT_LT(guard, 200);
    l.tick();
  }
  const auto stop = l.last(DriveEventKind::kQuickStop);
  ASSERT_TRUE(stop.has_value());
  EXPECT_EQ(stop->reason, QuickStopReason::kExpiry);
  EXPECT_GE(stop->at, until);
  EXPECT_LE(stop->at - until, kW);
}

TEST(Timeout, MissingCommandsStartQuickStop) {
  Loop l;
  l.run_at_vmax();
  // Discard the command sent after the last SYNC: one more SYNC actuates it, then none come.
  l.period(false);
  const MonoTime last_actuation = l.now();
  for (int i = 0; i < 2; ++i) {
    l.period(false);  // SYNC continues, no commands.
  }
  l.ticks(4);
  EXPECT_EQ(l.now() - last_actuation, milliseconds(28));
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
  l.tick();
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kTimeout);
  EXPECT_TRUE(l.drive.policy_latched());
  EXPECT_EQ(l.drive.issuer().current(), 0);
}

TEST(Timeout, HeartbeatDoesNotRenew) {
  Loop l;
  l.run_at_vmax();
  l.period(false);
  const MonoTime last_actuation = l.now();
  for (int guard = 0; l.state() == Cia402State::kOperationEnabled; ++guard) {
    ASSERT_LT(guard, 200);
    l.tick();
    l.frame(cc::encode_heartbeat({kOther, cc::NmtState::kOperational}));
    if ((l.now() - last_actuation).ns % milliseconds(10).ns == 0) {
      l.sync();
    }
  }
  EXPECT_EQ(l.now() - last_actuation, milliseconds(30));
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kTimeout);
}

TEST(Timeout, OperationEnabledWithoutTagTimesOut) {
  Loop l;
  // Standard 3+4 reaches Operation Enabled with no commit: nothing may actuate, and the command
  // timeout still applies.
  l.cw(0x06);
  l.cw(0x0F);
  ASSERT_EQ(l.state(), Cia402State::kOperationEnabled);
  ASSERT_EQ(l.drive.issuer().current(), 0);
  const MonoTime enabled = l.now();
  l.sync();
  for (int guard = 0; l.state() == Cia402State::kOperationEnabled; ++guard) {
    ASSERT_LT(guard, 200);
    l.command(kFar);  // tag 0: no tag is current.
    l.period(false);
  }
  EXPECT_EQ(l.count(DriveEventKind::kActuated), 0U);
  EXPECT_GT(l.dropped(DropReason::kOtherTag), 0U);
  EXPECT_EQ(l.drive.joint().position_nrad, 0);
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kTimeout);
  EXPECT_EQ(l.last(DriveEventKind::kQuickStop)->at - enabled, milliseconds(30));
}

TEST(Gap, HoldsLastValidTargetWithinValidity) {
  Loop l;
  l.run_at_vmax();
  const auto held = l.drive.actuated();
  ASSERT_TRUE(held.has_value());
  // The window before the next-but-one SYNC gets only RPDO1 of the next seq: an ordinary gap.
  l.period(false);  // Actuates the command already sent; nothing new follows it.
  const auto tracked = l.drive.actuated();
  ++l.seq;
  l.rpdo1(l.seq, kFar);
  l.period(false);
  EXPECT_EQ(l.dropped(DropReason::kMissingHalf), 1U);
  EXPECT_EQ(l.drive.actuated(), tracked);  // Last valid target held.
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
  EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax);
  // The stream resumes and the newest command is tracked again.
  l.command(kFar);
  l.period(false);
  EXPECT_EQ(l.drive.actuated()->command_seq, l.seq);
  EXPECT_EQ(l.count(DriveEventKind::kQuickStop), 0U);
}

TEST(Integrity, StopsWithinWWithoutFurtherSync) {
  Loop l;
  l.run_at_vmax();
  l.period(false);  // Actuated command, still valid for 38 ms.
  ASSERT_TRUE(l.drive.actuated().has_value());
  // Two different payloads for one seq, then no SYNC at all.
  l.tick();
  const MonoTime detected = l.now();
  ++l.seq;
  l.rpdo1(l.seq, kFar);
  l.rpdo1(l.seq, kFar - 1);
  EXPECT_EQ(l.count(DriveEventKind::kIntegrity), 1U);
  EXPECT_FALSE(l.drive.actuated().has_value());  // Blocked at once.
  l.tick();
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  EXPECT_EQ(l.stop_reason(), QuickStopReason::kIntegrity);
  EXPECT_LE(l.last(DriveEventKind::kQuickStop)->at - detected, kW);
  EXPECT_TRUE(l.drive.policy_latched());
  EXPECT_EQ(l.drive.issuer().current(), 0);
  EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax - kDvPerTick);
}

TEST(Integrity, LatchesOutsideOperationEnabled) {
  Loop l;
  l.cw(0x06);
  l.cw(0x07);
  l.commit();
  ++l.seq;
  l.rpdo1(l.seq, 5);
  l.rpdo1(l.seq, 6);
  l.tick();
  // No quick stop transition exists from Switched On, but the latch and the revoke still apply.
  EXPECT_EQ(l.state(), Cia402State::kSwitchedOn);
  EXPECT_TRUE(l.drive.policy_latched());
  EXPECT_EQ(l.drive.issuer().current(), 0);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kSwitchedOn);
  EXPECT_FALSE(l.drive.commit(2, 2, l.now()).has_value());
}

TEST(Stop, RampIsAchievedBothDirections) {
  for (const std::int32_t target : {kFar, -kFar}) {
    SCOPED_TRACE(target);
    Loop l;
    l.run_at_vmax(target);
    const std::int64_t v0 = l.drive.joint().velocity_nrad_s;
    const std::int64_t x0 = l.drive.joint().position_nrad;
    l.cw(0x02);  // Quick Stop.
    ASSERT_EQ(l.state(), Cia402State::kQuickStopActive);
    std::int64_t v = v0;
    int steps = 0;
    while (v != 0) {
      l.tick();
      const std::int64_t next = l.drive.joint().velocity_nrad_s;
      EXPECT_EQ(std::llabs(v) - std::llabs(next), kDvPerTick);
      EXPECT_TRUE(next == 0 || (next > 0) == (v0 > 0));
      v = next;
      ++steps;
      ASSERT_LT(steps, 100);
    }
    EXPECT_EQ(steps, 50);
    const std::int64_t travel = std::llabs(l.drive.joint().position_nrad - x0);
    EXPECT_LE(travel, kBrakeBoundNrad);
    EXPECT_EQ(travel, 49'000'000);
  }
}

TEST(Stop, TotalTravelWithinBoundAcrossPhases) {
  // The last command is sent with the full 48 ms lease and reaches the drive late (20 ms on the
  // bus), so the command timeout cannot preempt it and the expiry guard is what stops the joint.
  // The deadline sweeps 20 phases of one W. Travel counts from the joint position at the last W
  // tick at or before the send (an over-estimate) to rest.
  std::int64_t max_travel = 0;
  for (int phase = 0; phase < 20; ++phase) {
    SCOPED_TRACE(phase);
    Loop l;
    l.run_at_vmax();
    const MonoTime g0 = l.now();  // A SYNC instant; the last stream command was just sent.
    const MonoTime send = g0 + milliseconds(1) + microseconds(100 * phase);
    const MonoTime deliver = send + milliseconds(20);
    const std::uint32_t until = to_wire_us(send + milliseconds(48));
    std::optional<std::int64_t> x_send;
    bool delivered = false;
    MonoTime next_tick = g0 + kW;
    while (!l.drive.stop_confirmed()) {
      ASSERT_LT(l.now(), g0 + milliseconds(400));
      if (!x_send && send < next_tick) {
        l.clock.advance(send - l.now());
        x_send = l.drive.joint().position_nrad;
        continue;
      }
      if (!delivered && deliver < next_tick) {
        l.clock.advance(deliver - l.now());
        ++l.seq;
        l.rpdo1(l.seq, kFar);
        l.rpdo2(l.seq, until);
        delivered = true;
        continue;
      }
      l.clock.advance(next_tick - l.now());
      l.drive.tick(l.now());
      if ((l.now() - g0).ns % milliseconds(10).ns == 0) {
        l.sync();
      }
      next_tick = next_tick + kW;
    }
    ASSERT_TRUE(x_send.has_value());
    EXPECT_EQ(l.stop_reason(), QuickStopReason::kExpiry);
    ASSERT_GT(l.count(DriveEventKind::kActuated), 0U);
    EXPECT_EQ(l.last(DriveEventKind::kActuated)->seq, l.seq);  // The late command did actuate.
    const std::int64_t travel = l.drive.joint().position_nrad - *x_send;
    std::printf("phase %2d: travel %.6f rad\n", phase, static_cast<double>(travel) * 1e-9);
    EXPECT_LE(travel, kEAllowNrad);
    max_travel = std::max(max_travel, travel);
  }
  std::printf("max travel over phases: %.6f rad (e_allow 0.100000)\n",
              static_cast<double>(max_travel) * 1e-9);
  ::testing::Test::RecordProperty("max_travel_nrad", std::to_string(max_travel));
}

TEST(Stop, ConfirmsAfterFiveQuietTicks) {
  Loop l;
  l.run_at_vmax();
  l.cw(0x02);
  const MonoTime started = l.now();
  int quiet = 0;
  while (quiet < 5) {
    EXPECT_FALSE(l.drive.stop_confirmed());
    l.tick();
    quiet = std::llabs(l.drive.joint().velocity_nrad_s) < drive_core::kQuietVelocityNradS
                ? quiet + 1
                : 0;
    if (quiet < 5) {
      EXPECT_FALSE(l.drive.stop_confirmed()) << "confirmed after " << quiet << " quiet ticks";
    }
    ASSERT_LT(l.now() - started, milliseconds(400));
  }
  EXPECT_TRUE(l.drive.stop_confirmed());
  EXPECT_EQ(l.count(DriveEventKind::kStopConfirmed), 1U);
  EXPECT_LE(l.now() - started, drive_core::kStopDeadline);
  EXPECT_FALSE(l.drive.stop_unconfirmed());
  EXPECT_EQ(l.count(DriveEventKind::kStopUnconfirmed), 0U);
}

TEST(Stop, UnconfirmedPastDeadline) {
  Loop l;
  l.run_at_vmax();
  l.drive.inject_stuck_joint(true);
  l.cw(0x02);
  const MonoTime started = l.now();
  while (l.now() - started < Duration{drive_core::kStopDeadline.ns - kW.ns}) {
    l.tick();
    EXPECT_FALSE(l.drive.stop_unconfirmed());
  }
  l.tick();
  EXPECT_TRUE(l.drive.stop_unconfirmed());
  EXPECT_EQ(l.count(DriveEventKind::kStopUnconfirmed), 1U);
  EXPECT_FALSE(l.drive.stop_confirmed());
  EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax);
  l.ticks(50);
  EXPECT_EQ(l.count(DriveEventKind::kStopConfirmed), 0U);
  EXPECT_EQ(l.count(DriveEventKind::kStopUnconfirmed), 1U);
}

TEST(Stop, NewCommandDoesNotResume) {
  Loop l;
  l.run_at_vmax();
  for (int i = 0; i < 4; ++i) {
    l.period(false);
  }
  ASSERT_EQ(l.state(), Cia402State::kQuickStopActive);  // Command timeout.
  const auto actuated = l.count(DriveEventKind::kActuated);
  std::int64_t v = l.drive.joint().velocity_nrad_s;
  for (int i = 0; i < 20; ++i) {
    l.command(kFar);  // Old tag.
    EXPECT_FALSE(l.drive.commit(9, 9, l.now()).has_value());
    l.cw(0x0F);
    for (int k = 0; k < 5; ++k) {
      l.tick();
      EXPECT_LE(std::llabs(l.drive.joint().velocity_nrad_s), std::llabs(v));
      v = l.drive.joint().velocity_nrad_s;
    }
    l.sync();
    EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  }
  EXPECT_EQ(l.count(DriveEventKind::kActuated), actuated);
  EXPECT_EQ(v, 0);
  EXPECT_TRUE(l.drive.stop_confirmed());
}

// Runs into a policy stop (command timeout) from v_max.
void policy_stop(Loop& l) {
  l.run_at_vmax();
  for (int i = 0; i < 4; ++i) {
    l.period(false);
  }
  ASSERT_EQ(l.state(), Cia402State::kQuickStopActive);
  ASSERT_TRUE(l.drive.policy_latched());
}

void until_confirmed(Loop& l) {
  for (int i = 0; i < 200 && !l.drive.stop_confirmed(); ++i) {
    l.tick();
  }
  ASSERT_TRUE(l.drive.stop_confirmed());
}

TEST(Latch, DelayedEnableOperationIsRefused) {
  Loop l;
  policy_stop(l);
  l.cw(0x0F);  // An Enable Operation the master sent before it saw the stop.
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  until_confirmed(l);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kQuickStopActive);
  // Every other way into Operation Enabled is closed while latched too.
  l.cw(0x00);
  l.cw(0x06);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kReadyToSwitchOn);
  l.cw(0x07);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kSwitchedOn);
}

TEST(Latch, ResetWithoutEnableDoesNotMove) {
  Loop l;
  policy_stop(l);
  EXPECT_FALSE(l.drive.policy_reset(l.now()));  // Not before stop_confirmed.
  EXPECT_TRUE(l.drive.policy_latched());
  until_confirmed(l);
  ASSERT_TRUE(l.drive.policy_reset(l.now()));
  EXPECT_FALSE(l.drive.policy_latched());
  EXPECT_EQ(l.state(), Cia402State::kSwitchOnDisabled);
  const std::int64_t x = l.drive.joint().position_nrad;
  const auto actuated = l.count(DriveEventKind::kActuated);
  l.commit();  // Commits are accepted again, but nothing is enabled.
  l.sync();
  for (int i = 0; i < 5; ++i) {
    l.command(kFar);
    l.period(false);
  }
  EXPECT_EQ(l.drive.joint().position_nrad, x);
  EXPECT_EQ(l.count(DriveEventKind::kActuated), actuated);
  EXPECT_EQ(l.dropped(DropReason::kNotEnabled), 5U);
}

TEST(Latch, FullRearmResumes) {
  Loop l;
  policy_stop(l);
  until_confirmed(l);
  ASSERT_TRUE(l.drive.policy_reset(l.now()));
  const std::int64_t x = l.drive.joint().position_nrad;
  l.enable();  // 0x06, 0x07, a new commit, 0x0F.
  l.sync();
  l.command(kFar);
  l.period(false);
  ASSERT_TRUE(l.drive.actuated().has_value());
  EXPECT_EQ(l.drive.actuated()->exec_tag, l.tag);
  l.tick();
  EXPECT_EQ(l.drive.joint().position_nrad, x + 2'000'000);
}

TEST(Latch, LeavingOperationalStopsAndLatches) {
  for (const auto command : {cc::NmtCommand::kStop, cc::NmtCommand::kEnterPreOperational,
                             cc::NmtCommand::kResetCommunication, cc::NmtCommand::kResetNode}) {
    SCOPED_TRACE(static_cast<int>(command));
    Loop l;
    l.run_at_vmax();
    l.nmt(command);
    EXPECT_EQ(l.stop_reason(), QuickStopReason::kCommState);
    EXPECT_TRUE(l.drive.policy_latched());
    EXPECT_EQ(l.drive.issuer().current(), 0);
    EXPECT_FALSE(l.drive.actuated().has_value());
    const auto expected = command == cc::NmtCommand::kResetNode ? Cia402State::kSwitchOnDisabled
                                                                : Cia402State::kQuickStopActive;
    EXPECT_EQ(l.state(), expected);
    // The joint ramps down; nothing resumes it.
    l.tick();
    EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax - kDvPerTick);
    until_confirmed(l);
  }
}

TEST(Latch, SurvivesResetCommunicationAndResetNode) {
  Loop l;
  policy_stop(l);
  until_confirmed(l);
  l.nmt(cc::NmtCommand::kResetCommunication);
  EXPECT_TRUE(l.drive.policy_latched());
  l.nmt(cc::NmtCommand::kResetNode);
  EXPECT_TRUE(l.drive.policy_latched());
  EXPECT_EQ(l.state(), Cia402State::kSwitchOnDisabled);
  // The re-initialised CiA 402 machine carries the inhibit again.
  l.nmt(cc::NmtCommand::kStart);
  l.cw(0x06);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kReadyToSwitchOn);
  l.clock.advance(milliseconds(100));  // Past the new boot's quarantine.
  EXPECT_FALSE(l.drive.commit(3, 3, l.now()).has_value());
  // policy_reset after the (confirmed) stop clears it.
  until_confirmed(l);
  EXPECT_TRUE(l.drive.policy_reset(l.now()));
  l.cw(0x06);
  l.cw(0x07);
  EXPECT_TRUE(l.drive.commit(3, 3, l.now()).has_value());
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kOperationEnabled);
}

TEST(Disable, NormalDisableNeedsNoReset) {
  Loop l;
  l.run_at_vmax();
  l.cw(0x07);  // Disable Operation.
  EXPECT_EQ(l.state(), Cia402State::kSwitchedOn);
  EXPECT_FALSE(l.drive.policy_latched());
  EXPECT_EQ(l.drive.issuer().current(), 0);
  EXPECT_FALSE(l.drive.actuated().has_value());
  EXPECT_EQ(l.count(DriveEventKind::kQuickStop), 0U);
  EXPECT_FALSE(l.drive.policy_reset(l.now()));  // Nothing to reset.
  l.ticks(60);
  EXPECT_EQ(l.drive.joint().velocity_nrad_s, 0);
  // A new commit and 0x0F resume.
  l.commit();
  l.cw(0x0F);
  ASSERT_EQ(l.state(), Cia402State::kOperationEnabled);
  l.sync();
  l.command(kFar);
  l.period(false);
  ASSERT_TRUE(l.drive.actuated().has_value());
  EXPECT_EQ(l.drive.actuated()->exec_tag, l.tag);
}

TEST(Fault, RecoverableFaultRamps) {
  Loop l;
  l.run_at_vmax();
  l.drive.inject_fault(drive_core::kErrorFollowing, l.now());
  EXPECT_EQ(l.state(), Cia402State::kFaultReactionActive);
  ASSERT_TRUE(l.last(DriveEventKind::kFault).has_value());
  EXPECT_EQ(l.last(DriveEventKind::kFault)->fault_code, drive_core::kErrorFollowing);
  EXPECT_EQ(l.drive.issuer().current(), 0);
  std::int64_t v = l.drive.joint().velocity_nrad_s;
  for (int guard = 0; v != 0; ++guard) {
    ASSERT_LT(guard, 200);
    l.tick();
    EXPECT_EQ(v - l.drive.joint().velocity_nrad_s, kDvPerTick);
    v = l.drive.joint().velocity_nrad_s;
    EXPECT_EQ(l.state(), Cia402State::kFaultReactionActive);
  }
  until_confirmed(l);
  EXPECT_EQ(l.state(), Cia402State::kFault);
  EXPECT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorFollowing);
  EXPECT_FALSE(l.drive.policy_latched());
}

TEST(Fault, TorqueLossIsNotAControlledStop) {
  Loop l;
  l.run_at_vmax();
  l.drive.inject_torque_loss(l.now());
  EXPECT_EQ(l.state(), Cia402State::kFault);  // At once, no reaction ramp.
  EXPECT_TRUE(l.drive.drive_unavailable());
  EXPECT_TRUE(l.drive.stop_unconfirmed());
  EXPECT_EQ(l.count(DriveEventKind::kStopUnconfirmed), 1U);
  EXPECT_EQ(l.last(DriveEventKind::kFault)->fault_code, drive_core::kErrorTorqueLoss);
  std::int64_t x = l.drive.joint().position_nrad;
  for (int i = 0; i < 100; ++i) {
    l.tick();
    EXPECT_EQ(l.drive.joint().velocity_nrad_s, kVMax);  // Residual velocity, uncontrolled.
    EXPECT_EQ(l.drive.joint().position_nrad, x + 2'000'000);
    x = l.drive.joint().position_nrad;
  }
  EXPECT_EQ(l.count(DriveEventKind::kQuickStop), 0U);
  EXPECT_EQ(l.count(DriveEventKind::kStopConfirmed), 0U);
  EXPECT_FALSE(l.drive.stop_confirmed());
}

TEST(Fault, TorqueLossHoldsFaultAcrossResetAndResetNode) {
  constexpr std::uint16_t kSwFault = 1U << 3U;
  constexpr std::uint16_t kSwSwitchOnDisabled = 1U << 6U;
  Loop l;
  l.run_at_vmax();
  l.drive.inject_torque_loss(l.now());
  ASSERT_EQ(l.state(), Cia402State::kFault);
  // A Fault Reset cannot leave Fault while the drive authority is lost: the statusword keeps
  // showing Fault, never an enable-able Switch On Disabled.
  for (int i = 0; i < 2; ++i) {
    l.cw(0x00);
    l.cw(0x80);
    EXPECT_EQ(l.state(), Cia402State::kFault);
    EXPECT_EQ(l.drive.cia402().statusword() & kSwFault, kSwFault);
    EXPECT_EQ(l.drive.cia402().statusword() & kSwSwitchOnDisabled, 0);
    EXPECT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorTorqueLoss);
  }
  l.cw(0x06);
  l.cw(0x07);
  l.cw(0x0F);
  EXPECT_EQ(l.state(), Cia402State::kFault);
  // Nor does a reset node: the re-initialised machine goes straight back to Fault.
  l.nmt(cc::NmtCommand::kResetNode);
  EXPECT_EQ(l.state(), Cia402State::kFault);
  EXPECT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorTorqueLoss);
  EXPECT_EQ(l.drive.cia402().statusword() & kSwFault, kSwFault);
  EXPECT_TRUE(l.drive.drive_unavailable());
}

TEST(Fault, TorqueLossDuringFaultHandlingUpdatesCode) {
  {
    SCOPED_TRACE("Fault Reaction Active");
    Loop l;
    l.run_at_vmax();
    l.drive.inject_fault(drive_core::kErrorFollowing, l.now());
    ASSERT_EQ(l.state(), Cia402State::kFaultReactionActive);
    l.tick();
    l.drive.inject_torque_loss(l.now());
    EXPECT_EQ(l.state(), Cia402State::kFault);  // The ramp ends: there is no actuator.
    EXPECT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorTorqueLoss);
    EXPECT_EQ(l.last(DriveEventKind::kFault)->fault_code, drive_core::kErrorTorqueLoss);
    const std::int64_t v = l.drive.joint().velocity_nrad_s;
    l.tick();
    EXPECT_EQ(l.drive.joint().velocity_nrad_s, v);  // Coasting, not braking.
  }
  {
    SCOPED_TRACE("Fault");
    Loop l;
    l.cw(0x06);
    l.cw(0x07);
    l.drive.inject_fault(drive_core::kErrorFollowing, l.now());
    l.ticks(10);
    ASSERT_EQ(l.state(), Cia402State::kFault);
    ASSERT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorFollowing);
    l.drive.inject_torque_loss(l.now());
    EXPECT_EQ(l.state(), Cia402State::kFault);
    EXPECT_EQ(l.drive.cia402().fault_code(), drive_core::kErrorTorqueLoss);
  }
}

TEST(Fault, ResetNeedsNewCommitAndEnable) {
  Loop l;
  l.run_at_vmax();
  const std::uint16_t old_tag = l.tag;
  l.drive.inject_fault(drive_core::kErrorFollowing, l.now());
  until_confirmed(l);
  ASSERT_EQ(l.state(), Cia402State::kFault);
  l.cw(0x80);  // Fault Reset.
  EXPECT_EQ(l.state(), Cia402State::kSwitchOnDisabled);
  const std::int64_t x = l.drive.joint().position_nrad;
  const auto actuated = l.count(DriveEventKind::kActuated);
  l.cw(0x06);
  l.cw(0x07);
  // The old tag is dead.
  l.sync();
  l.command(kFar);
  l.period(false);
  EXPECT_EQ(l.dropped(DropReason::kOtherTag), 2U);
  // A new commit without Enable Operation does not move.
  EXPECT_NE(l.commit(), old_tag);
  l.command(kFar);
  l.period(false);
  EXPECT_EQ(l.dropped(DropReason::kNotEnabled), 1U);
  EXPECT_EQ(l.count(DriveEventKind::kActuated), actuated);
  EXPECT_EQ(l.drive.joint().position_nrad, x);
  // Enable Operation with the new tag resumes.
  l.cw(0x0F);
  l.command(kFar);
  l.period(false);
  EXPECT_EQ(l.count(DriveEventKind::kActuated), actuated + 1);
  l.tick();
  EXPECT_EQ(l.drive.joint().position_nrad, x + 2'000'000);
}

TEST(Tpdo, JoinsBySampleSeq) {
  Loop l;
  l.enable();
  l.sync();
  std::optional<std::uint16_t> previous;
  for (int i = 0; i < 4; ++i) {
    l.command(1'000 * (i + 1));
    l.ticks(5);
    const MonoTime at = l.now();
    const auto fb = unpack(l.sync());
    EXPECT_EQ(fb.t1.sample_seq, fb.t2.sample_seq);
    EXPECT_EQ(fb.t1.sample_seq, fb.t3.sample_seq);
    if (previous) {
      EXPECT_EQ(fb.t1.sample_seq, static_cast<std::uint16_t>(*previous + 1));
    }
    previous = fb.t1.sample_seq;
    EXPECT_EQ(fb.t2.applied_seq, l.seq);  // The command actuated at this SYNC.
    EXPECT_EQ(fb.t3.applied_tag, l.tag);
    EXPECT_EQ(fb.t3.sample_time_us, to_wire_us(at));
  }
}

}  // namespace
