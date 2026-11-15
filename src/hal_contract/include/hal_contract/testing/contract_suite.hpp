// SPDX-License-Identifier: Apache-2.0
#pragma once

// The backend contract suite: one gtest typed suite every hal_contract::Backend runs through
// the Hal, in logic time. A backend's test instantiates it with its own fixture:
//
//   using Fixtures = ::testing::Types<MyFixture>;
//   INSTANTIATE_TYPED_TEST_SUITE_P(MyBackend, Contract, Fixtures);
//
// The cases judge what the device does (its own state and events, through the fixture) and
// what the Hal reports, never the backend's internals. Numbers are the E3a/E3b project values
// (W = 2 ms, T_bus = 10 ms, lease 48 ms, command timeout 30 ms, v_max 1 rad/s, a_stop 10 rad/s^2)
// and the bounds derived from them in the E3c plan (C17-C19).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <vector>

#include "hal_contract/backend.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/time.hpp"

namespace hal_contract {

// Readable gtest failure messages for the Hal's enums (found by argument-dependent lookup).
inline void PrintTo(HalVerdict v, std::ostream* os) { *os << to_string(v); }
inline void PrintTo(StopStatus s, std::ostream* os) { *os << to_string(s); }
inline void PrintTo(OpState s, std::ostream* os) { *os << to_string(s); }
inline void PrintTo(ArmState s, std::ostream* os) { *os << to_string(s); }

}  // namespace hal_contract

namespace hal_contract::testing {

inline constexpr runtime_core::Duration kW = runtime_core::milliseconds(2);
inline constexpr runtime_core::Duration kTBus = runtime_core::milliseconds(10);
// The controller period the suite writes at (one write per T_bus, as ros2_control at 100 Hz).
inline constexpr runtime_core::Duration kPeriod = runtime_core::milliseconds(10);
inline constexpr runtime_core::Duration kMaxFeedbackAge = runtime_core::milliseconds(20);
inline constexpr double kVMax = 1.0;           // rad/s
inline constexpr double kRestVelocity = 0.01;  // rad/s, "at rest" for C18
// After the last approval: lease + v_max / a_stop + 6 W (Contract.ExpiredPermitStopsWithinBound).
inline constexpr runtime_core::Duration kStopBound = runtime_core::milliseconds(160);
inline constexpr double kMaxTravelAfterApproval = 0.100;  // rad
// C18 host deadline for stop confirmation, from the inhibit.
inline constexpr runtime_core::Duration kStopConfirmDeadline = runtime_core::milliseconds(172);
// Every axis gets its stop within 2 x T_bus of the injected single-axis stop (C17; plan Task 4
// Session.OneNodeStopInhibitsGroup: "Quick Stop to the other five within 2 x T_bus"). Measured
// from the injection, so it includes the backend's time to notice the stop and latch.
inline constexpr runtime_core::Duration kGroupStopLatency = runtime_core::milliseconds(20);
// C19's widest steady-state acceptance bound (CANopen: 2 x T_bus). A fixture may declare a
// tighter one (Direct: 1 W), never a looser one.
inline constexpr runtime_core::Duration kMaxAcceptanceLatency = runtime_core::milliseconds(20);
// Distinct, non-zero joint positions: a backend that reports zeros or swaps joints is caught.
inline constexpr Joints kStartPosition{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
// Ramp step per controller period: 0.9 rad/s, under v_max with margin for rounding.
inline constexpr double kRampStep = 0.009;

// What the device did, as the device saw it (the drive's own trace, or the joint model).
enum class DeviceEventKind : std::uint8_t {
  kActuated,   // a position target was applied this step (approved command or arm hold)
  kDisabled,   // the host disabled operation (re-arm phase 2, deactivate)
  kCommitted,  // a new execution identity was committed (re-arm phase 3)
  kEnabled,    // operation enabled under the committed identity
  kLocalStop,  // the device stopped on its own: expiry, command timeout, hold end, injected
  kQuickStop,  // the host's group inhibit ordered this axis to stop
};

struct DeviceEvent {
  runtime_core::MonoTime at{};
  std::size_t node{0};
  DeviceEventKind kind{DeviceEventKind::kActuated};
  double target{0.0};    // kActuated: the applied target
  double position{0.0};  // the axis position when the event happened
  double velocity{0.0};  // the axis velocity when the event happened
  // kActuated: the approved command's source_seq; nullopt for an arm hold.
  std::optional<std::uint64_t> source_seq{};
  // kActuated: the command's identity; kCommitted: the committed identity.
  AuthorityId id{};
};

// The device's true state (not the feedback the backend reports).
struct DeviceState {
  Joints position{};
  Joints velocity{};
};

// What a backend's test supplies. Every hook is backend-neutral: it names an effect on the
// device or the path between host and device, not a mechanism of one backend.
//
//   make()                    the backend, once per test, with the device at rest at
//                             kStartPosition, unconfigured. The fixture keeps a way to step it.
//   now()                     the current logic time.
//   advance(d)                moves logic time by d, stepping the device and the backend.
//   stall_feedback(b)         while true, no new device sample reaches the backend (the samples
//                             are lost, not queued); commands still flow.
//   delay_feedback(d)         from now on, each sample reaches the backend d after its source
//                             time (a transport delay), in order.
//   stop_accepting_commands() from now on, no submitted command reaches the device.
//   inject_node_stop(n)       axis n stops on its own at its next step, as on a lost command or
//                             a drive fault on that axis alone.
//   device()                  the device's true state.
//   drain_events()            the device events since the last drain, in time order.
//   Traits::kArmLatency       arm op to armed, from rest (C19); at most kArmTimeout.
//   Traits::kAcceptanceLatency a retained command to group_applied_source_seq (C19: 2 x T_bus
//                             on CANopen, 1 W on Direct).
template <class F>
concept BackendFixture =
    requires(F f, const F cf, runtime_core::Duration d, bool b, std::size_t n) {
      { f.make() } -> std::same_as<std::unique_ptr<Backend>>;
      { cf.now() } -> std::same_as<runtime_core::MonoTime>;
      f.advance(d);
      f.stall_feedback(b);
      f.delay_feedback(d);
      f.stop_accepting_commands();
      f.inject_node_stop(n);
      { cf.device() } -> std::same_as<DeviceState>;
      { f.drain_events() } -> std::same_as<std::vector<DeviceEvent>>;
      { F::Traits::kArmLatency } -> std::convertible_to<runtime_core::Duration>;
      { F::Traits::kAcceptanceLatency } -> std::convertible_to<runtime_core::Duration>;
    };

// runtime_core defines MonoTime +/- Duration only; the suite adds durations through its public,
// overflow-checked MonoTime operators.
constexpr runtime_core::Duration plus(runtime_core::Duration a, runtime_core::Duration b) {
  return (runtime_core::MonoTime{} + a + b) - runtime_core::MonoTime{};
}

inline runtime_core::Requirements all_requirements() { return {true, true, true, true, true}; }

inline runtime_core::JointLimits<kJoints> suite_limits() {
  runtime_core::JointLimits<kJoints> l{};
  for (std::size_t i = 0; i < kJoints; ++i) {
    l.pos_min[i] = -3.0;
    l.pos_max[i] = 3.0;
    l.vel_max[i] = kVMax;
    l.acc_max[i] = 100.0;
  }
  return l;
}

inline runtime_core::MotionPermit permit_for(const AuthorityId& id,
                                             runtime_core::MonoTime expires_at) {
  return runtime_core::MotionPermit{id.manager, id.session, id.generation, expires_at};
}

inline double direction(std::size_t joint) { return joint % 2 == 0 ? 1.0 : -1.0; }

// A ramp at kRampStep per period from `base`, every axis moving (alternating directions).
struct Ramp {
  AuthorityId id{};
  Joints base{};
  std::uint64_t seq{0};

  HalSetpoint next() {
    ++seq;
    return at(seq);
  }
  HalSetpoint at(std::uint64_t k) const {
    HalSetpoint sp{};
    sp.id = id;
    sp.snapshot.generation = id.generation;
    sp.snapshot.source_seq = k;
    for (std::size_t i = 0; i < kJoints; ++i) {
      sp.snapshot.position[i] = base[i] + direction(i) * kRampStep * static_cast<double>(k);
    }
    return sp;
  }
};

struct WriteRecord {
  runtime_core::MonoTime at{};
  AuthorityId id{};
  std::uint64_t source_seq{0};
  HalVerdict verdict{HalVerdict::kSend};
};

struct TraceSample {
  runtime_core::MonoTime at{};
  DeviceState device{};
  Feedback feedback{};
};

template <BackendFixture F>
class Contract : public ::testing::Test {
  // A fixture cannot loosen the contract's own latency bounds (C19, C10).
  static_assert(runtime_core::Duration{F::Traits::kAcceptanceLatency} <= kMaxAcceptanceLatency,
                "Traits::kAcceptanceLatency exceeds C19's 2 x T_bus");
  static_assert(runtime_core::Duration{F::Traits::kArmLatency} <= kArmTimeout,
                "Traits::kArmLatency exceeds the arm op deadline (C10)");

 protected:
  Contract()
      : hal_(fx_.make(), all_requirements(), runtime_core::GuardConfig{kPeriod, kMaxFeedbackAge},
             suite_limits()) {}

  void SetUp() override {
    const DeviceState d = fx_.device();
    for (std::size_t i = 0; i < kJoints; ++i) {
      ASSERT_EQ(d.position[i], kStartPosition[i])
          << "INCONCLUSIVE: make() must place the device at kStartPosition (joint " << i << ")";
      ASSERT_EQ(d.velocity[i], 0.0) << "INCONCLUSIVE: make() must leave the device at rest";
    }
  }

  runtime_core::MonoTime now() const { return fx_.now(); }

  // Advances `d` in W steps; after each step records the device, the events and a Hal read.
  void step(runtime_core::Duration d) {
    for (runtime_core::Duration done{}; done < d; done = plus(done, kW)) {
      fx_.advance(kW);
      std::vector<DeviceEvent> fresh = fx_.drain_events();
      events_.insert(events_.end(), fresh.begin(), fresh.end());
      feedback_ = hal_.read(now());
      trace_.push_back(TraceSample{now(), fx_.device(), feedback_});
    }
  }

  // One controller period: read, write, then advance one period.
  HalVerdict cycle(const runtime_core::MotionPermit* permit, const HalSetpoint& sp) {
    feedback_ = hal_.read(now());
    const HalVerdict v = hal_.write(permit, sp, now());
    writes_.push_back(WriteRecord{now(), sp.id, sp.snapshot.source_seq, v});
    step(kPeriod);
    return v;
  }

  // One period of the ramp under a permit renewed to now + 200 ms.
  HalVerdict ramp_cycle(Ramp& ramp) {
    const runtime_core::MotionPermit p =
        permit_for(ramp.id, now() + runtime_core::milliseconds(200));
    return cycle(&p, ramp.next());
  }

  // Steps in W until the op leaves kPending or `limit` passes; returns the time it took.
  runtime_core::Duration settle(OpHandle h, runtime_core::Duration limit) {
    const runtime_core::MonoTime start = now();
    while (hal_.op_state(h) == OpState::kPending && now() - start < limit) step(kW);
    return now() - start;
  }

  void bring_up() {
    const OpHandle c = hal_.configure(now());
    settle(c, kConfigureTimeout);
    ASSERT_EQ(hal_.op_state(c), OpState::kSucceeded) << "INCONCLUSIVE: configure failed";
    const OpHandle a = hal_.activate(now());
    settle(a, kActivateTimeout);
    ASSERT_EQ(hal_.op_state(a), OpState::kSucceeded) << "INCONCLUSIVE: activate failed";
  }

  // Arms `id` with a hold of `hold`; returns the arm latency.
  runtime_core::Duration arm(const AuthorityId& id, runtime_core::Duration hold) {
    const OpHandle h = hal_.arm(ArmRequest{id, now() + hold, ++request_id_}, now());
    const runtime_core::Duration latency = settle(h, kArmTimeout);
    EXPECT_EQ(hal_.op_state(h), OpState::kSucceeded) << "arm did not complete";
    feedback_ = hal_.read(now());
    return latency;
  }

  Ramp ramp_from_here(const AuthorityId& id) {
    feedback_ = hal_.read(now());
    return Ramp{id, fx_.device().position, 0};
  }

  bool moving(const DeviceState& d) const {
    return std::any_of(d.velocity.begin(), d.velocity.end(),
                       [](double v) { return std::abs(v) >= 0.5; });
  }

  // The last recorded time at which any axis moved at kRestVelocity or faster.
  runtime_core::MonoTime motion_end() const {
    runtime_core::MonoTime end{};
    for (const TraceSample& s : trace_) {
      for (double v : s.device.velocity) {
        if (std::abs(v) >= kRestVelocity) end = s.at;
      }
    }
    return end;
  }

  const TraceSample& sample_at(runtime_core::MonoTime t) const {
    for (const TraceSample& s : trace_) {
      if (s.at >= t) return s;
    }
    return trace_.back();
  }

  std::vector<DeviceEvent> events_of(DeviceEventKind kind) const {
    std::vector<DeviceEvent> out;
    for (const DeviceEvent& e : events_) {
      if (e.kind == kind) out.push_back(e);
    }
    return out;
  }

  // Asserts the motion after `last_approval` ended within kStopBound and travelled at most
  // kMaxTravelAfterApproval on every axis, and that the device was moving at that time.
  void expect_stopped_within_bound(runtime_core::MonoTime last_approval) {
    const TraceSample& at = sample_at(last_approval);
    ASSERT_TRUE(moving(at.device))
        << "INCONCLUSIVE: the device was not moving at the last approval";
    const runtime_core::MonoTime end = motion_end();
    EXPECT_LE((end - last_approval).ns, kStopBound.ns)
        << "motion ended " << (end - last_approval).ns << " ns after the last approval";
    const DeviceState final_state = trace_.back().device;
    for (std::size_t i = 0; i < kJoints; ++i) {
      EXPECT_LE(std::abs(final_state.position[i] - at.device.position[i]), kMaxTravelAfterApproval)
          << "joint " << i;
      EXPECT_LT(std::abs(final_state.velocity[i]), kRestVelocity) << "joint " << i;
    }
  }

  static constexpr AuthorityId kA{runtime_core::ManagerInstance{1}, runtime_core::SessionId{1},
                                  runtime_core::Generation{1}};

  F fx_;
  Hal hal_;
  Feedback feedback_{};
  std::vector<DeviceEvent> events_;
  std::vector<TraceSample> trace_;
  std::vector<WriteRecord> writes_;
  std::uint64_t request_id_{0};
};

TYPED_TEST_SUITE_P(Contract);

TYPED_TEST_P(Contract, NoCommandBeforeActivation) {
  const runtime_core::MotionPermit p =
      permit_for(this->kA, this->now() + runtime_core::milliseconds(500));
  Ramp ramp{this->kA, kStartPosition, 0};
  EXPECT_EQ(this->cycle(&p, ramp.next()), HalVerdict::kHardwareNotActive);
  const OpHandle c = this->hal_.configure(this->now());
  this->settle(c, kConfigureTimeout);
  ASSERT_EQ(this->hal_.op_state(c), OpState::kSucceeded) << "INCONCLUSIVE: configure failed";
  EXPECT_EQ(this->cycle(&p, ramp.next()), HalVerdict::kHardwareNotActive);
  const OpHandle a = this->hal_.activate(this->now());
  this->settle(a, kActivateTimeout);
  ASSERT_EQ(this->hal_.op_state(a), OpState::kSucceeded) << "INCONCLUSIVE: activate failed";
  for (int i = 0; i < 10; ++i) EXPECT_EQ(this->cycle(&p, ramp.next()), HalVerdict::kNotArmed);

  EXPECT_TRUE(this->events_of(DeviceEventKind::kActuated).empty());
  EXPECT_TRUE(this->events_of(DeviceEventKind::kEnabled).empty());
  const DeviceState d = this->fx_.device();
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(d.position[i], kStartPosition[i]) << "joint " << i;
    EXPECT_EQ(d.velocity[i], 0.0) << "joint " << i;
  }
}

TYPED_TEST_P(Contract, ArmHoldsActualPositionWithoutPermit) {
  this->bring_up();
  const runtime_core::Duration latency = this->arm(this->kA, runtime_core::milliseconds(500));
  EXPECT_LE(latency.ns, runtime_core::Duration{TypeParam::Traits::kArmLatency}.ns);
  for (int i = 0; i < 30; ++i) {
    EXPECT_EQ(this->cycle(nullptr, Ramp{this->kA, kStartPosition, 0}.at(1)), HalVerdict::kNoPermit);
  }
  const std::vector<DeviceEvent> actuated = this->events_of(DeviceEventKind::kActuated);
  ASSERT_FALSE(actuated.empty()) << "INCONCLUSIVE: the armed device never applied a hold";
  for (const DeviceEvent& e : actuated) {
    EXPECT_LE(std::abs(e.target - kStartPosition[e.node]), 1e-6) << "node " << e.node;
    EXPECT_FALSE(e.source_seq.has_value()) << "an approved command without a permit";
  }
  EXPECT_TRUE(this->events_of(DeviceEventKind::kLocalStop).empty());
  const DeviceState d = this->fx_.device();
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_LE(std::abs(d.position[i] - kStartPosition[i]), 1e-6) << "joint " << i;
    EXPECT_LE(std::abs(this->feedback_.position[i] - kStartPosition[i]), 1e-6) << "joint " << i;
  }
  EXPECT_FALSE(this->feedback_.faults.any()) << this->feedback_.faults.bits();
  EXPECT_EQ(this->hal_.arm_state(), ArmState::kArmed);
}

TYPED_TEST_P(Contract, ArmHoldEndsAtHoldUntil) {
  this->bring_up();
  const runtime_core::MonoTime hold_until = this->now() + runtime_core::milliseconds(200);
  this->arm(this->kA, hold_until - this->now());
  const HalSetpoint sp = Ramp{this->kA, kStartPosition, 0}.at(1);
  while (this->now() + kPeriod < hold_until) {
    this->cycle(nullptr, sp);
    EXPECT_FALSE(this->feedback_.faults.has(Fault::kPolicyStop));
  }
  while (this->now() < hold_until + kStopConfirmDeadline + kTBus) this->cycle(nullptr, sp);

  const std::vector<DeviceEvent> stops = this->events_of(DeviceEventKind::kLocalStop);
  ASSERT_FALSE(stops.empty()) << "the hold never ended";
  EXPECT_GE(stops.front().at, hold_until);
  EXPECT_LE((stops.front().at - hold_until).ns, kW.ns);
  for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
    EXPECT_LT(e.at, hold_until) << "a hold applied at or after hold_until";
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kPolicyStop));
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kInhibited));
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(this->feedback_.stop[i], StopStatus::kConfirmed) << "node " << i;
  }
  const runtime_core::MotionPermit p =
      permit_for(this->kA, this->now() + runtime_core::milliseconds(200));
  EXPECT_EQ(this->cycle(&p, sp), HalVerdict::kInhibited);
}

TYPED_TEST_P(Contract, TracksApprovedCommandsWithinVmax) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 50; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
  this->step(runtime_core::Duration{TypeParam::Traits::kAcceptanceLatency});
  const HalSetpoint last = ramp.at(ramp.seq);
  const DeviceState d = this->fx_.device();
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_LE(std::abs(d.position[i] - last.snapshot.position[i]), 1e-6) << "joint " << i;
  }
  double fastest = 0.0;
  std::size_t approved = 0;
  for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
    EXPECT_LE(std::abs(e.velocity), kVMax + 1e-9) << "node " << e.node;
    fastest = std::max(fastest, std::abs(e.velocity));
    if (!e.source_seq.has_value()) continue;
    ++approved;
    // Every applied target is exactly the approved position of that seq.
    EXPECT_EQ(e.id, this->kA);
    EXPECT_LE(std::abs(e.target - ramp.at(*e.source_seq).snapshot.position[e.node]), 1e-6)
        << "node " << e.node;
  }
  for (const TraceSample& s : this->trace_) {
    for (double v : s.device.velocity) EXPECT_LE(std::abs(v), kVMax + 1e-9);
  }
  ASSERT_GT(approved, 0U) << "INCONCLUSIVE: no approved command was applied";
  EXPECT_GE(fastest, 0.5) << "INCONCLUSIVE: the device never moved";
  EXPECT_TRUE(this->events_of(DeviceEventKind::kLocalStop).empty());
}

TYPED_TEST_P(Contract, ExpiredPermitStopsWithinBound) {
  // (a) The permit expires while the controller keeps writing the ramp.
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  const runtime_core::MotionPermit p =
      permit_for(this->kA, this->now() + runtime_core::milliseconds(100));
  for (int i = 0; i < 40; ++i) this->cycle(&p, ramp.next());
  runtime_core::MonoTime last_approval{};
  for (const WriteRecord& w : this->writes_) {
    if (w.verdict == HalVerdict::kSend) last_approval = w.at;
  }
  ASSERT_GT(last_approval.ns, 0) << "INCONCLUSIVE: nothing was approved";
  EXPECT_LT(last_approval, p.expires_at);
  this->expect_stopped_within_bound(last_approval);
  EXPECT_FALSE(this->events_of(DeviceEventKind::kLocalStop).empty());
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kPolicyStop));

  // (b) Commands keep being approved but stop reaching the device.
  this->trace_.clear();
  this->events_.clear();
  this->writes_.clear();
  this->bring_up();
  const AuthorityId b{runtime_core::ManagerInstance{1}, runtime_core::SessionId{1},
                      runtime_core::Generation{2}};
  this->arm(b, runtime_core::milliseconds(500));
  Ramp ramp_b = this->ramp_from_here(b);
  for (int i = 0; i < 15; ++i) ASSERT_EQ(this->ramp_cycle(ramp_b), HalVerdict::kSend) << i;
  const runtime_core::MonoTime cut = this->now();
  const std::uint64_t last_delivered = ramp_b.seq;
  this->fx_.stop_accepting_commands();
  for (int i = 0; i < 30; ++i) this->ramp_cycle(ramp_b);
  // The last command the device got was approved one period before the cut.
  this->expect_stopped_within_bound(cut - kPeriod);
  for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
    if (e.source_seq.has_value()) {
      EXPECT_LE(*e.source_seq, last_delivered);
    }
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kPolicyStop));
}

TYPED_TEST_P(Contract, RepeatedSnapshotDoesNotExtendLife) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 20; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
  const runtime_core::MonoTime original_approval = this->now();
  ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend);
  const HalSetpoint repeated = ramp.at(ramp.seq);
  const runtime_core::MonoTime original_deadline = original_approval + kMaxLease;
  for (int i = 0; i < 30; ++i) {
    // The permit stays valid: only the duplicate rule keeps the deadline where it was.
    const runtime_core::MotionPermit p =
        permit_for(this->kA, this->now() + runtime_core::milliseconds(200));
    const HalVerdict v = this->cycle(&p, repeated);
    EXPECT_TRUE(v == HalVerdict::kDuplicate || v == HalVerdict::kInhibited) << to_string(v);
  }
  for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
    if (e.source_seq == repeated.snapshot.source_seq) {
      EXPECT_LT(e.at, original_deadline) << "the repeated command outlived its deadline";
    }
  }
  const std::vector<DeviceEvent> stops = this->events_of(DeviceEventKind::kLocalStop);
  ASSERT_FALSE(stops.empty()) << "the device never stopped";
  EXPECT_LE(stops.front().at, original_deadline + kW);
  this->expect_stopped_within_bound(original_approval);
}

TYPED_TEST_P(Contract, OldIdentityRefusedAfterReadoption) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 10; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;

  // The manager restarted: same numeric session and generation, a new instance.
  const AuthorityId b{runtime_core::ManagerInstance{2}, this->kA.session, this->kA.generation};
  const OpHandle h =
      this->hal_.arm(ArmRequest{b, this->now() + runtime_core::milliseconds(500), 99}, this->now());
  const runtime_core::MotionPermit old_permit =
      permit_for(this->kA, this->now() + runtime_core::milliseconds(200));
  EXPECT_EQ(this->cycle(&old_permit, ramp.next()), HalVerdict::kNotArmed);
  this->settle(h, kArmTimeout);
  ASSERT_EQ(this->hal_.op_state(h), OpState::kSucceeded) << "INCONCLUSIVE: re-arm failed";
  const runtime_core::MonoTime readopted = this->now();

  const runtime_core::MotionPermit new_permit =
      permit_for(b, this->now() + runtime_core::milliseconds(200));
  Ramp ramp_b = this->ramp_from_here(b);
  EXPECT_EQ(this->cycle(&old_permit, ramp.next()), HalVerdict::kIdentityMismatch);
  EXPECT_EQ(this->cycle(&new_permit, ramp.next()), HalVerdict::kIdentityMismatch);
  EXPECT_EQ(this->cycle(&old_permit, ramp_b.at(1)), HalVerdict::kIdentityMismatch);
  EXPECT_EQ(this->cycle(&new_permit, ramp_b.next()), HalVerdict::kSend);

  bool committed_b = false;
  for (const DeviceEvent& e : this->events_) {
    if (e.kind == DeviceEventKind::kCommitted && e.id == b) committed_b = true;
    if (e.kind == DeviceEventKind::kActuated && e.source_seq.has_value() && e.at >= readopted) {
      EXPECT_EQ(e.id, b) << "an old-identity command reached the device after re-adoption";
    }
  }
  EXPECT_TRUE(committed_b);
}

TYPED_TEST_P(Contract, LostFeedbackTurnsStale) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 10; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
  const runtime_core::MonoTime stalled = this->now();
  this->fx_.stall_feedback(true);
  while (this->now() < stalled + kMaxFeedbackAge + kStopConfirmDeadline + kTBus + kTBus) {
    const Feedback fb = this->hal_.read(this->now());
    EXPECT_LE(fb.sampled_at, stalled) << "the snapshot time advanced without a sample";
    const HalVerdict v = this->ramp_cycle(ramp);
    if (this->now() - kPeriod - fb.sampled_at > kMaxFeedbackAge) {
      EXPECT_NE(v, HalVerdict::kSend) << "a command approved on stale feedback";
    }
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kInhibited));
  // Without fresh samples no stop can be confirmed (C18).
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(this->feedback_.stop[i], StopStatus::kUnconfirmed) << "node " << i;
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kStopUnconfirmed));
  // The device itself still stopped.
  runtime_core::MonoTime last_approval{};
  for (const WriteRecord& w : this->writes_) {
    if (w.verdict == HalVerdict::kSend) last_approval = w.at;
  }
  this->expect_stopped_within_bound(last_approval);
}

TYPED_TEST_P(Contract, SteadyAcceptanceLatency) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  const std::uint64_t superseded_before = this->feedback_.counters.superseded;
  std::vector<runtime_core::MonoTime> approved_at;
  for (int i = 0; i < 30; ++i) {
    const runtime_core::MonoTime t = this->now();
    ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
    approved_at.push_back(t);
  }
  this->step(kPeriod);
  std::vector<std::optional<runtime_core::MonoTime>> first_seen(approved_at.size());
  for (const TraceSample& s : this->trace_) {
    const std::optional<std::uint64_t> seq = s.feedback.group_applied_source_seq;
    if (!seq.has_value() || *seq == 0 || *seq > approved_at.size()) continue;
    std::optional<runtime_core::MonoTime>& seen = first_seen[*seq - 1];
    if (!seen.has_value()) seen = s.at;
  }
  std::size_t seen_count = 0;
  for (std::size_t k = 0; k < approved_at.size(); ++k) {
    if (!first_seen[k].has_value()) continue;
    ++seen_count;
    EXPECT_LE((*first_seen[k] - approved_at[k]).ns,
              runtime_core::Duration{TypeParam::Traits::kAcceptanceLatency}.ns)
        << "seq " << k + 1;
  }
  const std::uint64_t superseded = this->feedback_.counters.superseded - superseded_before;
  ASSERT_GT(seen_count, 0U) << "INCONCLUSIVE: no command was ever reported applied";
  EXPECT_EQ(seen_count + superseded, approved_at.size())
      << "a command was neither applied nor counted as superseded";
}

TYPED_TEST_P(Contract, SupersededCommandsAreCountedNotAcknowledged) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  const std::uint64_t before = this->feedback_.counters.superseded;
  const runtime_core::MotionPermit p =
      permit_for(this->kA, this->now() + runtime_core::milliseconds(200));
  // Two approvals in the same instant: the first can never be dispatched.
  HalSetpoint first = ramp.at(1);
  HalSetpoint second = ramp.at(2);
  for (std::size_t i = 0; i < kJoints; ++i) {
    first.snapshot.position[i] = ramp.base[i] + direction(i) * 0.003;
    second.snapshot.position[i] = ramp.base[i] + direction(i) * 0.006;
  }
  ASSERT_EQ(this->hal_.write(&p, first, this->now()), HalVerdict::kSend);
  ASSERT_EQ(this->hal_.write(&p, second, this->now()), HalVerdict::kSend);
  this->step(plus(TypeParam::Traits::kAcceptanceLatency, kPeriod));
  EXPECT_EQ(this->feedback_.counters.superseded - before, 1U);
  bool second_applied = false;
  for (const TraceSample& s : this->trace_) {
    EXPECT_NE(s.feedback.group_applied_source_seq, std::optional<std::uint64_t>{1})
        << "a superseded command was acknowledged";
    if (s.feedback.group_applied_source_seq == std::optional<std::uint64_t>{2}) {
      second_applied = true;
    }
  }
  for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
    EXPECT_NE(e.source_seq, std::optional<std::uint64_t>{1});
  }
  EXPECT_TRUE(second_applied);
}

TYPED_TEST_P(Contract, SourceTimeNotReceiveTime) {
  constexpr runtime_core::Duration kDelay = runtime_core::milliseconds(6);
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  this->fx_.delay_feedback(kDelay);
  const runtime_core::MonoTime from = this->now() + kDelay + kTBus;
  for (int i = 0; i < 10; ++i) this->ramp_cycle(ramp);
  std::size_t judged = 0;
  for (const TraceSample& s : this->trace_) {
    if (s.at < from) continue;
    ++judged;
    const Feedback& fb = s.feedback;
    EXPECT_GE((s.at - fb.sampled_at).ns, kDelay.ns) << "a receive time reported as source time";
    runtime_core::MonoTime oldest = fb.node_sampled_at[0];
    for (const runtime_core::MonoTime t : fb.node_sampled_at) {
      EXPECT_GE((s.at - t).ns, kDelay.ns);
      oldest = std::min(oldest, t);
    }
    EXPECT_EQ(fb.sampled_at, oldest) << "sampled_at must be the oldest node time";
  }
  ASSERT_GT(judged, 0U) << "INCONCLUSIVE: no snapshot after the delay took effect";
}

TYPED_TEST_P(Contract, ReArmDisablesBeforeCommit) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 15; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
  ASSERT_TRUE(this->moving(this->fx_.device())) << "INCONCLUSIVE: not moving at re-arm";
  const AuthorityId next{this->kA.manager, this->kA.session, runtime_core::Generation{2}};
  this->events_.clear();
  const OpHandle h = this->hal_.arm(
      ArmRequest{next, this->now() + runtime_core::milliseconds(500), 50}, this->now());
  this->settle(h, kArmTimeout);
  ASSERT_EQ(this->hal_.op_state(h), OpState::kSucceeded) << "INCONCLUSIVE: re-arm failed";

  std::array<std::optional<DeviceEvent>, kJoints> disabled{}, committed{}, enabled{};
  for (const DeviceEvent& e : this->events_) {
    std::optional<DeviceEvent>* first = nullptr;
    if (e.kind == DeviceEventKind::kDisabled) first = &disabled[e.node];
    if (e.kind == DeviceEventKind::kCommitted) first = &committed[e.node];
    if (e.kind == DeviceEventKind::kEnabled) first = &enabled[e.node];
    if (first != nullptr && !first->has_value()) *first = e;
  }
  runtime_core::MonoTime last_disable{};
  runtime_core::MonoTime first_commit{std::numeric_limits<std::int64_t>::max()};
  for (std::size_t i = 0; i < kJoints; ++i) {
    ASSERT_TRUE(disabled[i].has_value()) << "node " << i << " was not disabled";
    ASSERT_TRUE(committed[i].has_value()) << "node " << i << " was not committed";
    ASSERT_TRUE(enabled[i].has_value()) << "node " << i << " was not enabled";
    EXPECT_EQ(committed[i]->id, next);
    EXPECT_LT(std::abs(committed[i]->velocity), kRestVelocity) << "committed while moving";
    EXPECT_GE(enabled[i]->at, committed[i]->at);
    last_disable = std::max(last_disable, disabled[i]->at);
    first_commit = std::min(first_commit, committed[i]->at);
    for (const DeviceEvent& e : this->events_) {
      if (e.node == i && e.kind == DeviceEventKind::kActuated && e.at >= disabled[i]->at) {
        EXPECT_GE(e.at, enabled[i]->at) << "node " << i << " actuated while disabled";
        EXPECT_EQ(e.id, next) << "node " << i << " actuated the old identity after the disable";
      }
    }
  }
  // Group phases: every node is disabled before any node commits (C9).
  EXPECT_LE(last_disable, first_commit);
}

TYPED_TEST_P(Contract, InhibitStopsEveryAxisAndConfirms) {
  constexpr std::size_t kNode = 2;
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  Ramp ramp = this->ramp_from_here(this->kA);
  for (int i = 0; i < 15; ++i) ASSERT_EQ(this->ramp_cycle(ramp), HalVerdict::kSend) << i;
  ASSERT_TRUE(this->moving(this->fx_.device())) << "INCONCLUSIVE: not moving at the stop";
  const runtime_core::MonoTime injected = this->now();
  this->fx_.inject_node_stop(kNode);
  while (this->now() < injected + kStopConfirmDeadline + kTBus) {
    const HalVerdict v = this->ramp_cycle(ramp);
    if (this->now() - kPeriod >= injected + kGroupStopLatency) {
      EXPECT_EQ(v, HalVerdict::kInhibited);
    }
  }

  std::array<std::optional<runtime_core::MonoTime>, kJoints> stopped_at{};
  for (const DeviceEvent& e : this->events_) {
    if (e.at < injected) continue;
    if (e.kind == DeviceEventKind::kLocalStop || e.kind == DeviceEventKind::kQuickStop) {
      if (!stopped_at[e.node].has_value()) stopped_at[e.node] = e.at;
    }
  }
  const std::vector<DeviceEvent> local = this->events_of(DeviceEventKind::kLocalStop);
  ASSERT_FALSE(local.empty()) << "INCONCLUSIVE: the injected stop did not happen";
  EXPECT_EQ(local.front().node, kNode);
  for (std::size_t i = 0; i < kJoints; ++i) {
    ASSERT_TRUE(stopped_at[i].has_value()) << "axis " << i << " never got a stop";
    EXPECT_LE((*stopped_at[i] - injected).ns, kGroupStopLatency.ns) << "axis " << i;
    for (const DeviceEvent& e : this->events_of(DeviceEventKind::kActuated)) {
      if (e.node == i) {
        EXPECT_LT(e.at, *stopped_at[i]) << "axis " << i << " actuated after its stop";
      }
    }
    EXPECT_EQ(this->feedback_.stop[i], StopStatus::kConfirmed) << "axis " << i;
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kInhibited));
  EXPECT_FALSE(this->feedback_.faults.has(Fault::kStopUnconfirmed));
  this->expect_stopped_within_bound(injected);
}

// C18: a stop is confirmed only by 5 distinct fresh samples. With the axes already at rest and
// feedback stalled, the last sample says "at rest" forever; counting it again must never
// confirm anything.
TYPED_TEST_P(Contract, StaleAtRestSampleConfirmsNothing) {
  this->bring_up();
  this->arm(this->kA, runtime_core::milliseconds(500));
  const HalSetpoint sp = Ramp{this->kA, kStartPosition, 0}.at(1);
  for (int i = 0; i < 5; ++i) this->cycle(nullptr, sp);
  for (std::size_t i = 0; i < kJoints; ++i) {
    ASSERT_LT(std::abs(this->fx_.device().velocity[i]), kRestVelocity)
        << "INCONCLUSIVE: joint " << i << " not at rest before the stall";
    ASSERT_LT(std::abs(this->feedback_.velocity[i]), kRestVelocity)
        << "INCONCLUSIVE: the last sample of joint " << i << " is not at rest";
  }
  const runtime_core::MonoTime stalled = this->now();
  this->fx_.stall_feedback(true);
  this->fx_.inject_node_stop(0);
  while (this->now() < stalled + kMaxFeedbackAge + kStopConfirmDeadline + kTBus) {
    this->cycle(nullptr, sp);
  }
  for (const TraceSample& s : this->trace_) {
    if (s.at < stalled) continue;
    for (std::size_t i = 0; i < kJoints; ++i) {
      EXPECT_NE(s.feedback.stop[i], StopStatus::kConfirmed)
          << "node " << i << " confirmed from a stale sample at +" << (s.at - stalled).ns << " ns";
    }
  }
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kInhibited));
  EXPECT_TRUE(this->feedback_.faults.has(Fault::kStopUnconfirmed));
  for (std::size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(this->feedback_.stop[i], StopStatus::kUnconfirmed) << "node " << i;
  }
}

REGISTER_TYPED_TEST_SUITE_P(Contract, NoCommandBeforeActivation,
                            ArmHoldsActualPositionWithoutPermit, ArmHoldEndsAtHoldUntil,
                            TracksApprovedCommandsWithinVmax, ExpiredPermitStopsWithinBound,
                            RepeatedSnapshotDoesNotExtendLife, OldIdentityRefusedAfterReadoption,
                            LostFeedbackTurnsStale, SteadyAcceptanceLatency,
                            SupersededCommandsAreCountedNotAcknowledged, SourceTimeNotReceiveTime,
                            ReArmDisablesBeforeCommit, InhibitStopsEveryAxisAndConfirms,
                            StaleAtRestSampleConfirmsNothing);

}  // namespace hal_contract::testing
