// SPDX-License-Identifier: Apache-2.0
// DirectBackend on its own, without the Hal: the drive's stop semantics and its capability
// profile. The contract suite (test_contract_direct.cpp) covers the same backend through the Hal.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "direct_backend/direct_backend.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

namespace db = direct_backend;
namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

constexpr rc::Duration kW = db::kStep;
constexpr double kDvPerStep = 10.0 * 0.002;  // a_stop * W, rad/s

constexpr hc::AuthorityId kId{rc::ManagerInstance{1}, rc::SessionId{1}, rc::Generation{1}};

struct Rig {
  Rig() : backend(config()) {
    backend.set_event_sink(&Rig::collect, this);
    backend.configure({++ids, t + rc::milliseconds(3000)}, t);
    backend.activate({++ids, t + rc::milliseconds(1000)}, t);
    backend.arm({++ids, t + rc::milliseconds(1000)},
                hc::ArmRequest{kId, t + rc::milliseconds(1000), 1}, t);
    tick();
  }

  static db::DirectConfig config() {
    db::DirectConfig c{};
    c.start_position = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
    return c;
  }
  static void collect(void* self, const db::Event& e) {
    static_cast<Rig*>(self)->events.push_back(e);
  }
  void tick() {
    t = t + kW;
    backend.step(t);
  }
  // Approves a far target on every joint, so the joints run at v_max.
  void command(std::uint64_t seq, rc::MonoTime valid_until) {
    hc::ApprovedCommand c{};
    c.id = kId;
    c.source_seq = seq;
    for (std::size_t i = 0; i < hc::kJoints; ++i) c.position[i] = backend.joint_position()[i] + 1.0;
    c.approved_at = t;
    c.valid_until = valid_until;
    backend.submit(c, t);
  }
  std::optional<db::Event> first(db::EventKind kind) const {
    for (const db::Event& e : events) {
      if (e.kind == kind) return e;
    }
    return std::nullopt;
  }

  rc::MonoTime t{1'000'000'000};
  std::uint64_t ids{0};
  db::DirectBackend backend;
  std::vector<db::Event> events;
};

}  // namespace

TEST(DirectBackend, ExpiredCommandBrakesAndLatches) {
  Rig rig;
  ASSERT_EQ(rig.backend.op_state(3), hc::OpState::kSucceeded) << "INCONCLUSIVE: arm did not finish";
  const rc::MonoTime valid_until = rig.t + rc::milliseconds(20);
  rig.command(1, valid_until);
  // Tracks while open: the joints reach v_max.
  while (rig.t + kW < valid_until) rig.tick();
  ASSERT_FALSE(rig.backend.inhibited());
  ASSERT_NEAR(std::abs(rig.backend.joint_velocity()[0]), 1.0, 1e-9)
      << "INCONCLUSIVE: the joint was not at v_max";
  const double before = std::abs(rig.backend.joint_velocity()[0]);

  rig.tick();  // t == valid_until: expired, equality is expired
  ASSERT_EQ(rig.t, valid_until);
  const std::optional<db::Event> stop = rig.first(db::EventKind::kLocalStop);
  ASSERT_TRUE(stop.has_value());
  EXPECT_EQ(stop->at, valid_until);
  // Braking at a_stop: one a_stop * W of velocity per step, on every joint.
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    EXPECT_NEAR(std::abs(rig.backend.joint_velocity()[i]), before - kDvPerStep, 1e-9) << i;
  }
  rig.tick();
  EXPECT_NEAR(std::abs(rig.backend.joint_velocity()[0]), before - 2 * kDvPerStep, 1e-9);

  const hc::Feedback fb = rig.backend.poll(rig.t);
  EXPECT_TRUE(fb.faults.has(hc::Fault::kPolicyStop));
  EXPECT_TRUE(fb.faults.has(hc::Fault::kInhibited));
  EXPECT_TRUE(rig.backend.inhibited());
}

// A documented divergence from the vdrive, which E4 settles: here a new configure clears a
// latched policy stop (the joints are reset), while a drive_core drive keeps its D11 policy latch
// until an explicit policy reset (0x2015, drive_core Latch.PolicyResetOverSdo), so on CANopen
// every contract case that stops the device starts from a fresh fixture.
TEST(DirectBackend, ConfigureClearsALatchedPolicyStop) {
  Rig rig;
  rig.command(1, rig.t + rc::milliseconds(20));
  for (int i = 0; i < 100; ++i) rig.tick();  // expired, braked to rest
  ASSERT_TRUE(rig.backend.poll(rig.t).faults.has(hc::Fault::kPolicyStop)) << "INCONCLUSIVE";
  ASSERT_TRUE(rig.backend.inhibited()) << "INCONCLUSIVE";
  rig.backend.configure({++rig.ids, rig.t + rc::milliseconds(3000)}, rig.t);
  rig.tick();
  EXPECT_EQ(rig.backend.op_state(rig.ids), hc::OpState::kSucceeded);
  EXPECT_FALSE(rig.backend.inhibited());
  EXPECT_FALSE(rig.backend.poll(rig.t).faults.has(hc::Fault::kPolicyStop));
  rig.backend.activate({++rig.ids, rig.t + rc::milliseconds(1000)}, rig.t);
  rig.tick();
  EXPECT_EQ(rig.backend.op_state(rig.ids), hc::OpState::kSucceeded);
}

TEST(DirectBackend, NoCommandFor30msBrakes) {
  Rig rig;
  rig.command(1, rig.t + rc::milliseconds(500));  // valid for long: only the silence stops it
  rig.tick();
  const rc::MonoTime taken = rig.t;  // the step that took the command
  while (rig.t + kW < taken + db::kCommandTimeout) {
    rig.tick();
    ASSERT_FALSE(rig.first(db::EventKind::kLocalStop).has_value()) << "stopped early";
  }
  rig.tick();
  ASSERT_EQ(rig.t, taken + db::kCommandTimeout);
  const std::optional<db::Event> stop = rig.first(db::EventKind::kLocalStop);
  ASSERT_TRUE(stop.has_value());
  EXPECT_EQ(stop->at, taken + db::kCommandTimeout);
  EXPECT_TRUE(rig.backend.poll(rig.t).faults.has(hc::Fault::kPolicyStop));
  EXPECT_TRUE(rig.backend.inhibited());
}

TEST(DirectBackend, CapabilitiesAreDirectProfile) {
  db::DirectBackend backend(Rig::config());
  backend.configure({1, rc::MonoTime{} + rc::milliseconds(3000)}, rc::MonoTime{});
  const rc::DeviceCapabilities got = backend.capabilities();
  const rc::DeviceCapabilities want = rc::direct_backend_profile();
  EXPECT_EQ(got.device_side_expiry, want.device_side_expiry);
  EXPECT_EQ(got.execution_generation, want.execution_generation);
  EXPECT_EQ(got.local_stop, want.local_stop);
  EXPECT_EQ(got.apply_ack, want.apply_ack);
  EXPECT_EQ(got.source_timestamp, want.source_timestamp);
  EXPECT_TRUE(rc::missing_capabilities({true, true, true, true, true}, got).empty());
}

TEST(DirectBackend, ConfigureRejectsFeedbackAgeOutsideTheDriveProfile) {
  for (const std::int64_t ms : {19, 31}) {
    db::DirectConfig c = Rig::config();
    c.max_feedback_age = rc::milliseconds(ms);
    db::DirectBackend backend(c);
    backend.configure({1, rc::MonoTime{} + rc::milliseconds(3000)}, rc::MonoTime{});
    EXPECT_EQ(backend.op_state(1), hc::OpState::kFailed) << ms;
    const std::optional<hc::OpResult> r = backend.wait(1, rc::Duration{});
    ASSERT_TRUE(r.has_value());
    EXPECT_NE(r->error.find("[20, 30] ms"), std::string::npos) << r->error;
  }
  for (const std::int64_t ms : {20, 30}) {
    db::DirectConfig c = Rig::config();
    c.max_feedback_age = rc::milliseconds(ms);
    db::DirectBackend backend(c);
    backend.configure({1, rc::MonoTime{} + rc::milliseconds(3000)}, rc::MonoTime{});
    EXPECT_EQ(backend.op_state(1), hc::OpState::kSucceeded) << ms;
  }
}

TEST(DirectBackend, UnavailableDriveIsUnconfirmedWhateverItsVelocity) {
  Rig rig;
  rig.backend.set_drive_available(3, false);
  rig.backend.inject_stop(0);
  for (int i = 0; i < 20; ++i) rig.tick();  // 40 ms: all joints at rest long since
  const hc::Feedback fb = rig.backend.poll(rig.t);
  ASSERT_TRUE(fb.faults.has(hc::Fault::kInhibited));
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    ASSERT_LT(std::abs(fb.velocity[i]), db::kRestVelocity) << "INCONCLUSIVE: joint " << i;
    if (i == 3) {
      EXPECT_EQ(fb.stop[i], hc::StopStatus::kUnconfirmed);
    } else {
      EXPECT_EQ(fb.stop[i], hc::StopStatus::kConfirmed) << i;
    }
  }
  EXPECT_TRUE(fb.faults.has(hc::Fault::kStopUnconfirmed));
}

TEST(DirectBackend, StartPositionIsExactAndOutOfRangeIsRejected) {
  db::DirectBackend backend(Rig::config());
  const hc::Joints p = backend.joint_position();
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    EXPECT_EQ(p[i], Rig::config().start_position[i]) << i;
    EXPECT_EQ(backend.joint_velocity()[i], 0.0) << i;
  }
  db::DirectConfig bad = Rig::config();
  bad.start_position[2] = 1e3;
  EXPECT_THROW(db::DirectBackend{bad}, std::invalid_argument);
  bad.start_position[2] = std::nan("");
  EXPECT_THROW(db::DirectBackend{bad}, std::invalid_argument);
}
