// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hal_contract/backend.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"

namespace hc = hal_contract;
namespace rc = runtime_core;
using C = rc::Capability;
using S = rc::Support;
using V = hc::HalVerdict;
using rc::milliseconds;

// Counts heap allocations and frees while armed, so a test can prove a path allocation-free.
// Replacing the global operators applies to the whole test binary; counting is off by default.
namespace {
std::atomic<bool> g_counting{false};
std::atomic<int> g_allocations{0};
std::atomic<int> g_frees{0};

void* counted_alloc(std::size_t n) {
  if (g_counting.load()) ++g_allocations;
  if (void* p = std::malloc(n == 0 ? 1 : n)) return p;
  throw std::bad_alloc();
}
void counted_free(void* p) noexcept {
  if (p != nullptr && g_counting.load()) ++g_frees;
  std::free(p);
}
}  // namespace

void* operator new(std::size_t n) { return counted_alloc(n); }
void* operator new[](std::size_t n) { return counted_alloc(n); }
// The nothrow forms too: gtest uses them, and every pointer that reaches the replaced delete
// must come from malloc (a sanitizer's own new paired with free is an alloc-dealloc mismatch).
void* operator new(std::size_t n, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return counted_alloc(n);
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
}
void* operator new[](std::size_t n, const std::nothrow_t& tag) noexcept {
  return operator new(n, tag);
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept { counted_free(p); }
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept { counted_free(p); }
void operator delete(void* p) noexcept { counted_free(p); }
void operator delete[](void* p) noexcept { counted_free(p); }
void operator delete(void* p, std::size_t /*n*/) noexcept { counted_free(p); }
void operator delete[](void* p, std::size_t /*n*/) noexcept { counted_free(p); }

namespace {

// Runs `f` and returns how many allocations plus frees it made.
template <class Fn>
int heap_events(Fn&& f) {
  g_allocations = 0;
  g_frees = 0;
  g_counting = true;
  f();
  g_counting = false;
  return g_allocations.load() + g_frees.load();
}

// Everything the spy saw, kept outside the backend so the test can read it after the Hal took
// ownership of the backend.
struct SpyLog {
  rc::DeviceCapabilities capabilities{rc::virtual_canopen_profile()};
  bool complete_ops{true};  // ops succeed when posted; false leaves them pending
  int configure_calls{0};
  int activate_calls{0};
  int arm_calls{0};
  int on_error_calls{0};
  // Fixed storage, so the spy itself never allocates inside a counted region.
  std::vector<hc::ApprovedCommand> submits = [] {
    std::vector<hc::ApprovedCommand> v;
    v.reserve(256);
    return v;
  }();
  std::array<hc::OpState, 256> ops = [] {
    std::array<hc::OpState, 256> a{};
    a.fill(hc::OpState::kUnknown);
    return a;
  }();
  hc::Feedback feedback{};
  bool inhibited{false};
  rc::Duration apply_margin{};  // can_apply: valid_until > now + apply_margin
};

class SpyBackend final : public hc::Backend {
 public:
  explicit SpyBackend(SpyLog* log) : log_(log) {}

  void configure(const hc::OpTicket& t, rc::MonoTime /*now*/) override {
    ++log_->configure_calls;
    post(t);
  }
  rc::DeviceCapabilities capabilities() const override { return log_->capabilities; }
  void activate(const hc::OpTicket& t, rc::MonoTime /*now*/) override {
    ++log_->activate_calls;
    post(t);
  }
  void arm(const hc::OpTicket& t, const hc::ArmRequest& /*r*/, rc::MonoTime /*now*/) override {
    ++log_->arm_calls;
    post(t);
  }
  void submit(const hc::ApprovedCommand& c, rc::MonoTime /*now*/) override {
    log_->submits.push_back(c);
  }
  hc::Feedback poll(rc::MonoTime /*now*/) override { return log_->feedback; }
  void deactivate(const hc::OpTicket& t, rc::MonoTime /*now*/) override { post(t); }
  void on_error_nonblocking() override {
    ++log_->on_error_calls;
    log_->inhibited = true;
  }
  hc::OpState op_state(std::uint64_t id) const override {
    return id < log_->ops.size() ? log_->ops[static_cast<std::size_t>(id)] : hc::OpState::kUnknown;
  }
  std::optional<hc::OpResult> wait(std::uint64_t id, rc::Duration /*timeout*/) override {
    const hc::OpState s = op_state(id);
    if (s == hc::OpState::kPending) return std::nullopt;
    return hc::OpResult{s == hc::OpState::kSucceeded, {}, {}};
  }
  bool inhibited() const override { return log_->inhibited; }
  bool can_apply(rc::MonoTime valid_until, rc::MonoTime now) const override {
    return valid_until > now + log_->apply_margin;
  }

 private:
  void post(const hc::OpTicket& t) {
    log_->ops.at(static_cast<std::size_t>(t.op_id)) =
        log_->complete_ops ? hc::OpState::kSucceeded : hc::OpState::kPending;
  }
  SpyLog* log_;
};

constexpr rc::MonoTime kT0{1'000'000'000};
const hc::AuthorityId kId{rc::ManagerInstance{1}, rc::SessionId{1}, rc::Generation{1}};

rc::Requirements all_required() { return {true, true, true, true, true}; }

rc::JointLimits<hc::kJoints> limits() {
  rc::JointLimits<hc::kJoints> l{};
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    l.pos_min[i] = -3.0;
    l.pos_max[i] = 3.0;
    l.vel_max[i] = 1.0;
    l.acc_max[i] = 100.0;
  }
  return l;
}

const rc::GuardConfig kGuard{milliseconds(10), milliseconds(20)};

rc::MotionPermit permit_for(const hc::AuthorityId& id, rc::MonoTime expires_at) {
  return rc::MotionPermit{id.manager, id.session, id.generation, expires_at};
}

hc::HalSetpoint setpoint(const hc::AuthorityId& id, std::uint64_t seq, double q = 0.005) {
  hc::HalSetpoint sp{};
  sp.id = id;
  sp.snapshot.generation = id.generation;
  sp.snapshot.source_seq = seq;
  sp.snapshot.position.fill(q);
  return sp;
}

struct Rig {
  SpyLog log;
  hc::Hal hal;
  rc::MonoTime now{kT0};

  explicit Rig(rc::Requirements req = all_required())
      : hal(std::make_unique<SpyBackend>(&log), req, kGuard, limits()) {
    log.feedback.sampled_at = kT0;
  }

  // configure -> activate -> arm(kId) -> one read of fresh feedback.
  void bring_up(const hc::AuthorityId& id = kId) {
    ASSERT_EQ(hal.op_state(hal.configure(now)), hc::OpState::kSucceeded);
    ASSERT_EQ(hal.op_state(hal.activate(now)), hc::OpState::kSucceeded);
    ASSERT_EQ(hal.op_state(hal.arm(hc::ArmRequest{id, now + milliseconds(500), 7}, now)),
              hc::OpState::kSucceeded);
    fresh_read();
  }
  void fresh_read() {
    log.feedback.sampled_at = now;
    (void)hal.read(now);
  }
  void tick(rc::Duration d = milliseconds(10)) {
    now = now + d;
    fresh_read();
  }
  V write(const rc::MotionPermit* p, const hc::HalSetpoint& sp) { return hal.write(p, sp, now); }
};

}  // namespace

TEST(Hal, ActivationRefusedListsMissing) {
  Rig r;
  r.log.capabilities = rc::generic_cia402_profile();
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  const hc::OpHandle h = r.hal.activate(r.now);
  EXPECT_EQ(r.hal.op_state(h), hc::OpState::kFailed);
  const std::optional<hc::OpResult> result = r.hal.wait(h, milliseconds(0));
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ok);
  const std::vector<rc::Missing> expected{{C::kDeviceSideExpiry, S::kUnsupported},
                                          {C::kExecutionGeneration, S::kUnsupported},
                                          {C::kLocalStop, S::kUnknown},
                                          {C::kApplyAck, S::kUnsupported},
                                          {C::kSourceTimestamp, S::kUnknown}};
  EXPECT_EQ(result->missing, expected);
  EXPECT_NE(result->error.find("LOCAL_STOP=UNKNOWN"), std::string::npos) << result->error;
  EXPECT_NE(result->error.find("APPLY_ACK=UNSUPPORTED"), std::string::npos) << result->error;
  EXPECT_EQ(r.log.activate_calls, 0);
  // Refused activation leaves the Hal inactive.
  EXPECT_EQ(r.write(nullptr, setpoint(kId, 1)), V::kHardwareNotActive);
}

TEST(Hal, ActivationAllowedWhenOnlyUnsupportedOnesAreNotRequired) {
  rc::Requirements req{};
  req.device_side_expiry = true;  // kProvidedByHost
  req.local_stop = true;          // kSupported
  Rig r(req);
  r.log.capabilities = {S::kProvidedByHost, S::kUnsupported, S::kSupported, S::kUnknown,
                        S::kUnsupported};
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  const hc::OpHandle h = r.hal.activate(r.now);
  EXPECT_EQ(r.hal.op_state(h), hc::OpState::kSucceeded);
  EXPECT_EQ(r.log.activate_calls, 1);

  // The same profile with one unmet capability required is refused, naming only that one.
  Rig refused(rc::Requirements{false, true, false, false, false});
  refused.log.capabilities = r.log.capabilities;
  ASSERT_EQ(refused.hal.op_state(refused.hal.configure(refused.now)), hc::OpState::kSucceeded);
  const std::optional<hc::OpResult> res =
      refused.hal.wait(refused.hal.activate(refused.now), milliseconds(0));
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->missing, (std::vector<rc::Missing>{{C::kExecutionGeneration, S::kUnsupported}}));
  EXPECT_EQ(refused.log.activate_calls, 0);
}

TEST(Hal, AdoptsIdentityOnlyWhenArmCompletes) {
  Rig r;
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  ASSERT_EQ(r.hal.op_state(r.hal.activate(r.now)), hc::OpState::kSucceeded);
  r.log.complete_ops = false;
  const hc::OpHandle h = r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(500), 9}, r.now);
  r.fresh_read();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kArming);
  EXPECT_FALSE(r.hal.adopted().has_value());
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kNotArmed);
  EXPECT_TRUE(r.log.submits.empty());

  r.log.ops.at(static_cast<std::size_t>(h.op_id)) = hc::OpState::kSucceeded;
  EXPECT_EQ(r.hal.op_state(h), hc::OpState::kSucceeded);
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kArmed);
  ASSERT_TRUE(r.hal.adopted().has_value());
  EXPECT_EQ(*r.hal.adopted(), kId);
  EXPECT_EQ(r.hal.armed_request_id(), 9U);
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kSend);

  // A failed re-arm leaves nothing adopted.
  const hc::AuthorityId b{rc::ManagerInstance{1}, rc::SessionId{1}, rc::Generation{2}};
  const hc::OpHandle h2 = r.hal.arm(hc::ArmRequest{b, r.now + milliseconds(500), 10}, r.now);
  r.log.ops.at(static_cast<std::size_t>(h2.op_id)) = hc::OpState::kFailed;
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kFailed);
  EXPECT_FALSE(r.hal.adopted().has_value());
  EXPECT_EQ(r.write(&p, setpoint(kId, 2)), V::kNotArmed);
}

TEST(Hal, PermitIdentityMismatchRefused) {
  const std::vector<hc::AuthorityId> others{
      {rc::ManagerInstance{2}, kId.session, kId.generation},
      {kId.manager, rc::SessionId{2}, kId.generation},
      {kId.manager, kId.session, rc::Generation{2}},
  };
  for (const hc::AuthorityId& other : others) {
    Rig r;
    r.bring_up();
    const rc::MotionPermit p = permit_for(other, r.now + milliseconds(200));
    EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kIdentityMismatch);
    EXPECT_TRUE(r.log.submits.empty());
  }
}

TEST(Hal, SetpointIdentityMismatchRefused) {
  const std::vector<hc::AuthorityId> others{
      {rc::ManagerInstance{2}, kId.session, kId.generation},
      {kId.manager, rc::SessionId{2}, kId.generation},
      {kId.manager, kId.session, rc::Generation{2}},
  };
  for (const hc::AuthorityId& other : others) {
    Rig r;
    r.bring_up();
    const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
    EXPECT_EQ(r.write(&p, setpoint(other, 1)), V::kIdentityMismatch);
    EXPECT_TRUE(r.log.submits.empty());
  }
  // The snapshot's own generation must match its identity.
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
  hc::HalSetpoint sp = setpoint(kId, 1);
  sp.snapshot.generation = rc::Generation{2};
  EXPECT_EQ(r.write(&p, sp), V::kIdentityMismatch);
  EXPECT_TRUE(r.log.submits.empty());
}

TEST(Hal, DuplicateKeepsOriginalDeadline) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
  ASSERT_EQ(r.write(&p, setpoint(kId, 5)), V::kSend);
  const rc::MonoTime original = r.now + milliseconds(48);
  for (int i = 0; i < 4; ++i) {
    r.tick();
    EXPECT_EQ(r.write(&p, setpoint(kId, 5)), V::kDuplicate);
  }
  ASSERT_EQ(r.log.submits.size(), 1U);
  EXPECT_EQ(r.log.submits[0].valid_until, original);
  // The next seq is measured against the original command and goes out.
  r.tick();
  EXPECT_EQ(r.write(&p, setpoint(kId, 6, 0.01)), V::kSend);
  ASSERT_EQ(r.log.submits.size(), 2U);
  EXPECT_EQ(r.log.submits[1].valid_until, r.now + milliseconds(48));
}

TEST(Hal, DuplicateWithChangedPositionRefused) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
  ASSERT_EQ(r.write(&p, setpoint(kId, 5)), V::kSend);
  r.tick();
  EXPECT_EQ(r.write(&p, setpoint(kId, 5, 0.006)), V::kDuplicateMismatch);
  ASSERT_EQ(r.log.submits.size(), 1U);
  EXPECT_EQ(r.log.submits[0].position[0], 0.005);
}

TEST(Hal, RegressionRefused) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
  ASSERT_EQ(r.write(&p, setpoint(kId, 5)), V::kSend);
  r.tick();
  EXPECT_EQ(r.write(&p, setpoint(kId, 4)), V::kRegression);
  EXPECT_EQ(r.write(&p, setpoint(kId, 0)), V::kRegression);
  EXPECT_EQ(r.log.submits.size(), 1U);
}

TEST(Hal, InhibitRefusesEveryWrite) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
  r.log.inhibited = true;
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kInhibited);
  EXPECT_EQ(r.write(nullptr, setpoint(kId, 2)), V::kInhibited);
  const rc::MotionPermit other =
      permit_for({rc::ManagerInstance{9}, kId.session, kId.generation}, r.now + milliseconds(500));
  EXPECT_EQ(r.write(&other, setpoint(kId, 3)), V::kInhibited);
  r.tick();
  EXPECT_EQ(r.write(&p, setpoint(kId, 4)), V::kInhibited);
  EXPECT_TRUE(r.log.submits.empty());
}

TEST(Hal, WriteSubmitsOnlyOnSend) {
  // Each case brings a fresh Hal to the refusal and expects no submit.
  // The guard's own manager/session/generation verdicts cannot be reached through Hal: identity is
  // refused first as kIdentityMismatch.
  struct Case {
    V expected;
    std::function<V(Rig&)> run;
  };
  const auto armed_permit = [](Rig& r) { return permit_for(kId, r.now + milliseconds(500)); };
  const std::vector<Case> cases{
      {V::kHardwareNotActive,
       [](Rig& r) {
         const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kNotArmed,
       [](Rig& r) {
         (void)r.hal.configure(r.now);
         (void)r.hal.activate(r.now);
         const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kNoPermit,
       [](Rig& r) {
         r.bring_up();
         return r.write(nullptr, setpoint(kId, 1));
       }},
      {V::kPermitExpired,
       [](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = permit_for(kId, r.now);
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kIdentityMismatch,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint({kId.manager, rc::SessionId{3}, kId.generation}, 1));
       }},
      {V::kFeedbackStale,
       [&](Rig& r) {
         r.bring_up();
         r.now = r.now + milliseconds(21);
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kFeedbackStale,  // skewed feedback is stale whatever its time says
       [&](Rig& r) {
         r.bring_up();
         r.log.feedback.skewed = true;
         r.fresh_read();
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kNonFinite,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1, std::numeric_limits<double>::quiet_NaN()));
       }},
      {V::kPositionLimit,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1, 3.5));
       }},
      {V::kVelocityLimit,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1, 0.02));  // 2 rad/s over one 10 ms period
       }},
      {V::kAccelerationLimit,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         if (r.write(&p, setpoint(kId, 1, 0.009)) != V::kSend) return V::kSend;
         r.log.submits.clear();
         r.tick();
         // 0.9 rad/s to -0.2 rad/s in 10 ms: 110 rad/s^2 against a 100 rad/s^2 limit.
         return r.write(&p, setpoint(kId, 2, 0.007));
       }},
      {V::kDuplicate,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         if (r.write(&p, setpoint(kId, 1)) != V::kSend) return V::kSend;
         r.log.submits.clear();
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kDuplicateMismatch,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         if (r.write(&p, setpoint(kId, 1)) != V::kSend) return V::kSend;
         r.log.submits.clear();
         return r.write(&p, setpoint(kId, 1, 0.004));
       }},
      {V::kRegression,
       [&](Rig& r) {
         r.bring_up();
         const rc::MotionPermit p = armed_permit(r);
         if (r.write(&p, setpoint(kId, 2)) != V::kSend) return V::kSend;
         r.log.submits.clear();
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kInhibited,
       [&](Rig& r) {
         r.bring_up();
         r.log.inhibited = true;
         const rc::MotionPermit p = armed_permit(r);
         return r.write(&p, setpoint(kId, 1));
       }},
      {V::kExpiresBeforeApplication,
       [&](Rig& r) {
         r.bring_up();
         r.log.apply_margin = milliseconds(13);
         const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(12));
         return r.write(&p, setpoint(kId, 1));
       }},
  };
  for (const Case& c : cases) {
    Rig r;
    const V got = c.run(r);
    EXPECT_EQ(got, c.expected) << "expected " << hc::to_string(c.expected) << ", got "
                               << hc::to_string(got);
    EXPECT_TRUE(r.log.submits.empty()) << hc::to_string(c.expected);
  }
}

TEST(Hal, ValidUntilIsMinOfPermitAndLease) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit short_permit = permit_for(kId, r.now + milliseconds(30));
  ASSERT_EQ(r.write(&short_permit, setpoint(kId, 1)), V::kSend);
  ASSERT_EQ(r.log.submits.size(), 1U);
  EXPECT_EQ(r.log.submits[0].valid_until, r.now + milliseconds(30));
  EXPECT_EQ(r.log.submits[0].approved_at, r.now);

  r.tick();
  const rc::MotionPermit long_permit = permit_for(kId, r.now + milliseconds(200));
  ASSERT_EQ(r.write(&long_permit, setpoint(kId, 2, 0.01)), V::kSend);
  ASSERT_EQ(r.log.submits.size(), 2U);
  EXPECT_EQ(r.log.submits[1].valid_until, r.now + milliseconds(48));
  EXPECT_EQ(r.log.submits[1].id, kId);
  EXPECT_EQ(r.log.submits[1].source_seq, 2U);
}

TEST(Hal, PolicyRefusalIsNotAFault) {
  Rig r;
  r.bring_up();
  const rc::MotionPermit expired = permit_for(kId, r.now);
  EXPECT_EQ(r.write(&expired, setpoint(kId, 1)), V::kPermitExpired);
  EXPECT_EQ(r.write(nullptr, setpoint(kId, 1)), V::kNoPermit);
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
  EXPECT_EQ(r.write(&p, setpoint(kId, 1, 0.5)), V::kVelocityLimit);
  EXPECT_EQ(r.log.on_error_calls, 0);
  EXPECT_FALSE(r.log.inhibited);
  r.tick();
  const hc::Feedback fb = r.hal.read(r.now);
  EXPECT_FALSE(fb.faults.any());
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kArmed);
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kSend);
}

TEST(Hal, RejectsInvalidConstruction) {
  SpyLog log;
  EXPECT_THROW(hc::Hal(nullptr, all_required(), kGuard, limits()), std::invalid_argument);
  EXPECT_THROW(hc::Hal(std::make_unique<SpyBackend>(&log), all_required(), kGuard, limits(),
                       milliseconds(49)),
               std::invalid_argument);
  EXPECT_THROW(hc::Hal(std::make_unique<SpyBackend>(&log), all_required(), kGuard, limits(),
                       rc::Duration{0}),
               std::invalid_argument);
}

TEST(Hal, ArmRefusedOutsideHoldWindowOrWhenInactive) {
  Rig r;
  const hc::OpHandle inactive = r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(100), 1}, r.now);
  EXPECT_EQ(r.hal.op_state(inactive), hc::OpState::kFailed);
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  ASSERT_EQ(r.hal.op_state(r.hal.activate(r.now)), hc::OpState::kSucceeded);
  EXPECT_EQ(r.hal.op_state(r.hal.arm(hc::ArmRequest{kId, r.now, 1}, r.now)), hc::OpState::kFailed);
  EXPECT_EQ(r.hal.op_state(r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(1001), 1}, r.now)),
            hc::OpState::kFailed);
  EXPECT_EQ(r.log.arm_calls, 0);
  EXPECT_EQ(r.hal.op_state(r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(1000), 1}, r.now)),
            hc::OpState::kSucceeded);
  EXPECT_EQ(r.log.arm_calls, 1);
  EXPECT_EQ(r.hal.op_state(hc::OpHandle{999}), hc::OpState::kUnknown);
}

TEST(Hal, ArmAndWritePathsDoNotAllocate) {
  // Task 9 posts arm ops from write(): every arm outcome and every write must stay off the heap
  // once active, including the ring of op slots wrapping over earlier refusals.
  Rig r;
  r.bring_up();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(500));
  int events = heap_events([&] { (void)r.write(&p, setpoint(kId, 1)); });
  EXPECT_EQ(events, 0) << "write(kSend)";
  events = heap_events([&] { (void)r.write(&p, setpoint(kId, 1)); });
  EXPECT_EQ(events, 0) << "write(kDuplicate)";
  events = heap_events([&] { (void)r.write(nullptr, setpoint(kId, 2)); });
  EXPECT_EQ(events, 0) << "write(kNoPermit)";
  events = heap_events([&] { (void)r.hal.read(r.now); });
  EXPECT_EQ(events, 0) << "read";
  for (int i = 0; i < 3 * static_cast<int>(hc::Hal::kOpSlots); ++i) {
    events = heap_events([&] {
      // Refused: hold_until outside (now, now + 1 s].
      const hc::OpHandle bad = r.hal.arm(hc::ArmRequest{kId, r.now, 1}, r.now);
      (void)r.hal.op_state(bad);
      // Accepted: posted to the backend, then settled.
      const hc::OpHandle good = r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(500), 2}, r.now);
      (void)r.hal.op_state(good);
    });
    EXPECT_EQ(events, 0) << "arm round " << i;
  }
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kArmed);
  // The refusal still reads back with its reason on the lifecycle path.
  const hc::OpHandle bad = r.hal.arm(hc::ArmRequest{kId, r.now, 1}, r.now);
  const std::optional<hc::OpResult> res = r.hal.wait(bad, milliseconds(0));
  ASSERT_TRUE(res.has_value());
  EXPECT_FALSE(res->ok);
  EXPECT_NE(res->error.find("hold_until"), std::string::npos) << res->error;
}

TEST(Hal, EvictedPendingArmSettlesAsFailed) {
  Rig r;
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  ASSERT_EQ(r.hal.op_state(r.hal.activate(r.now)), hc::OpState::kSucceeded);
  r.log.complete_ops = false;
  const hc::OpHandle h = r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(500), 3}, r.now);
  ASSERT_EQ(r.hal.arm_state(), hc::ArmState::kArming);
  // Enough local refusals to reuse every ring slot, the pending arm's included.
  for (std::size_t i = 0; i < hc::Hal::kOpSlots; ++i) {
    (void)r.hal.arm(hc::ArmRequest{kId, r.now, 1}, r.now);
  }
  EXPECT_EQ(r.hal.op_state(h), hc::OpState::kUnknown);
  EXPECT_EQ(r.hal.arm_state(), hc::ArmState::kFailed) << "an evicted arm must not stay arming";
  // The backend finishing it later adopts nothing.
  r.log.ops.at(static_cast<std::size_t>(h.op_id)) = hc::OpState::kSucceeded;
  EXPECT_FALSE(r.hal.adopted().has_value());
  r.fresh_read();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kNotArmed);
}

TEST(Hal, OpsPostedBeforeDeactivateOrConfigureDoNotSettle) {
  Rig r;
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  r.log.complete_ops = false;
  const hc::OpHandle act = r.hal.activate(r.now);
  (void)r.hal.deactivate(r.now);
  r.log.ops.at(static_cast<std::size_t>(act.op_id)) = hc::OpState::kSucceeded;  // late success
  r.fresh_read();
  const rc::MotionPermit p = permit_for(kId, r.now + milliseconds(200));
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kHardwareNotActive);
  EXPECT_EQ(r.hal.op_state(r.hal.arm(hc::ArmRequest{kId, r.now + milliseconds(500), 4}, r.now)),
            hc::OpState::kFailed);

  // The same after a new configure: an activate posted before it settles nothing.
  r.log.complete_ops = true;
  ASSERT_EQ(r.hal.op_state(r.hal.configure(r.now)), hc::OpState::kSucceeded);
  r.log.complete_ops = false;
  const hc::OpHandle act2 = r.hal.activate(r.now);
  const hc::OpHandle cfg = r.hal.configure(r.now);
  r.log.ops.at(static_cast<std::size_t>(act2.op_id)) = hc::OpState::kSucceeded;
  r.log.ops.at(static_cast<std::size_t>(cfg.op_id)) = hc::OpState::kSucceeded;
  EXPECT_EQ(r.write(&p, setpoint(kId, 1)), V::kHardwareNotActive);
}
