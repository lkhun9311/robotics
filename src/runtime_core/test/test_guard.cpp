// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "runtime_core/guard.hpp"

namespace rc = runtime_core;

namespace {

constexpr std::size_t kN = 2;
const rc::MonoTime kNow{50'000'000};

rc::JointLimits<kN> limits(double acc_max = 1000.0) {
  rc::JointLimits<kN> l;
  l.pos_min = {-1.0, -1.0};
  l.pos_max = {1.0, 1.0};
  l.vel_max = {1.0, 1.0};
  l.acc_max = {acc_max, acc_max};
  return l;
}

rc::GuardConfig config() { return rc::GuardConfig{rc::milliseconds(1), rc::milliseconds(5)}; }

struct Fixture {
  rc::MotionPermit permit{rc::ManagerInstance{1}, rc::SessionId{7}, rc::Generation{3},
                          rc::MonoTime{100'000'000}};

  rc::GuardInput<kN> input(rc::MonoTime now, double q0 = 0.0005) const {
    rc::GuardInput<kN> in;
    in.hardware_active = true;
    in.permit = &permit;
    in.current_manager = rc::ManagerInstance{1};
    in.current_session = rc::SessionId{7};
    in.setpoint.generation = rc::Generation{3};
    in.setpoint.source_seq = 1;
    in.setpoint.position = {q0, 0.0};
    in.feedback_sampled_at = now - rc::milliseconds(1);
    in.feedback_position = {0.0, 0.0};
    return in;
  }
};

}  // namespace

TEST(HalGuard, SendsWhenAllConditionsHold) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  EXPECT_EQ(g.check(f.input(kNow), kNow), rc::GuardVerdict::kSend);
}

TEST(HalGuard, BlocksWhenHardwareInactive) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.hardware_active = false;
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kHardwareNotActive);
}

TEST(HalGuard, BlocksWithoutPermit) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.permit = nullptr;
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kNoPermit);
}

TEST(HalGuard, PermitExpiredAtExactBoundary) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  const rc::MonoTime at = f.permit.expires_at;
  EXPECT_EQ(g.check(f.input(at), at), rc::GuardVerdict::kPermitExpired);
  rc::HalGuard<kN> g2{config(), limits()};
  const rc::MonoTime just_before{at.ns - 1};
  EXPECT_EQ(g2.check(f.input(just_before), just_before), rc::GuardVerdict::kSend);
}

TEST(HalGuard, BlocksPermitFromAnotherManager) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.current_manager = rc::ManagerInstance{2};
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kManagerMismatch);
}

TEST(HalGuard, RejectsMeaninglessConfiguration) {
  EXPECT_THROW((rc::HalGuard<kN>{rc::GuardConfig{rc::Duration{0}, rc::milliseconds(5)}, limits()}),
               std::invalid_argument);
  EXPECT_THROW((rc::HalGuard<kN>{rc::GuardConfig{rc::milliseconds(1), rc::Duration{-1}}, limits()}),
               std::invalid_argument);
  auto bad = limits();
  bad.vel_max[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((rc::HalGuard<kN>{config(), bad}), std::invalid_argument);
  bad = limits();
  bad.pos_min[1] = 2.0;
  EXPECT_THROW((rc::HalGuard<kN>{config(), bad}), std::invalid_argument);
}

TEST(HalGuard, BlocksSessionAndGenerationMismatch) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.current_session = rc::SessionId{8};
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kSessionMismatch);
  in = f.input(kNow);
  in.setpoint.generation = rc::Generation{2};
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kGenerationMismatch);
}

TEST(HalGuard, StaleFeedbackIsBlocked) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.feedback_sampled_at = kNow - rc::milliseconds(6);
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kFeedbackStale);
}

TEST(HalGuard, FutureFeedbackIsStale) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.feedback_sampled_at = kNow + rc::Duration{1};
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kFeedbackStale);
}

TEST(HalGuard, NonFiniteSetpointIsBlocked) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.setpoint.position[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kNonFinite);
  in.setpoint.position[1] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kNonFinite);
}

TEST(HalGuard, NonFiniteFeedbackIsBlocked) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  auto in = f.input(kNow);
  in.feedback_position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(g.check(in, kNow), rc::GuardVerdict::kNonFinite);
}

TEST(HalGuard, PositionLimit) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  EXPECT_EQ(g.check(f.input(kNow, 1.5), kNow), rc::GuardVerdict::kPositionLimit);
}

TEST(HalGuard, VelocityLimit) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  // 0.002 rad in 1 ms is 2 rad/s, limit is 1 rad/s.
  EXPECT_EQ(g.check(f.input(kNow, 0.002), kNow), rc::GuardVerdict::kVelocityLimit);
}

TEST(HalGuard, AccelerationLimit) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits(100.0)};
  // From rest to 0.05 rad/s: 50 rad/s^2, allowed.
  EXPECT_EQ(g.check(f.input(kNow, 0.00005), kNow), rc::GuardVerdict::kSend);
  const rc::MonoTime next = kNow + rc::milliseconds(1);
  // 0.05 -> 0.2 rad/s in 1 ms: 150 rad/s^2, over 100.
  EXPECT_EQ(g.check(f.input(next, 0.00025), next), rc::GuardVerdict::kAccelerationLimit);
}

TEST(HalGuard, ContinuousSendsUseLastSentCommandAsReference) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  EXPECT_EQ(g.check(f.input(kNow, 0.0005), kNow), rc::GuardVerdict::kSend);
  const rc::MonoTime next = kNow + rc::milliseconds(1);
  // From last sent 0.0005: 0.9 rad/s, 400 rad/s^2. Both allowed.
  EXPECT_EQ(g.check(f.input(next, 0.0014), next), rc::GuardVerdict::kSend);
}

TEST(HalGuard, AfterABlockTheReferenceIsFeedbackAtRest) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  EXPECT_EQ(g.check(f.input(kNow, 0.0005), kNow), rc::GuardVerdict::kSend);
  auto blocked = f.input(kNow + rc::milliseconds(1), 0.0009);
  blocked.hardware_active = false;
  EXPECT_EQ(g.check(blocked, kNow + rc::milliseconds(1)), rc::GuardVerdict::kHardwareNotActive);
  const rc::MonoTime next = kNow + rc::milliseconds(2);
  // Feedback is 0.0: 0.0014 rad in 1 ms is 1.4 rad/s, over the limit.
  EXPECT_EQ(g.check(f.input(next, 0.0014), next), rc::GuardVerdict::kVelocityLimit);
}

TEST(HalGuard, ResetReferenceAtArming) {
  const Fixture f;
  rc::HalGuard<kN> g{config(), limits()};
  EXPECT_EQ(g.check(f.input(kNow, 0.0005), kNow), rc::GuardVerdict::kSend);
  g.reset_reference();
  const rc::MonoTime next = kNow + rc::milliseconds(1);
  EXPECT_EQ(g.check(f.input(next, 0.0014), next), rc::GuardVerdict::kVelocityLimit);
}

TEST(HalGuard, ExpiryPredicateIsInjectableForDefectTests) {
  const Fixture f;
  auto off_by_one = +[](rc::MonoTime now, rc::MonoTime valid_until) { return now > valid_until; };
  rc::HalGuard<kN> g{config(), limits(), off_by_one};
  const rc::MonoTime at = f.permit.expires_at;
  EXPECT_EQ(g.check(f.input(at), at), rc::GuardVerdict::kSend);
}

TEST(HalGuard, VerdictNames) {
  EXPECT_EQ(std::string{rc::to_string(rc::GuardVerdict::kPermitExpired)}, "PERMIT_EXPIRED");
  EXPECT_EQ(std::string{rc::to_string(rc::GuardVerdict::kGenerationMismatch)},
            "GENERATION_MISMATCH");
}
