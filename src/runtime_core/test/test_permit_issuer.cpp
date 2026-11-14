// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "runtime_core/permit_issuer.hpp"

namespace rc = runtime_core;

namespace {

rc::PermitIssuer issuer() {
  return rc::PermitIssuer{rc::ManagerInstance{9}, rc::milliseconds(48), rc::milliseconds(5)};
}

rc::RenewalInputs healthy(rc::MonoTime now, std::uint64_t progress) {
  rc::RenewalInputs in;
  in.state = rc::RobotState::kEnabled;
  in.controller_adopted = rc::Generation{3};
  in.controller_progress_seq = progress;
  in.feedback_sampled_at = now - rc::milliseconds(1);
  return in;
}

}  // namespace

TEST(PermitIssuer, IssueSetsIdentityAndExpiry) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  EXPECT_EQ(p.manager.value, 9U);
  EXPECT_EQ(p.session.value, 2U);
  EXPECT_EQ(p.generation.value, 3U);
  EXPECT_EQ(p.expires_at.ns, 48'000'000);
}

TEST(PermitIssuer, RenewExtendsFromNowAndKeepsIdentity) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  const rc::MonoTime now = rc::MonoTime{} + rc::milliseconds(10);
  const auto r = is.renew(p, healthy(now, 11), now);
  ASSERT_TRUE(r.permit.has_value());
  EXPECT_EQ(r.refusal, rc::RenewalRefusal::kNone);
  EXPECT_EQ(r.permit->expires_at.ns, 58'000'000);
  EXPECT_EQ(r.permit->session.value, 2U);
  EXPECT_EQ(r.permit->generation.value, 3U);
}

TEST(PermitIssuer, RefusesPermitFromAnotherManagerInstance) {
  auto is = issuer();
  rc::MotionPermit foreign{rc::ManagerInstance{8}, rc::SessionId{2}, rc::Generation{3},
                           rc::MonoTime{48'000'000}};
  const rc::MonoTime now{1'000'000};
  const auto r = is.renew(foreign, healthy(now, 11), now);
  EXPECT_FALSE(r.permit.has_value());
  EXPECT_EQ(r.refusal, rc::RenewalRefusal::kForeignManager);
}

TEST(PermitIssuer, RefusesWhenNotEnabled) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  auto in = healthy(rc::MonoTime{1'000'000}, 11);
  in.state = rc::RobotState::kStopping;
  EXPECT_EQ(is.renew(p, in, rc::MonoTime{1'000'000}).refusal, rc::RenewalRefusal::kNotEnabled);
}

TEST(PermitIssuer, RefusesPermitAtItsExpiry) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  const rc::MonoTime at = p.expires_at;
  const auto r = is.renew(p, healthy(at, 11), at);
  EXPECT_FALSE(r.permit.has_value());
  EXPECT_EQ(r.refusal, rc::RenewalRefusal::kAlreadyExpired);
}

TEST(PermitIssuer, RefusesWhenGenerationNotAdopted) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  auto in = healthy(rc::MonoTime{1'000'000}, 11);
  in.controller_adopted = rc::Generation{2};
  EXPECT_EQ(is.renew(p, in, rc::MonoTime{1'000'000}).refusal,
            rc::RenewalRefusal::kGenerationNotAdopted);
}

TEST(PermitIssuer, RefusesWhenControllerStalled) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  const rc::MonoTime t1{1'000'000};
  EXPECT_EQ(is.renew(p, healthy(t1, 10), t1).refusal, rc::RenewalRefusal::kControllerStalled);
  EXPECT_EQ(is.renew(p, healthy(t1, 11), t1).refusal, rc::RenewalRefusal::kNone);
  const rc::MonoTime t2{2'000'000};
  EXPECT_EQ(is.renew(p, healthy(t2, 11), t2).refusal, rc::RenewalRefusal::kControllerStalled);
}

TEST(PermitIssuer, RefusesStaleFeedback) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  const rc::MonoTime now{10'000'000};
  auto in = healthy(now, 11);
  in.feedback_sampled_at = now - rc::milliseconds(6);
  EXPECT_EQ(is.renew(p, in, now).refusal, rc::RenewalRefusal::kFeedbackStale);
}

TEST(PermitIssuer, RefusesFutureFeedback) {
  auto is = issuer();
  const auto p = is.issue(rc::SessionId{2}, rc::Generation{3}, 10, rc::MonoTime{0});
  const rc::MonoTime now{10'000'000};
  auto in = healthy(now, 11);
  in.feedback_sampled_at = now + rc::Duration{1};
  EXPECT_EQ(is.renew(p, in, now).refusal, rc::RenewalRefusal::kFeedbackStale);
}
