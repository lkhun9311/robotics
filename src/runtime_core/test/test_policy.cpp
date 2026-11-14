// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <string>

#include "runtime_core/policy.hpp"

namespace rc = runtime_core;
using E = rc::PolicyEvent;
using S = rc::RobotState;

namespace {

rc::PolicyPreconditions all_true() { return {true, true, true, true, true}; }

rc::PolicyStateMachine ready() {
  rc::PolicyStateMachine sm;
  sm.handle(E::kBootOk);
  sm.handle(E::kInitReady);
  return sm;
}

rc::PolicyStateMachine enabled() {
  auto sm = ready();
  sm.handle(E::kEnableRequest, all_true());
  sm.handle(E::kArmingConfirmed);
  return sm;
}

}  // namespace

TEST(Policy, BootsToReady) { EXPECT_EQ(ready().state(), S::kReady); }

TEST(Policy, EnableNeedsFreshFeedbackAndConfirmedStop) {
  auto sm = ready();
  auto pre = all_true();
  pre.fresh_feedback = false;
  EXPECT_FALSE(sm.handle(E::kEnableRequest, pre).accepted);
  pre = all_true();
  pre.stop_confirmed = false;
  EXPECT_FALSE(sm.handle(E::kEnableRequest, pre).accepted);
  EXPECT_EQ(sm.state(), S::kReady);
  EXPECT_TRUE(sm.handle(E::kEnableRequest, all_true()).accepted);
  EXPECT_EQ(sm.state(), S::kArming);
}

TEST(Policy, NormalStopReturnsToReadyWithoutReset) {
  auto sm = enabled();
  EXPECT_EQ(sm.state(), S::kEnabled);
  sm.handle(E::kTrajectoryDone);
  EXPECT_EQ(sm.state(), S::kStopping);
  EXPECT_EQ(sm.stop_cause(), rc::StopCause::kNormal);
  sm.handle(E::kStopConfirmed);
  EXPECT_EQ(sm.state(), S::kReady);
}

TEST(Policy, FaultStopLatchesEvenWhenStopIsConfirmed) {
  auto sm = enabled();
  sm.handle(E::kFault);
  sm.handle(E::kStopConfirmed);
  EXPECT_EQ(sm.state(), S::kFaultLatched);
}

TEST(Policy, UnconfirmedStopLatches) {
  auto sm = enabled();
  sm.handle(E::kDisableRequest);
  sm.handle(E::kStopUnconfirmed);
  EXPECT_EQ(sm.state(), S::kFaultLatched);
}

TEST(Policy, FaultDuringNormalStopEscalates) {
  auto sm = enabled();
  sm.handle(E::kDisableRequest);
  EXPECT_TRUE(sm.handle(E::kFault).accepted);
  EXPECT_EQ(sm.stop_cause(), rc::StopCause::kFault);
  sm.handle(E::kStopConfirmed);
  EXPECT_EQ(sm.state(), S::kFaultLatched);
}

TEST(Policy, ResetNeverEnables) {
  auto sm = enabled();
  sm.handle(E::kFault);
  sm.handle(E::kStopConfirmed);
  EXPECT_FALSE(sm.handle(E::kEnableRequest, all_true()).accepted);
  EXPECT_TRUE(sm.handle(E::kResetRequest, all_true()).accepted);
  EXPECT_EQ(sm.state(), S::kRecovering);
  EXPECT_TRUE(sm.handle(E::kRecoveryVerified, all_true()).accepted);
  EXPECT_EQ(sm.state(), S::kReady);
}

TEST(Policy, ResetRequiresClearedCause) {
  auto sm = enabled();
  sm.handle(E::kFault);
  sm.handle(E::kStopConfirmed);
  auto pre = all_true();
  pre.cause_cleared = false;
  EXPECT_FALSE(sm.handle(E::kResetRequest, pre).accepted);
  EXPECT_EQ(sm.state(), S::kFaultLatched);
}

TEST(Policy, RecoveryRequiresEveryPrecondition) {
  for (int missing = 0; missing < 4; ++missing) {
    auto sm = enabled();
    sm.handle(E::kFault);
    sm.handle(E::kStopConfirmed);
    sm.handle(E::kResetRequest, all_true());
    auto pre = all_true();
    if (missing == 0) pre.epochs_verified = false;
    if (missing == 1) pre.queues_flushed = false;
    if (missing == 2) pre.fresh_feedback = false;
    if (missing == 3) pre.stop_confirmed = false;
    EXPECT_FALSE(sm.handle(E::kRecoveryVerified, pre).accepted) << "missing=" << missing;
    EXPECT_EQ(sm.state(), S::kRecovering);
  }
}

TEST(Policy, ShutdownIsRejectedWhileMoving) {
  auto sm = enabled();
  EXPECT_FALSE(sm.handle(E::kShutdownRequest).accepted);
  EXPECT_EQ(sm.state(), S::kEnabled);
}

TEST(Policy, UnexpectedEventsAreRejectedWithoutStateChange) {
  auto sm = ready();
  const auto r = sm.handle(E::kStopConfirmed);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.state, S::kReady);
}

TEST(Policy, Names) {
  EXPECT_EQ(std::string{rc::to_string(S::kFaultLatched)}, "FAULT_LATCHED");
  EXPECT_EQ(std::string{rc::to_string(S::kEnabled)}, "ENABLED");
  EXPECT_EQ(std::string{rc::to_string(E::kEnableRequest)}, "ENABLE_REQUEST");
}
