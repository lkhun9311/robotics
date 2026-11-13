// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace runtime_core {

enum class RobotState {
  kBoot,
  kInitializing,
  kReady,
  kArming,
  kEnabled,
  kStopping,
  kFaultLatched,
  kRecovering,
  kShutdown,
};

enum class StopCause { kNone, kNormal, kFault };

enum class PolicyEvent {
  kBootOk,
  kBootFailed,
  kInitReady,
  kInitFailed,
  kEnableRequest,
  kArmingConfirmed,
  kArmingFailed,
  kTrajectoryDone,
  kDisableRequest,
  kFault,
  kStopConfirmed,
  kStopUnconfirmed,
  kResetRequest,
  kRecoveryVerified,
  kRecoveryFailed,
  kShutdownRequest,
};

struct PolicyPreconditions {
  bool cause_cleared{false};
  bool epochs_verified{false};
  bool queues_flushed{false};
  bool fresh_feedback{false};
  bool stop_confirmed{false};
};

struct PolicyResult {
  bool accepted{false};
  RobotState state{RobotState::kBoot};
};

// Robot policy states (spec §8.1, §8.4). Reset never enables motion; a fault stop always
// latches; a normal stop returns to READY without Reset.
class PolicyStateMachine {
 public:
  RobotState state() const { return state_; }
  StopCause stop_cause() const { return cause_; }
  PolicyResult handle(PolicyEvent event, const PolicyPreconditions& pre = {});

 private:
  PolicyResult accept(RobotState next, StopCause cause);
  PolicyResult reject() const { return PolicyResult{false, state_}; }

  RobotState state_{RobotState::kBoot};
  StopCause cause_{StopCause::kNone};
};

const char* to_string(RobotState state);
const char* to_string(PolicyEvent event);

}  // namespace runtime_core
