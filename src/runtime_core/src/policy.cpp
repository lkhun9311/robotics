// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/policy.hpp"

namespace runtime_core {

PolicyResult PolicyStateMachine::accept(RobotState next, StopCause cause) {
  state_ = next;
  cause_ = cause;
  return PolicyResult{true, state_};
}

PolicyResult PolicyStateMachine::handle(PolicyEvent e, const PolicyPreconditions& pre) {
  using S = RobotState;
  using E = PolicyEvent;
  switch (state_) {
    case S::kBoot:
      if (e == E::kBootOk) return accept(S::kInitializing, StopCause::kNone);
      if (e == E::kBootFailed || e == E::kFault) return accept(S::kFaultLatched, StopCause::kFault);
      if (e == E::kShutdownRequest) return accept(S::kShutdown, StopCause::kNone);
      break;
    case S::kInitializing:
      if (e == E::kInitReady) return accept(S::kReady, StopCause::kNone);
      if (e == E::kInitFailed || e == E::kFault) return accept(S::kFaultLatched, StopCause::kFault);
      if (e == E::kShutdownRequest) return accept(S::kShutdown, StopCause::kNone);
      break;
    case S::kReady:
      if (e == E::kEnableRequest && pre.fresh_feedback && pre.stop_confirmed) {
        return accept(S::kArming, StopCause::kNone);
      }
      if (e == E::kFault) return accept(S::kFaultLatched, StopCause::kFault);
      if (e == E::kShutdownRequest) return accept(S::kShutdown, StopCause::kNone);
      break;
    case S::kArming:
      if (e == E::kArmingConfirmed) return accept(S::kEnabled, StopCause::kNone);
      if (e == E::kArmingFailed || e == E::kFault) return accept(S::kStopping, StopCause::kFault);
      if (e == E::kDisableRequest) return accept(S::kStopping, StopCause::kNormal);
      break;
    case S::kEnabled:
      if (e == E::kTrajectoryDone || e == E::kDisableRequest) {
        return accept(S::kStopping, StopCause::kNormal);
      }
      if (e == E::kFault) return accept(S::kStopping, StopCause::kFault);
      break;
    case S::kStopping:
      if (e == E::kFault) return accept(S::kStopping, StopCause::kFault);
      if (e == E::kStopConfirmed) {
        return cause_ == StopCause::kFault ? accept(S::kFaultLatched, StopCause::kFault)
                                           : accept(S::kReady, StopCause::kNone);
      }
      if (e == E::kStopUnconfirmed) return accept(S::kFaultLatched, StopCause::kFault);
      break;
    case S::kFaultLatched:
      if (e == E::kResetRequest && pre.cause_cleared) {
        return accept(S::kRecovering, StopCause::kFault);
      }
      if (e == E::kShutdownRequest) return accept(S::kShutdown, StopCause::kFault);
      break;
    case S::kRecovering:
      if (e == E::kRecoveryVerified && pre.epochs_verified && pre.queues_flushed &&
          pre.fresh_feedback && pre.stop_confirmed) {
        return accept(S::kReady, StopCause::kNone);
      }
      if (e == E::kRecoveryFailed || e == E::kFault) {
        return accept(S::kFaultLatched, StopCause::kFault);
      }
      break;
    case S::kShutdown:
      break;
  }
  return reject();
}

const char* to_string(RobotState state) {
  switch (state) {
    case RobotState::kBoot:
      return "BOOT";
    case RobotState::kInitializing:
      return "INITIALIZING";
    case RobotState::kReady:
      return "READY";
    case RobotState::kArming:
      return "ARMING";
    case RobotState::kEnabled:
      return "ENABLED";
    case RobotState::kStopping:
      return "STOPPING";
    case RobotState::kFaultLatched:
      return "FAULT_LATCHED";
    case RobotState::kRecovering:
      return "RECOVERING";
    case RobotState::kShutdown:
      return "SHUTDOWN";
  }
  return "UNKNOWN";
}

const char* to_string(PolicyEvent event) {
  switch (event) {
    case PolicyEvent::kBootOk:
      return "BOOT_OK";
    case PolicyEvent::kBootFailed:
      return "BOOT_FAILED";
    case PolicyEvent::kInitReady:
      return "INIT_READY";
    case PolicyEvent::kInitFailed:
      return "INIT_FAILED";
    case PolicyEvent::kEnableRequest:
      return "ENABLE_REQUEST";
    case PolicyEvent::kArmingConfirmed:
      return "ARMING_CONFIRMED";
    case PolicyEvent::kArmingFailed:
      return "ARMING_FAILED";
    case PolicyEvent::kTrajectoryDone:
      return "TRAJECTORY_DONE";
    case PolicyEvent::kDisableRequest:
      return "DISABLE_REQUEST";
    case PolicyEvent::kFault:
      return "FAULT";
    case PolicyEvent::kStopConfirmed:
      return "STOP_CONFIRMED";
    case PolicyEvent::kStopUnconfirmed:
      return "STOP_UNCONFIRMED";
    case PolicyEvent::kResetRequest:
      return "RESET_REQUEST";
    case PolicyEvent::kRecoveryVerified:
      return "RECOVERY_VERIFIED";
    case PolicyEvent::kRecoveryFailed:
      return "RECOVERY_FAILED";
    case PolicyEvent::kShutdownRequest:
      return "SHUTDOWN_REQUEST";
  }
  return "UNKNOWN";
}

}  // namespace runtime_core
