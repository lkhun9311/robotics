// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/guard.hpp"

namespace runtime_core {

const char* to_string(GuardVerdict verdict) {
  switch (verdict) {
    case GuardVerdict::kSend:
      return "SEND";
    case GuardVerdict::kHardwareNotActive:
      return "HARDWARE_NOT_ACTIVE";
    case GuardVerdict::kNoPermit:
      return "NO_PERMIT";
    case GuardVerdict::kPermitExpired:
      return "PERMIT_EXPIRED";
    case GuardVerdict::kManagerMismatch:
      return "MANAGER_MISMATCH";
    case GuardVerdict::kSessionMismatch:
      return "SESSION_MISMATCH";
    case GuardVerdict::kGenerationMismatch:
      return "GENERATION_MISMATCH";
    case GuardVerdict::kFeedbackStale:
      return "FEEDBACK_STALE";
    case GuardVerdict::kNonFinite:
      return "NON_FINITE";
    case GuardVerdict::kPositionLimit:
      return "POSITION_LIMIT";
    case GuardVerdict::kVelocityLimit:
      return "VELOCITY_LIMIT";
    case GuardVerdict::kAccelerationLimit:
      return "ACCELERATION_LIMIT";
  }
  return "UNKNOWN";
}

}  // namespace runtime_core
