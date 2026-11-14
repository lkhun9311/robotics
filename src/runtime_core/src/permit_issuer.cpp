// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/permit_issuer.hpp"

#include <stdexcept>

#include "runtime_core/deadline.hpp"

namespace runtime_core {

PermitIssuer::PermitIssuer(ManagerInstance manager, Duration lease, Duration max_feedback_age)
    : manager_(manager), lease_(lease), max_feedback_age_(max_feedback_age) {
  // A zero or negative lease issues permits that are already expired; a zero feedback age
  // refuses every renewal. Both are configuration errors, not runtime states.
  if (lease.ns <= 0) throw std::invalid_argument("PermitIssuer: lease must be positive");
  if (max_feedback_age.ns <= 0) {
    throw std::invalid_argument("PermitIssuer: max_feedback_age must be positive");
  }
}

MotionPermit PermitIssuer::issue(SessionId session, Generation generation,
                                 std::uint64_t controller_progress_seq, MonoTime now) {
  last_progress_ = controller_progress_seq;
  return MotionPermit{manager_, session, generation, now + lease_};
}

RenewalResult PermitIssuer::renew(const MotionPermit& current, const RenewalInputs& in,
                                  MonoTime now) {
  // A permit from another manager instance (e.g. before a manager restart) is never renewed.
  if (current.manager != manager_) return {std::nullopt, RenewalRefusal::kForeignManager};
  if (in.state != RobotState::kEnabled) return {std::nullopt, RenewalRefusal::kNotEnabled};
  if (is_expired(now, current.expires_at)) {
    return {std::nullopt, RenewalRefusal::kAlreadyExpired};
  }
  if (in.controller_adopted != current.generation) {
    return {std::nullopt, RenewalRefusal::kGenerationNotAdopted};
  }
  if (in.controller_progress_seq <= last_progress_) {
    return {std::nullopt, RenewalRefusal::kControllerStalled};
  }
  const Duration age = now - in.feedback_sampled_at;
  if (age.ns < 0 || age > max_feedback_age_) {
    return {std::nullopt, RenewalRefusal::kFeedbackStale};
  }
  last_progress_ = in.controller_progress_seq;
  return {MotionPermit{manager_, current.session, current.generation, now + lease_},
          RenewalRefusal::kNone};
}

const char* to_string(RenewalRefusal refusal) {
  switch (refusal) {
    case RenewalRefusal::kNone:
      return "NONE";
    case RenewalRefusal::kForeignManager:
      return "FOREIGN_MANAGER";
    case RenewalRefusal::kNotEnabled:
      return "NOT_ENABLED";
    case RenewalRefusal::kAlreadyExpired:
      return "ALREADY_EXPIRED";
    case RenewalRefusal::kGenerationNotAdopted:
      return "GENERATION_NOT_ADOPTED";
    case RenewalRefusal::kControllerStalled:
      return "CONTROLLER_STALLED";
    case RenewalRefusal::kFeedbackStale:
      return "FEEDBACK_STALE";
  }
  return "UNKNOWN";
}

}  // namespace runtime_core
