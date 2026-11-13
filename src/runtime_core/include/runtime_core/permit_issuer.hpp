// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

#include "runtime_core/ids.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/policy.hpp"
#include "runtime_core/time.hpp"

namespace runtime_core {

struct RenewalInputs {
  RobotState state{RobotState::kBoot};
  Generation controller_adopted{};
  std::uint64_t controller_progress_seq{0};
  MonoTime feedback_sampled_at{};
};

enum class RenewalRefusal {
  kNone,
  kForeignManager,
  kNotEnabled,
  kAlreadyExpired,
  kGenerationNotAdopted,
  kControllerStalled,
  kFeedbackStale,
};

struct RenewalResult {
  std::optional<MotionPermit> permit;
  RenewalRefusal refusal{RenewalRefusal::kNone};
};

// Renews a permit only on observed progress (spec §6.2): a live timer alone never renews.
// An expired permit is never renewed; a new one must be issued through ARMING.
class PermitIssuer {
 public:
  PermitIssuer(ManagerInstance manager, Duration lease, Duration max_feedback_age);

  MotionPermit issue(SessionId session, Generation generation,
                     std::uint64_t controller_progress_seq, MonoTime now);
  RenewalResult renew(const MotionPermit& current, const RenewalInputs& in, MonoTime now);

 private:
  ManagerInstance manager_;
  Duration lease_;
  Duration max_feedback_age_;
  std::uint64_t last_progress_{0};
};

const char* to_string(RenewalRefusal refusal);

}  // namespace runtime_core
