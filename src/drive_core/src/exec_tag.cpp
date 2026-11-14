// SPDX-License-Identifier: Apache-2.0
#include "drive_core/exec_tag.hpp"

namespace drive_core {

std::uint16_t exec_tag_start(std::uint32_t boot_id) {
  // MurmurHash3 finalizer: consecutive boot ids spread over the whole tag range.
  std::uint32_t h = boot_id;
  h ^= h >> 16;
  h *= 0x85EB'CA6BU;
  h ^= h >> 13;
  h *= 0xC2B2'AE35U;
  h ^= h >> 16;
  return static_cast<std::uint16_t>(h % 0xFFFFU + 1U);  // 1..0xFFFF, never 0.
}

ExecTagIssuer::ExecTagIssuer(std::uint32_t boot_id, runtime_core::MonoTime boot_time,
                             runtime_core::Duration lifetime, runtime_core::Duration quarantine)
    : lifetime_(lifetime), quarantine_(quarantine) {
  boot(boot_id, boot_time);
}

ExecTagIssuer::CommitOutcome ExecTagIssuer::commit_ex(std::uint64_t generation,
                                                      std::uint32_t session,
                                                      runtime_core::MonoTime now) {
  if (now < commits_allowed_from_) {
    revoke();
    return {std::nullopt, CommitRefusal::kBootQuarantine};
  }
  if (next_ > 0xFFFFU) {
    revoke();
    return {std::nullopt, CommitRefusal::kExhausted};
  }
  current_ = static_cast<std::uint16_t>(next_);
  ++next_;
  expires_at_ = now + lifetime_;
  generation_ = generation;
  session_ = session;
  ++epoch_;
  return {current_, std::nullopt};
}

void ExecTagIssuer::tick(runtime_core::MonoTime now) {
  if (current_ != 0 && now >= expires_at_) {
    revoke();
  }
}

void ExecTagIssuer::revoke() {
  // Always a new epoch, even with no tag current: a consumer that missed nothing loses nothing,
  // and one that held state for an earlier tag drops it.
  current_ = 0;
  ++epoch_;
}

void ExecTagIssuer::reset_communication() {
  revoke();
  generation_ = 0;
  session_ = 0;
}

void ExecTagIssuer::boot(std::uint32_t boot_id, runtime_core::MonoTime now) {
  boot_id_ = boot_id;
  commits_allowed_from_ = now + quarantine_;
  next_ = exec_tag_start(boot_id);
  generation_ = 0;
  session_ = 0;
  revoke();
}

}  // namespace drive_core
