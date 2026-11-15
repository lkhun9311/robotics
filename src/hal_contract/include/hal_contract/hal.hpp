// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include "hal_contract/backend.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/time.hpp"

namespace hal_contract {

enum class ArmState : std::uint8_t { kIdle, kArming, kArmed, kFailed };

const char* to_string(ArmState state);

// The host side of one arm (C1). Owns the backend, a HalGuard<6>, the capability requirements
// and the adopted AuthorityId, which changes only when an arm op completes.
//
// write() decides in this order, and submits only on kSend:
//   kInhibited, kHardwareNotActive, kNotArmed (no adopted identity yet, nothing to compare),
//   kNoPermit, kIdentityMismatch (permit), kIdentityMismatch (setpoint, or a snapshot whose
//   generation differs from its id), kDuplicate / kDuplicateMismatch / kRegression (against the
//   last submitted command, decided here once for every backend, C3), then the guard, which
//   receives the adopted manager and session (never the permit's), then
//   kExpiresBeforeApplication for a command the guard approved but the backend could no
//   longer apply.
// Every refusal except kDuplicate resets the guard's reference, so the next send is measured
// against feedback at rest; a duplicate leaves the device on the original command. A skewed
// feedback snapshot is passed to the guard as stale (C4).
class Hal {
 public:
  // Throws std::invalid_argument on a null backend or a lease outside (0, kMaxLease], and
  // whatever HalGuard throws on an invalid guard configuration or limits.
  Hal(std::unique_ptr<Backend> backend, runtime_core::Requirements requirements,
      runtime_core::GuardConfig guard_config, runtime_core::JointLimits<kJoints> limits,
      runtime_core::Duration lease = kMaxLease);

  OpHandle configure(runtime_core::MonoTime now);
  // The capability gate: refuses with the missing_capabilities list before Backend::activate.
  OpHandle activate(runtime_core::MonoTime now);
  // Refused without calling the backend when not active or when hold_until is not in
  // (now, now + kMaxHold]. From the call on, writes are kNotArmed until the op completes.
  OpHandle arm(const ArmRequest& request, runtime_core::MonoTime now);
  OpHandle deactivate(runtime_core::MonoTime now);
  void on_error_nonblocking();

  // Non-blocking. kUnknown for an id this Hal never issued or has forgotten (it keeps the
  // last kOpSlots ops).
  OpState op_state(OpHandle handle);
  // Lifecycle only: may block up to `timeout` in the backend. nullopt while still pending.
  std::optional<OpResult> wait(OpHandle handle, runtime_core::Duration timeout);

  HalVerdict write(const runtime_core::MotionPermit* permit, const HalSetpoint& setpoint,
                   runtime_core::MonoTime now);
  Feedback read(runtime_core::MonoTime now);

  ArmState arm_state();
  std::optional<AuthorityId> adopted();
  // The request id of the arm that produced the adopted identity (0 before any).
  std::uint64_t armed_request_id() const { return armed_request_id_; }

  static constexpr std::size_t kOpSlots = 8;

 private:
  enum class OpKind : std::uint8_t { kConfigure, kActivate, kArm, kDeactivate };

  struct OpSlot {
    std::uint64_t op_id{0};
    OpKind kind{OpKind::kConfigure};
    bool local{false};    // decided here; the backend never saw it
    bool settled{false};  // its effect on Hal state has been applied
    OpState state{OpState::kUnknown};
    ArmRequest request{};
  };

  struct LastSubmitted {
    AuthorityId id{};
    std::uint64_t source_seq{0};
    Joints position{};
  };

  OpSlot& post(OpKind kind);
  OpHandle refuse_locally(OpKind kind, OpResult result);
  OpSlot* find(std::uint64_t op_id);
  void refresh();
  void settle(OpSlot& slot);
  HalVerdict refuse(HalVerdict verdict);

  std::unique_ptr<Backend> backend_;
  runtime_core::Requirements requirements_;
  runtime_core::GuardConfig guard_config_;
  runtime_core::HalGuard<kJoints> guard_;
  runtime_core::Duration lease_;

  std::array<OpSlot, kOpSlots> ops_{};
  std::array<std::optional<OpResult>, kOpSlots> local_results_{};
  std::uint64_t next_op_id_{1};

  bool configured_{false};
  bool active_{false};
  ArmState arm_state_{ArmState::kIdle};
  std::uint64_t pending_arm_op_{0};
  std::optional<AuthorityId> adopted_{};
  std::uint64_t armed_request_id_{0};
  std::optional<LastSubmitted> last_submitted_{};
  Feedback feedback_{};
};

}  // namespace hal_contract
