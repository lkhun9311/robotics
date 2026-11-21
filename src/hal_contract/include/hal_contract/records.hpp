// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "hal_contract/faults.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/ids.hpp"
#include "runtime_core/time.hpp"

namespace hal_contract {

inline constexpr std::size_t kJoints = 6;
using Joints = std::array<double, kJoints>;

// The longest validity a host command may carry (spec §6.2, the 48 ms wire lease of E3a).
inline constexpr runtime_core::Duration kMaxLease = runtime_core::milliseconds(48);
// The furthest ahead an arm request may hold without a permit (C12: hold_until in (now, now+1s]).
inline constexpr runtime_core::Duration kMaxHold = runtime_core::milliseconds(1000);

// Absolute op deadlines are set when an op is posted, whether or not anyone waits (C10).
inline constexpr runtime_core::Duration kConfigureTimeout = runtime_core::milliseconds(3000);
inline constexpr runtime_core::Duration kActivateTimeout = runtime_core::milliseconds(1000);
inline constexpr runtime_core::Duration kArmTimeout = runtime_core::milliseconds(1000);
inline constexpr runtime_core::Duration kDeactivateTimeout = runtime_core::milliseconds(1000);

// The three fields that name who may move the arm (C1).
// All three are compared, always: a restarted manager may reuse the numbers of session and
// generation, never its instance.
struct AuthorityId {
  runtime_core::ManagerInstance manager{};
  runtime_core::SessionId session{};
  runtime_core::Generation generation{};
  bool operator==(const AuthorityId&) const = default;
};

// Arming adopts `id` once the backend has committed it, holding the actual position until
// `hold_until` under the request's own bounded authority (no permit needed, C9 phase 4).
struct ArmRequest {
  AuthorityId id{};
  runtime_core::MonoTime hold_until{};
  std::uint64_t request_id{0};
};

// One controller output: the identity it was computed under and the snapshot (C1).
// `snapshot.generation` must equal `id.generation`.
struct HalSetpoint {
  AuthorityId id{};
  runtime_core::SetpointSnapshot<kJoints> snapshot{};
};

// What Hal hands a backend after HalGuard returned kSend (C3).
// valid_until = min(permit.expires_at, approved_at + lease).
struct ApprovedCommand {
  AuthorityId id{};
  std::uint64_t source_seq{0};
  Joints position{};
  runtime_core::MonoTime approved_at{};
  runtime_core::MonoTime valid_until{};
};

// HalGuard's verdicts, in the same order and values, then the ones Hal adds (C3).
enum class HalVerdict : std::uint8_t {
  kSend,
  kHardwareNotActive,
  kNoPermit,
  kPermitExpired,
  kManagerMismatch,
  kSessionMismatch,
  kGenerationMismatch,
  kFeedbackStale,
  kNonFinite,
  kPositionLimit,
  kVelocityLimit,
  kAccelerationLimit,
  kIdentityMismatch,          // permit or setpoint identity differs from the adopted one
  kDuplicate,                 // same (id, source_seq) as the last submitted: nothing new sent
  kDuplicateMismatch,         // same (id, source_seq) with a different position
  kRegression,                // a source_seq older than the last submitted
  kInhibited,                 // the group motion inhibit is latched (C17)
  kNotArmed,                  // active, but no arm op has completed (or a re-arm is running)
  kExpiresBeforeApplication,  // its valid_until closes before the backend could apply it (C6)
};

HalVerdict from_guard(runtime_core::GuardVerdict verdict);
const char* to_string(HalVerdict verdict);

// Per-node stop state after the group inhibit (C18).
enum class StopStatus : std::uint8_t {
  kNone,         // no stop requested
  kStopping,     // stop requested, not yet confirmed
  kConfirmed,    // the drive's own evidence and 5 distinct fresh samples at rest
  kUnconfirmed,  // drive evidence against it, stale feedback, or the 0.172 s deadline passed
  kNotEnabled,   // the node was never enabled in this arm: nothing to stop, never a powered hold
};

const char* to_string(StopStatus status);

// Counters of commands that never became device commands (C6, C19).
// They are reported, not acknowledged: a superseded command never appears in
// group_applied_source_seq.
struct CommandCounters {
  std::uint64_t superseded{0};               // overwritten by a newer one before dispatch
  std::uint64_t expired_before_dispatch{0};  // no longer open when it could have been sent
};

// The robot-level feedback snapshot (C4).
// Times are source times, never receive times.
struct Feedback {
  Joints position{};
  Joints velocity{};
  // The oldest of the per-node source times; MonoTime{} before the first sample.
  runtime_core::MonoTime sampled_at{};
  std::array<runtime_core::MonoTime, kJoints> node_sampled_at{};
  // The source times disagree by more than T_bus + W: the guard treats the snapshot as stale.
  bool skewed{false};
  // Set only when every node actuated the same approved command (never a hold command).
  std::optional<std::uint64_t> group_applied_source_seq{};
  FaultSet faults{};
  std::array<StopStatus, kJoints> stop{};
  CommandCounters counters{};
};

struct OpHandle {
  std::uint64_t op_id{0};  // 0 is never issued
  bool operator==(const OpHandle&) const = default;
};

enum class OpState : std::uint8_t { kPending, kSucceeded, kFailed, kUnknown };

const char* to_string(OpState state);

struct OpResult {
  bool ok{false};
  std::string error{};
  std::vector<runtime_core::Missing> missing{};  // set when activation was refused by the gate
};

// What the Hal gives a backend with every lifecycle op: the id it is reported under and the
// absolute deadline after which the backend must fail it, whether or not anyone waits (C10).
struct OpTicket {
  std::uint64_t op_id{0};
  runtime_core::MonoTime deadline{};
};

}  // namespace hal_contract
