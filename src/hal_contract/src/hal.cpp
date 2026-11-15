// SPDX-License-Identifier: Apache-2.0
#include "hal_contract/hal.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "runtime_core/deadline.hpp"

namespace hal_contract {

namespace rc = runtime_core;

const char* to_string(Fault fault) {
  switch (fault) {
    case Fault::kTransportLost:
      return "TRANSPORT_LOST";
    case Fault::kNodeLost:
      return "NODE_LOST";
    case Fault::kBootTimeout:
      return "BOOT_TIMEOUT";
    case Fault::kDriveFault:
      return "DRIVE_FAULT";
    case Fault::kEventOverflow:
      return "EVENT_OVERFLOW";
    case Fault::kStopUnconfirmed:
      return "STOP_UNCONFIRMED";
    case Fault::kPolicyStop:
      return "POLICY_STOP";
    case Fault::kInhibited:
      return "INHIBITED";
    case Fault::kArmFailed:
      return "ARM_FAILED";
  }
  return "UNKNOWN";
}

HalVerdict from_guard(rc::GuardVerdict verdict) {
  switch (verdict) {
    case rc::GuardVerdict::kSend:
      return HalVerdict::kSend;
    case rc::GuardVerdict::kHardwareNotActive:
      return HalVerdict::kHardwareNotActive;
    case rc::GuardVerdict::kNoPermit:
      return HalVerdict::kNoPermit;
    case rc::GuardVerdict::kPermitExpired:
      return HalVerdict::kPermitExpired;
    case rc::GuardVerdict::kManagerMismatch:
      return HalVerdict::kManagerMismatch;
    case rc::GuardVerdict::kSessionMismatch:
      return HalVerdict::kSessionMismatch;
    case rc::GuardVerdict::kGenerationMismatch:
      return HalVerdict::kGenerationMismatch;
    case rc::GuardVerdict::kFeedbackStale:
      return HalVerdict::kFeedbackStale;
    case rc::GuardVerdict::kNonFinite:
      return HalVerdict::kNonFinite;
    case rc::GuardVerdict::kPositionLimit:
      return HalVerdict::kPositionLimit;
    case rc::GuardVerdict::kVelocityLimit:
      return HalVerdict::kVelocityLimit;
    case rc::GuardVerdict::kAccelerationLimit:
      return HalVerdict::kAccelerationLimit;
  }
  // An unknown guard verdict is a refusal, never a send.
  return HalVerdict::kHardwareNotActive;
}

const char* to_string(HalVerdict verdict) {
  switch (verdict) {
    case HalVerdict::kSend:
      return "SEND";
    case HalVerdict::kHardwareNotActive:
      return "HARDWARE_NOT_ACTIVE";
    case HalVerdict::kNoPermit:
      return "NO_PERMIT";
    case HalVerdict::kPermitExpired:
      return "PERMIT_EXPIRED";
    case HalVerdict::kManagerMismatch:
      return "MANAGER_MISMATCH";
    case HalVerdict::kSessionMismatch:
      return "SESSION_MISMATCH";
    case HalVerdict::kGenerationMismatch:
      return "GENERATION_MISMATCH";
    case HalVerdict::kFeedbackStale:
      return "FEEDBACK_STALE";
    case HalVerdict::kNonFinite:
      return "NON_FINITE";
    case HalVerdict::kPositionLimit:
      return "POSITION_LIMIT";
    case HalVerdict::kVelocityLimit:
      return "VELOCITY_LIMIT";
    case HalVerdict::kAccelerationLimit:
      return "ACCELERATION_LIMIT";
    case HalVerdict::kIdentityMismatch:
      return "IDENTITY_MISMATCH";
    case HalVerdict::kDuplicate:
      return "DUPLICATE";
    case HalVerdict::kDuplicateMismatch:
      return "DUPLICATE_MISMATCH";
    case HalVerdict::kRegression:
      return "REGRESSION";
    case HalVerdict::kInhibited:
      return "INHIBITED";
    case HalVerdict::kNotArmed:
      return "NOT_ARMED";
    case HalVerdict::kExpiresBeforeApplication:
      return "EXPIRES_BEFORE_APPLICATION";
  }
  return "UNKNOWN";
}

const char* to_string(StopStatus status) {
  switch (status) {
    case StopStatus::kNone:
      return "NONE";
    case StopStatus::kStopping:
      return "STOPPING";
    case StopStatus::kConfirmed:
      return "CONFIRMED";
    case StopStatus::kUnconfirmed:
      return "UNCONFIRMED";
    case StopStatus::kNotEnabled:
      return "NOT_ENABLED";
  }
  return "UNKNOWN";
}

const char* to_string(OpState state) {
  switch (state) {
    case OpState::kPending:
      return "PENDING";
    case OpState::kSucceeded:
      return "SUCCEEDED";
    case OpState::kFailed:
      return "FAILED";
    case OpState::kUnknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char* to_string(ArmState state) {
  switch (state) {
    case ArmState::kIdle:
      return "IDLE";
    case ArmState::kArming:
      return "ARMING";
    case ArmState::kArmed:
      return "ARMED";
    case ArmState::kFailed:
      return "FAILED";
  }
  return "UNKNOWN";
}

namespace {

std::unique_ptr<Backend> require_backend(std::unique_ptr<Backend> backend) {
  if (backend == nullptr) throw std::invalid_argument("Hal: backend is null");
  return backend;
}

std::string describe_missing(const std::vector<rc::Missing>& missing) {
  std::string out = "missing capabilities:";
  for (const rc::Missing& m : missing) {
    out += ' ';
    out += rc::to_string(m.capability);
    out += '=';
    out += rc::to_string(m.support);
  }
  return out;
}

}  // namespace

Hal::Hal(std::unique_ptr<Backend> backend, rc::Requirements requirements,
         rc::GuardConfig guard_config, rc::JointLimits<kJoints> limits, rc::Duration lease)
    : backend_(require_backend(std::move(backend))),
      requirements_(requirements),
      guard_config_(guard_config),
      guard_(guard_config, limits),
      lease_(lease) {
  if (lease_.ns <= 0 || lease_ > kMaxLease) {
    throw std::invalid_argument("Hal: lease must be in (0, 48 ms]");
  }
}

Hal::OpSlot& Hal::post(OpKind kind) {
  const std::uint64_t id = next_op_id_++;
  const std::size_t index = static_cast<std::size_t>(id % kOpSlots);
  ops_[index] = OpSlot{id, kind, false, false, OpState::kPending, ArmRequest{}};
  local_results_[index].reset();
  return ops_[index];
}

OpHandle Hal::refuse_locally(OpKind kind, OpResult result) {
  OpSlot& slot = post(kind);
  slot.local = true;
  slot.settled = true;
  slot.state = OpState::kFailed;
  local_results_[static_cast<std::size_t>(slot.op_id % kOpSlots)] = std::move(result);
  return OpHandle{slot.op_id};
}

Hal::OpSlot* Hal::find(std::uint64_t op_id) {
  if (op_id == 0) return nullptr;
  OpSlot& slot = ops_[static_cast<std::size_t>(op_id % kOpSlots)];
  return slot.op_id == op_id ? &slot : nullptr;
}

void Hal::settle(OpSlot& slot) {
  slot.settled = true;
  const bool ok = slot.state == OpState::kSucceeded;
  switch (slot.kind) {
    case OpKind::kConfigure:
      configured_ = ok;
      break;
    case OpKind::kActivate:
      active_ = ok;
      break;
    case OpKind::kArm:
      // Only the newest arm decides; an older one finishing late changes nothing.
      if (slot.op_id != pending_arm_op_) break;
      pending_arm_op_ = 0;
      if (ok) {
        adopted_ = slot.request.id;
        armed_request_id_ = slot.request.request_id;
        arm_state_ = ArmState::kArmed;
      } else {
        arm_state_ = ArmState::kFailed;
      }
      last_submitted_.reset();
      guard_.reset_reference();
      break;
    case OpKind::kDeactivate:
      break;
  }
}

void Hal::refresh() {
  for (OpSlot& slot : ops_) {
    if (slot.op_id == 0 || slot.settled || slot.local) continue;
    slot.state = backend_->op_state(slot.op_id);
    if (slot.state == OpState::kSucceeded || slot.state == OpState::kFailed) settle(slot);
  }
}

OpHandle Hal::configure(rc::MonoTime now) {
  configured_ = false;
  active_ = false;
  arm_state_ = ArmState::kIdle;
  pending_arm_op_ = 0;
  adopted_.reset();
  last_submitted_.reset();
  guard_.reset_reference();
  const rc::MonoTime deadline = now + kConfigureTimeout;
  OpSlot& slot = post(OpKind::kConfigure);
  const std::uint64_t id = slot.op_id;
  backend_->configure(OpTicket{id, deadline}, now);
  refresh();
  return OpHandle{id};
}

OpHandle Hal::activate(rc::MonoTime now) {
  refresh();
  if (!configured_) return refuse_locally(OpKind::kActivate, OpResult{false, "not configured", {}});
  std::vector<rc::Missing> missing =
      rc::missing_capabilities(requirements_, backend_->capabilities());
  if (!missing.empty()) {
    std::string error = describe_missing(missing);
    return refuse_locally(OpKind::kActivate, OpResult{false, std::move(error), std::move(missing)});
  }
  const rc::MonoTime deadline = now + kActivateTimeout;
  OpSlot& slot = post(OpKind::kActivate);
  const std::uint64_t id = slot.op_id;
  backend_->activate(OpTicket{id, deadline}, now);
  refresh();
  return OpHandle{id};
}

OpHandle Hal::arm(const ArmRequest& request, rc::MonoTime now) {
  refresh();
  if (!active_) return refuse_locally(OpKind::kArm, OpResult{false, "not active", {}});
  if (request.hold_until <= now || request.hold_until > now + kMaxHold) {
    return refuse_locally(OpKind::kArm, OpResult{false, "hold_until not in (now, now + 1 s]", {}});
  }
  const rc::MonoTime deadline = now + kArmTimeout;
  OpSlot& slot = post(OpKind::kArm);
  slot.request = request;
  const std::uint64_t id = slot.op_id;
  arm_state_ = ArmState::kArming;
  pending_arm_op_ = id;
  adopted_.reset();
  last_submitted_.reset();
  guard_.reset_reference();
  backend_->arm(OpTicket{id, deadline}, request, now);
  refresh();
  return OpHandle{id};
}

OpHandle Hal::deactivate(rc::MonoTime now) {
  active_ = false;
  arm_state_ = ArmState::kIdle;
  pending_arm_op_ = 0;
  adopted_.reset();
  last_submitted_.reset();
  guard_.reset_reference();
  const rc::MonoTime deadline = now + kDeactivateTimeout;
  OpSlot& slot = post(OpKind::kDeactivate);
  const std::uint64_t id = slot.op_id;
  backend_->deactivate(OpTicket{id, deadline}, now);
  refresh();
  return OpHandle{id};
}

void Hal::on_error_nonblocking() { backend_->on_error_nonblocking(); }

OpState Hal::op_state(OpHandle handle) {
  refresh();
  const OpSlot* slot = find(handle.op_id);
  return slot == nullptr ? OpState::kUnknown : slot->state;
}

std::optional<OpResult> Hal::wait(OpHandle handle, rc::Duration timeout) {
  refresh();
  OpSlot* slot = find(handle.op_id);
  if (slot == nullptr) return OpResult{false, "unknown operation", {}};
  if (slot->local) return local_results_[static_cast<std::size_t>(slot->op_id % kOpSlots)];
  std::optional<OpResult> result = backend_->wait(handle.op_id, timeout);
  refresh();
  return result;
}

ArmState Hal::arm_state() {
  refresh();
  return arm_state_;
}

std::optional<AuthorityId> Hal::adopted() {
  refresh();
  return adopted_;
}

HalVerdict Hal::refuse(HalVerdict verdict) {
  if (verdict != HalVerdict::kDuplicate) guard_.reset_reference();
  return verdict;
}

HalVerdict Hal::write(const rc::MotionPermit* permit, const HalSetpoint& setpoint,
                      rc::MonoTime now) {
  refresh();
  if (backend_->inhibited()) return refuse(HalVerdict::kInhibited);
  if (!active_) return refuse(HalVerdict::kHardwareNotActive);
  if (arm_state_ != ArmState::kArmed || !adopted_.has_value()) {
    return refuse(HalVerdict::kNotArmed);
  }
  const AuthorityId& id = *adopted_;
  if (permit == nullptr) return refuse(HalVerdict::kNoPermit);
  if (permit->manager != id.manager || permit->session != id.session ||
      permit->generation != id.generation) {
    return refuse(HalVerdict::kIdentityMismatch);
  }
  if (setpoint.id != id || setpoint.snapshot.generation != setpoint.id.generation) {
    return refuse(HalVerdict::kIdentityMismatch);
  }
  if (last_submitted_.has_value() && last_submitted_->id == setpoint.id) {
    if (setpoint.snapshot.source_seq == last_submitted_->source_seq) {
      return refuse(setpoint.snapshot.position == last_submitted_->position
                        ? HalVerdict::kDuplicate
                        : HalVerdict::kDuplicateMismatch);
    }
    if (setpoint.snapshot.source_seq < last_submitted_->source_seq) {
      return refuse(HalVerdict::kRegression);
    }
  }

  rc::GuardInput<kJoints> in{};
  in.hardware_active = true;
  in.permit = permit;
  in.current_manager = id.manager;
  in.current_session = id.session;
  in.setpoint = setpoint.snapshot;
  // A skewed snapshot is stale by construction: one nanosecond past the age the guard allows.
  in.feedback_sampled_at = feedback_.skewed ? now - guard_config_.max_feedback_age - rc::Duration{1}
                                            : feedback_.sampled_at;
  in.feedback_position = feedback_.position;
  const rc::GuardVerdict guard_verdict = guard_.check(in, now);
  if (guard_verdict != rc::GuardVerdict::kSend) return refuse(from_guard(guard_verdict));

  const rc::MonoTime valid_until = rc::clamp_expiry(now + lease_, permit->expires_at);
  if (!backend_->can_apply(valid_until, now)) {
    return refuse(HalVerdict::kExpiresBeforeApplication);
  }
  ApprovedCommand command{};
  command.id = setpoint.id;
  command.source_seq = setpoint.snapshot.source_seq;
  command.position = setpoint.snapshot.position;
  command.approved_at = now;
  command.valid_until = valid_until;
  backend_->submit(command, now);
  last_submitted_ = LastSubmitted{command.id, command.source_seq, command.position};
  return HalVerdict::kSend;
}

Feedback Hal::read(rc::MonoTime now) {
  refresh();
  feedback_ = backend_->poll(now);
  return feedback_;
}

}  // namespace hal_contract
