// SPDX-License-Identifier: Apache-2.0
#include "direct_backend/direct_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "runtime_core/deadline.hpp"

namespace direct_backend {

namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

constexpr double kNradPerRad = 1e9;
// A start position or a target beyond this is not a joint angle; targets are clamped to it.
constexpr double kMaxRad = 100.0;
constexpr int kPlacementIterations = 100000;

std::int64_t to_nrad(double rad) {
  return std::llround(std::clamp(rad, -kMaxRad, kMaxRad) * kNradPerRad);
}
double to_rad(std::int64_t nrad) { return static_cast<double>(nrad) / kNradPerRad; }

}  // namespace

DirectBackend::DirectBackend(const DirectConfig& config)
    : max_feedback_age_(config.max_feedback_age) {
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    const double x = config.start_position[i];
    if (!std::isfinite(x) || std::abs(x) > kMaxRad) {
      throw std::invalid_argument("direct_backend: start position out of range");
    }
    // Time has not started: drive the joint model to its place until it rests there exactly.
    const std::int64_t target = to_nrad(x);
    drive_core::JointModel& model = nodes_[i].model;
    for (int k = 0; k < kPlacementIterations; ++k) {
      if (model.state().position_nrad == target && model.state().velocity_nrad_s == 0) break;
      model.track(target);
    }
    if (model.state().position_nrad != target || model.state().velocity_nrad_s != 0) {
      throw std::invalid_argument("direct_backend: could not place a joint");
    }
  }
}

void DirectBackend::set_event_sink(EventSink sink, void* context) {
  sink_ = sink;
  sink_context_ = context;
}

hc::Joints DirectBackend::joint_position() const {
  hc::Joints out{};
  for (std::size_t i = 0; i < hc::kJoints; ++i)
    out[i] = to_rad(nodes_[i].model.state().position_nrad);
  return out;
}

hc::Joints DirectBackend::joint_velocity() const {
  hc::Joints out{};
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    out[i] = to_rad(nodes_[i].model.state().velocity_nrad_s);
  }
  return out;
}

void DirectBackend::inject_stop(std::size_t node) { nodes_.at(node).inject = true; }

void DirectBackend::set_drive_available(std::size_t node, bool available) {
  nodes_.at(node).drive_available = available;
}

// ---- ops ----

void DirectBackend::set_op(std::uint64_t id, hc::OpState state, const char* error) {
  ops_[id % kOpRing] = OpEntry{id, state, error};
}

hc::OpState DirectBackend::op_state(std::uint64_t op_id) const {
  const OpEntry& e = ops_[op_id % kOpRing];
  return e.id == op_id ? e.state : hc::OpState::kUnknown;
}

std::optional<hc::OpResult> DirectBackend::wait(std::uint64_t op_id, rc::Duration /*timeout*/) {
  const OpEntry& e = ops_[op_id % kOpRing];
  if (e.id != op_id || e.state == hc::OpState::kUnknown) {
    return hc::OpResult{false, "unknown operation", {}};
  }
  if (e.state == hc::OpState::kPending) return std::nullopt;  // logic time: nothing to wait on
  return hc::OpResult{e.state == hc::OpState::kSucceeded, e.error != nullptr ? e.error : "", {}};
}

void DirectBackend::configure(const hc::OpTicket& t, rc::MonoTime now) {
  now_ = now;
  if (max_feedback_age_ < kMinFeedbackAge || max_feedback_age_ > kMaxFeedbackAge) {
    configured_ = false;
    set_op(t.op_id, hc::OpState::kFailed,
           "max_feedback_age must be in [20, 30] ms (the E3a drive profile)");
    return;
  }
  const bool stopping = std::any_of(nodes_.begin(), nodes_.end(), [](const Node& n) {
    return n.stop == hc::StopStatus::kStopping;
  });
  if (stopping) {
    // C18: the stop keeps its tracking until it has an outcome.
    // Configure would erase it before that, so it is refused and nothing here changes.
    set_op(t.op_id, hc::OpState::kFailed,
           "configure refused while a stop is still being confirmed: it would erase the stop "
           "before its outcome is known");
    return;
  }
  // A configure supersedes the ops still pending: each ends instead of staying pending for good.
  if (arm_op_.has_value()) {
    set_op(arm_op_->ticket.op_id, hc::OpState::kFailed, "superseded by a configure");
    arm_op_.reset();
  }
  if (deactivate_op_.has_value()) {
    set_op(deactivate_op_->op_id, hc::OpState::kFailed, "superseded by a configure");
    deactivate_op_.reset();
  }
  inhibited_ = false;
  faults_ = hc::FaultSet{};
  active_ = false;
  armed_.reset();
  box_.reset();
  command_.reset();
  dropping_ = false;
  for (Node& n : nodes_) {
    // A joint still moving brakes to rest; the others are Switched On.
    n.mode = n.model.state().velocity_nrad_s != 0 ? Mode::kDisabling : Mode::kDisabled;
    n.stop = hc::StopStatus::kNone;
    n.rest_samples = 0;
    n.accept = 0;
    n.inject = false;
  }
  configured_ = true;
  set_op(t.op_id, hc::OpState::kSucceeded);
}

rc::DeviceCapabilities DirectBackend::capabilities() const { return rc::direct_backend_profile(); }

void DirectBackend::activate(const hc::OpTicket& t, rc::MonoTime now) {
  now_ = now;
  supersede_deactivate(now);
  active_ = configured_ && !inhibited_;
  if (active_) {
    // A new cycle starts with no stop result, as on CANopen.
    for (Node& n : nodes_) {
      n.stop = hc::StopStatus::kNone;
      n.rest_samples = 0;
      n.accept = 0;
    }
    set_op(t.op_id, hc::OpState::kSucceeded);
  } else {
    set_op(t.op_id, hc::OpState::kFailed, "not configured or the group inhibit is latched");
  }
}

void DirectBackend::arm(const hc::OpTicket& t, const hc::ArmRequest& request, rc::MonoTime now) {
  now_ = now;
  if (arm_op_.has_value()) {  // a newer arm supersedes a pending one; that is not an arm fault
    set_op(arm_op_->ticket.op_id, hc::OpState::kFailed, "superseded by a newer arm");
    arm_op_.reset();
  }
  supersede_deactivate(now);
  if (!active_ || inhibited_) {
    set_op(t.op_id, hc::OpState::kFailed, "not active or the group inhibit is latched");
    return;
  }
  set_op(t.op_id, hc::OpState::kPending);
  arm_op_ = ArmOp{t, request, false};
  armed_.reset();
  box_.reset();
  command_.reset();
}

void DirectBackend::deactivate(const hc::OpTicket& t, rc::MonoTime now) {
  now_ = now;
  // A deactivate over a pending one supersedes it, as on CANopen: the first fails and latches.
  fail_deactivate("superseded by a newer op", now);
  if (arm_op_.has_value()) {
    // The deactivate supersedes a pending arm, which fails and never enables.
    // A failed op latches the inhibit (C17), as on CANopen.
    fail_arm("superseded by a deactivate");
    latch(now);
  }
  armed_.reset();
  box_.reset();
  command_.reset();
  // Not active and every stop resolved: already Switched On or lower.
  // A stop still being confirmed, an unconfirmed stop or a joint still moving goes through the
  // pending path instead, which waits for the outcome and fails on an unconfirmed stop (C18).
  const bool unresolved = std::any_of(nodes_.begin(), nodes_.end(), [](const Node& n) {
    return n.stop == hc::StopStatus::kStopping || n.stop == hc::StopStatus::kUnconfirmed;
  });
  if (!active_ && !unresolved && all_at_rest()) {
    set_op(t.op_id, hc::OpState::kSucceeded);
    return;
  }
  deactivate_op_ = t;
  set_op(t.op_id, hc::OpState::kPending);
  if (inhibited_) return;  // The inhibit already tracks every stop.
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    if (enabled(n.mode)) {
      n.mode = Mode::kDisabling;
      emit(now, i, EventKind::kDisabled);
      begin_stop(i, kAcceptSwitchedOn, now);
    } else if (n.stop == hc::StopStatus::kNone) {
      n.stop = hc::StopStatus::kNotEnabled;
    }
  }
}

void DirectBackend::on_error_nonblocking() { latch(now_); }

void DirectBackend::submit(const hc::ApprovedCommand& command, rc::MonoTime /*now*/) {
  // Only the armed identity reaches the joints, as on CANopen (defence in depth behind the Hal).
  if (dropping_ || !armed_.has_value() || !(command.id == *armed_) || inhibited_) return;
  for (const double x : command.position) {
    if (!std::isfinite(x)) return;
  }
  if (box_.has_value()) ++counters_.superseded;
  box_ = command;
}

bool DirectBackend::can_apply(rc::MonoTime valid_until, rc::MonoTime now) const {
  return valid_until > now + kStep;
}

hc::Feedback DirectBackend::poll(rc::MonoTime /*now*/) {
  hc::Feedback fb{};
  if (latest_.has_value()) {
    fb.position = latest_->position;
    fb.velocity = latest_->velocity;
    fb.sampled_at = latest_->at;
    fb.node_sampled_at.fill(latest_->at);
    fb.group_applied_source_seq = latest_->applied;
  }
  fb.faults = faults_;
  if (inhibited_) fb.faults.set(hc::Fault::kInhibited);
  for (std::size_t i = 0; i < hc::kJoints; ++i) fb.stop[i] = nodes_[i].stop;
  fb.counters = counters_;
  return fb;
}

// ---- the device period ----

void DirectBackend::emit(rc::MonoTime t, std::size_t i, EventKind kind, double target,
                         std::optional<std::uint64_t> seq, hc::AuthorityId id) {
  if (sink_ == nullptr) return;
  const drive_core::JointState& s = nodes_[i].model.state();
  sink_(sink_context_,
        Event{t, i, kind, target, to_rad(s.position_nrad), to_rad(s.velocity_nrad_s), seq, id});
}

bool DirectBackend::all_at_rest() const {
  return std::all_of(nodes_.begin(), nodes_.end(),
                     [](const Node& n) { return n.model.state().velocity_nrad_s == 0; });
}

void DirectBackend::step(rc::MonoTime t) {
  now_ = t;
  progress_arm(t);
  progress_deactivate(t);
  take_command(t);
  local_stops(t);
  move(t);
  // The deadline is judged before this step's samples are counted, as on CANopen.
  // A step that jumps past the deadline cannot confirm a stop with the samples it delivers.
  judge_stops(t);
  sample(t);
  check_feedback_age(t);
}

void DirectBackend::fail_arm(const char* error) {
  if (!arm_op_.has_value()) return;
  set_op(arm_op_->ticket.op_id, hc::OpState::kFailed, error);
  faults_.set(hc::Fault::kArmFailed);
  arm_op_.reset();
}

// The group inhibit (C17): every enabled joint gets a quick stop; a joint never enabled is
// reported not_enabled; a pending arm fails.
void DirectBackend::latch(rc::MonoTime t) {
  if (inhibited_) return;
  inhibited_ = true;
  box_.reset();
  command_.reset();
  fail_arm("the group inhibit latched during the arm");
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    n.rest_samples = 0;
    if (enabled(n.mode) || n.mode == Mode::kDisabling) {
      n.mode = Mode::kStopping;
      emit(t, i, EventKind::kQuickStop);
    }
    // A stop that already has its outcome keeps it.
    if (n.stop == hc::StopStatus::kConfirmed || n.stop == hc::StopStatus::kUnconfirmed) continue;
    if (n.mode != Mode::kStopping) {
      n.stop = hc::StopStatus::kNotEnabled;
    } else if (n.stop == hc::StopStatus::kStopping) {
      n.accept = static_cast<std::uint8_t>(n.accept | kAcceptQuickStop);
    } else {
      begin_stop(i, kAcceptQuickStop, t);
    }
  }
}

void DirectBackend::begin_stop(std::size_t i, std::uint8_t accept, rc::MonoTime t) {
  Node& n = nodes_[i];
  n.stop = hc::StopStatus::kStopping;
  n.rest_samples = 0;
  n.stop_from = t;
  n.accept = accept;
}

bool DirectBackend::accepts(std::uint8_t accept, Mode m) {
  if (m == Mode::kStopping) return (accept & kAcceptQuickStop) != 0U;
  if (m == Mode::kDisabling || m == Mode::kDisabled) return (accept & kAcceptSwitchedOn) != 0U;
  return false;
}

// Re-arm phases, group-wide (C9): disable every enabled joint, wait for all at rest, then
// commit, hold the actual position and enable.
void DirectBackend::progress_arm(rc::MonoTime t) {
  if (!arm_op_.has_value()) return;
  if (t >= arm_op_->ticket.deadline) {
    fail_arm("arm deadline passed");
    latch(t);
    return;
  }
  if (!arm_op_->disabled) {
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      if (enabled(nodes_[i].mode)) {
        nodes_[i].mode = Mode::kDisabling;
        emit(t, i, EventKind::kDisabled);
      }
    }
    arm_op_->disabled = true;
  }
  if (!all_at_rest()) return;
  const hc::ArmRequest& r = arm_op_->request;
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    emit(t, i, EventKind::kCommitted, 0.0, std::nullopt, r.id);
    n.mode = Mode::kHolding;
    n.stop = hc::StopStatus::kNone;
    n.hold_nrad = n.model.state().position_nrad;
    emit(t, i, EventKind::kEnabled, 0.0, std::nullopt, r.id);
  }
  armed_ = r.id;
  hold_until_ = r.hold_until;
  set_op(arm_op_->ticket.op_id, hc::OpState::kSucceeded);
  arm_op_.reset();
}

void DirectBackend::progress_deactivate(rc::MonoTime t) {
  if (!deactivate_op_.has_value()) return;
  const std::uint64_t id = deactivate_op_->op_id;
  if (t >= deactivate_op_->deadline) {
    fail_deactivate("deactivate deadline passed", t);
    return;
  }
  const bool confirming = std::any_of(nodes_.begin(), nodes_.end(), [](const Node& n) {
    return n.stop == hc::StopStatus::kStopping;
  });
  if (!all_at_rest() || confirming) return;
  const bool unconfirmed = std::any_of(nodes_.begin(), nodes_.end(), [](const Node& n) {
    return n.stop == hc::StopStatus::kUnconfirmed;
  });
  if (unconfirmed) {
    fail_deactivate("a stop was not confirmed", t);
    return;
  }
  for (Node& n : nodes_) n.mode = Mode::kDisabled;
  active_ = false;
  set_op(id, hc::OpState::kSucceeded);
  deactivate_op_.reset();
}

// An activate or arm posted while a deactivate is pending supersedes it, as on CANopen.
// The deactivate fails and latches the inhibit (C17), so the new op fails too.
void DirectBackend::supersede_deactivate(rc::MonoTime t) {
  fail_deactivate("superseded by a newer op", t);
}

// A failed or timed-out op latches the group inhibit (C17).
void DirectBackend::fail_deactivate(const char* error, rc::MonoTime t) {
  if (!deactivate_op_.has_value()) return;
  set_op(deactivate_op_->op_id, hc::OpState::kFailed, error);
  deactivate_op_.reset();
  latch(t);
}

void DirectBackend::take_command(rc::MonoTime t) {
  if (!box_.has_value()) return;
  command_ = *box_;
  received_at_ = t;
  box_.reset();
  for (Node& n : nodes_) {
    if (n.mode == Mode::kHolding) n.mode = Mode::kTracking;
  }
}

// Every joint decides its own stop before any joint moves, so a stop on one joint in this step
// reaches the whole group before anything else is actuated.
void DirectBackend::local_stops(rc::MonoTime t) {
  bool any = false;
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    bool stop = n.inject && enabled(n.mode);
    n.inject = false;
    if (n.mode == Mode::kHolding && rc::is_expired(t, hold_until_)) stop = true;
    if (n.mode == Mode::kTracking && command_.has_value() &&
        (rc::is_expired(t, command_->valid_until) || t - received_at_ >= kCommandTimeout)) {
      stop = true;
    }
    if (!stop) continue;
    n.mode = Mode::kStopping;
    begin_stop(i, kAcceptQuickStop, t);
    emit(t, i, EventKind::kLocalStop);
    any = true;
  }
  if (any) {
    faults_.set(hc::Fault::kPolicyStop);
    latch(t);  // an unexpected Quick Stop Active inhibits the group (C17)
  }
}

void DirectBackend::move(rc::MonoTime t) {
  applied_.reset();
  bool all_tracked = command_.has_value();
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    switch (n.mode) {
      case Mode::kHolding:
        n.model.track(n.hold_nrad);
        emit(t, i, EventKind::kActuated, to_rad(n.hold_nrad), std::nullopt,
             armed_.value_or(hc::AuthorityId{}));
        all_tracked = false;
        break;
      case Mode::kTracking:
        if (!command_.has_value()) {  // a re-arm discarded the command: hold where it is
          n.model.track(n.model.state().position_nrad);
          all_tracked = false;
          break;
        }
        n.model.track(to_nrad(command_->position[i]));
        emit(t, i, EventKind::kActuated, command_->position[i], command_->source_seq, command_->id);
        break;
      case Mode::kDisabling:
      case Mode::kStopping:
        n.model.brake();
        all_tracked = false;
        break;
      case Mode::kDisabled:
        all_tracked = false;
        break;
    }
  }
  if (all_tracked && command_.has_value()) applied_ = command_->source_seq;
}

void DirectBackend::sample(rc::MonoTime t) {
  if (!stalled_) {
    Sample s{};
    s.at = t;
    s.seq = next_sample_seq_++;
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      s.position[i] = to_rad(nodes_[i].model.state().position_nrad);
      s.velocity[i] = to_rad(nodes_[i].model.state().velocity_nrad_s);
      s.mode[i] = nodes_[i].mode;
    }
    s.applied = applied_;
    if (ring_size_ == kSampleRing) {  // the oldest in-flight sample is lost
      ring_head_ = (ring_head_ + 1) % kSampleRing;
      --ring_size_;
    }
    ring_[(ring_head_ + ring_size_) % kSampleRing] = s;
    ++ring_size_;
  }
  while (ring_size_ > 0 && ring_[ring_head_].at + delay_ <= t) {
    latest_ = ring_[ring_head_];
    ring_head_ = (ring_head_ + 1) % kSampleRing;
    --ring_size_;
    count_rest(*latest_);
  }
}

// C18: a stop is confirmed after 5 distinct fresh samples, each at rest with the joint in a mode
// its track accepts (Quick Stop Active for an inhibit, Switched On for a deactivate).
// A sample is counted once, when it arrives, and only if it was taken after the stop began.
// A drive that reports itself unavailable confirms nothing (judge_stops).
void DirectBackend::count_rest(const Sample& s) {
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    Node& n = nodes_[i];
    if (n.stop != hc::StopStatus::kStopping || !n.drive_available || s.at < n.stop_from) continue;
    n.rest_samples = (accepts(n.accept, s.mode[i]) && std::abs(s.velocity[i]) < kRestVelocity)
                         ? n.rest_samples + 1
                         : 0;
    if (n.rest_samples >= kConfirmSamples) n.stop = hc::StopStatus::kConfirmed;
  }
}

// The drive's own evidence first, then the deadline from the start of the stop.
void DirectBackend::judge_stops(rc::MonoTime t) {
  for (Node& n : nodes_) {
    if (n.stop != hc::StopStatus::kStopping) continue;
    if (!n.drive_available || t - n.stop_from >= kStopConfirmDeadline) {
      n.stop = hc::StopStatus::kUnconfirmed;
      faults_.set(hc::Fault::kStopUnconfirmed);
    }
  }
}

void DirectBackend::check_feedback_age(rc::MonoTime t) {
  if (!armed_.has_value() || inhibited_) return;
  if (!latest_.has_value() || t - latest_->at > max_feedback_age_) latch(t);
}

}  // namespace direct_backend
