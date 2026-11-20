// SPDX-License-Identifier: Apache-2.0
// Runs the backend contract suite against a reference fake, so the suite is proven before any
// real backend exists. The fake is deliberately simple but an honest model of the contract:
// it holds on arm, tracks approved commands while they are valid, brakes at a_stop on expiry or
// 30 ms without a new command and latches, inhibits the group on any unexpected stop (C17), and
// confirms each node's stop from fresh samples (C18).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "hal_contract/backend.hpp"
#include "hal_contract/records.hpp"
#include "hal_contract/testing/contract_suite.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/deadline.hpp"
#include "runtime_core/time.hpp"

namespace hc = hal_contract;
namespace ht = hal_contract::testing;
namespace rc = runtime_core;

namespace {

constexpr rc::Duration kW = ht::kW;
constexpr double kWSeconds = 0.002;
constexpr double kAStop = 10.0;  // rad/s^2
constexpr rc::Duration kCommandTimeout = rc::milliseconds(30);
constexpr rc::Duration kMaxFeedbackAge = ht::kMaxFeedbackAge;
constexpr int kConfirmSamples = 5;

class ReferenceBackend final : public hc::Backend {
 public:
  ReferenceBackend(const hc::Joints& start, rc::MonoTime now) : now_(now) {
    for (std::size_t i = 0; i < hc::kJoints; ++i) nodes_[i].x = start[i];
  }

  // ---- fixture side ----

  // One device period W at time t: arm and deactivate phases, command intake, motion,
  // sampling, then the backend's own checks (stop confirmation, feedback age).
  void step(rc::MonoTime t) {
    now_ = t;
    progress_arm(t);
    progress_deactivate(t);
    take_command(t);
    local_stops(t);
    move(t);
    sample(t);
    confirm_stops(t);
    check_feedback_age(t);
  }

  ht::DeviceState device() const {
    ht::DeviceState d{};
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      d.position[i] = nodes_[i].x;
      d.velocity[i] = nodes_[i].v;
    }
    return d;
  }

  std::vector<ht::DeviceEvent> drain() {
    std::vector<ht::DeviceEvent> out;
    out.swap(events_);
    return out;
  }

  void stall(bool on) { stalled_ = on; }
  void delay(rc::Duration d) { delay_ = d; }
  void drop_commands() { dropping_ = true; }
  void inject_stop(std::size_t node) { nodes_.at(node).inject = true; }

  // ---- Backend ----

  void configure(const hc::OpTicket& t, rc::MonoTime /*now*/) override {
    // A stop still being confirmed keeps its tracking: configure would erase it, so it is refused.
    if (std::any_of(nodes_.begin(), nodes_.end(),
                    [](const Node& n) { return n.stop == hc::StopStatus::kStopping; })) {
      ops_[t.op_id] = hc::OpState::kFailed;
      return;
    }
    inhibited_ = false;
    faults_ = hc::FaultSet{};
    active_ = false;
    armed_.reset();
    // Ops still pending are superseded: each ends instead of staying pending for good.
    if (arm_op_.has_value()) ops_[arm_op_->ticket.op_id] = hc::OpState::kFailed;
    if (deactivate_op_.has_value()) ops_[deactivate_op_->op_id] = hc::OpState::kFailed;
    arm_op_.reset();
    deactivate_op_.reset();
    box_.reset();
    command_.reset();
    dropping_ = false;
    for (Node& n : nodes_) {
      n.mode = Mode::kDisabled;
      n.v = 0.0;
      n.stop = hc::StopStatus::kNone;
      n.inject = false;
    }
    configured_ = true;
    ops_[t.op_id] = hc::OpState::kSucceeded;
  }
  rc::DeviceCapabilities capabilities() const override { return rc::virtual_canopen_profile(); }
  void activate(const hc::OpTicket& t, rc::MonoTime now) override {
    supersede_deactivate(now);
    active_ = configured_ && !inhibited_;
    if (active_) {
      // A new cycle starts with no stop result.
      for (Node& n : nodes_) {
        n.stop = hc::StopStatus::kNone;
        n.rest_samples = 0;
      }
    }
    ops_[t.op_id] = active_ ? hc::OpState::kSucceeded : hc::OpState::kFailed;
  }
  void arm(const hc::OpTicket& t, const hc::ArmRequest& request, rc::MonoTime now) override {
    if (arm_op_.has_value()) ops_[arm_op_->ticket.op_id] = hc::OpState::kFailed;
    arm_op_.reset();
    supersede_deactivate(now);
    if (!active_ || inhibited_) {
      ops_[t.op_id] = hc::OpState::kFailed;
      return;
    }
    ops_[t.op_id] = hc::OpState::kPending;
    arm_op_ = ArmOp{t, request, false};
    armed_.reset();
    box_.reset();
    command_.reset();
  }
  void submit(const hc::ApprovedCommand& command, rc::MonoTime /*now*/) override {
    if (dropping_ || !armed_.has_value() || inhibited_) return;
    if (box_.has_value()) ++counters_.superseded;
    box_ = command;
  }
  hc::Feedback poll(rc::MonoTime /*now*/) override {
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
  void deactivate(const hc::OpTicket& t, rc::MonoTime now) override {
    // A deactivate over a pending one supersedes it: the first fails and latches.
    fail_deactivate(now);
    // A pending arm is superseded: it fails and never enables, and the failed op latches (C17).
    if (arm_op_.has_value()) {
      ops_[arm_op_->ticket.op_id] = hc::OpState::kFailed;
      arm_op_.reset();
      latch(now);
    }
    deactivate_op_ = t;
    ops_[t.op_id] = hc::OpState::kPending;
    armed_.reset();
    box_.reset();
    command_.reset();
  }
  void on_error_nonblocking() override { latch(now_); }
  hc::OpState op_state(std::uint64_t id) const override {
    const auto it = ops_.find(id);
    return it == ops_.end() ? hc::OpState::kUnknown : it->second;
  }
  std::optional<hc::OpResult> wait(std::uint64_t id, rc::Duration /*timeout*/) override {
    const hc::OpState s = op_state(id);
    if (s == hc::OpState::kPending) return std::nullopt;  // logic time: nothing to wait on
    return hc::OpResult{s == hc::OpState::kSucceeded, {}, {}};
  }
  bool inhibited() const override { return inhibited_; }
  // Applied at the next step, which is at most W away: it must still be open then.
  bool can_apply(rc::MonoTime valid_until, rc::MonoTime now) const override {
    return valid_until > now + kW;
  }

 private:
  enum class Mode { kDisabled, kHolding, kTracking, kDisabling, kStopping };

  struct Node {
    double x{0.0};
    double v{0.0};
    Mode mode{Mode::kDisabled};
    hc::StopStatus stop{hc::StopStatus::kNone};
    int rest_samples{0};
    rc::MonoTime stop_from{};  // when the stop being confirmed began
    bool inject{false};
    bool stopping_now{false};
  };

  struct ArmOp {
    hc::OpTicket ticket{};
    hc::ArmRequest request{};
    bool disabled{false};
  };

  struct Sample {
    rc::MonoTime at{};
    hc::Joints position{};
    hc::Joints velocity{};
    std::optional<std::uint64_t> applied{};
  };

  static bool enabled(Mode m) { return m == Mode::kHolding || m == Mode::kTracking; }

  void emit(rc::MonoTime t, std::size_t i, ht::DeviceEventKind kind, double target = 0.0,
            std::optional<std::uint64_t> seq = std::nullopt, hc::AuthorityId id = {}) {
    events_.push_back(ht::DeviceEvent{t, i, kind, target, nodes_[i].x, nodes_[i].v, seq, id});
  }

  void track(Node& n, double target) {
    const double vmax = ht::kVMax;
    n.v = std::clamp((target - n.x) / kWSeconds, -vmax, vmax);
    n.x += n.v * kWSeconds;
  }
  static void brake(Node& n) {
    const double dv = std::min(std::abs(n.v), kAStop * kWSeconds);
    n.v -= std::copysign(dv, n.v);
    n.x += n.v * kWSeconds;
  }

  // A failed deactivate latches the inhibit (C17).
  void fail_deactivate(rc::MonoTime t) {
    if (!deactivate_op_.has_value()) return;
    ops_[deactivate_op_->op_id] = hc::OpState::kFailed;
    deactivate_op_.reset();
    latch(t);
  }
  // An activate or arm posted while a deactivate is pending supersedes it, so the new op fails.
  void supersede_deactivate(rc::MonoTime t) { fail_deactivate(t); }

  void fail_arm() {
    if (!arm_op_.has_value()) return;
    ops_[arm_op_->ticket.op_id] = hc::OpState::kFailed;
    faults_.set(hc::Fault::kArmFailed);
    arm_op_.reset();
  }

  // The group inhibit (C17): every enabled axis gets a quick stop; a disabled axis is reported
  // not_enabled; a pending arm fails.
  void latch(rc::MonoTime t) {
    if (inhibited_) return;
    inhibited_ = true;
    box_.reset();
    command_.reset();
    fail_arm();
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      Node& n = nodes_[i];
      n.rest_samples = 0;
      if (enabled(n.mode) || n.mode == Mode::kDisabling) {
        n.mode = Mode::kStopping;
        if (n.stop != hc::StopStatus::kStopping) n.stop_from = t;
        n.stop = hc::StopStatus::kStopping;
        emit(t, i, ht::DeviceEventKind::kQuickStop);
      } else if (n.mode == Mode::kStopping) {
        n.stop = hc::StopStatus::kStopping;
      } else {
        n.stop = hc::StopStatus::kNotEnabled;
      }
    }
  }

  // Re-arm phases, group-wide (C9): disable every enabled axis, wait for all at rest, then
  // commit, hold the actual position and enable.
  void progress_arm(rc::MonoTime t) {
    if (!arm_op_.has_value()) return;
    if (t >= arm_op_->ticket.deadline) {
      fail_arm();
      latch(t);
      return;
    }
    if (!arm_op_->disabled) {
      for (std::size_t i = 0; i < hc::kJoints; ++i) {
        if (enabled(nodes_[i].mode)) {
          nodes_[i].mode = Mode::kDisabling;
          emit(t, i, ht::DeviceEventKind::kDisabled);
        }
      }
      arm_op_->disabled = true;
    }
    const bool all_rest =
        std::all_of(nodes_.begin(), nodes_.end(), [](const Node& n) { return n.v == 0.0; });
    if (!all_rest) return;
    const hc::ArmRequest& r = arm_op_->request;
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      Node& n = nodes_[i];
      emit(t, i, ht::DeviceEventKind::kCommitted, 0.0, std::nullopt, r.id);
      n.mode = Mode::kHolding;
      n.stop = hc::StopStatus::kNone;
      hold_[i] = n.x;
      emit(t, i, ht::DeviceEventKind::kEnabled, 0.0, std::nullopt, r.id);
    }
    armed_ = r.id;
    hold_until_ = r.hold_until;
    ops_[arm_op_->ticket.op_id] = hc::OpState::kSucceeded;
    arm_op_.reset();
  }

  // C18: each disabled axis's stop is confirmed before the deactivate succeeds.
  void progress_deactivate(rc::MonoTime t) {
    if (!deactivate_op_.has_value()) return;
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      Node& n = nodes_[i];
      if (enabled(n.mode)) {
        n.mode = Mode::kDisabling;
        emit(t, i, ht::DeviceEventKind::kDisabled);
        if (!inhibited_) {
          n.stop = hc::StopStatus::kStopping;
          n.stop_from = t;
          n.rest_samples = 0;
        }
      }
    }
    if (!std::all_of(nodes_.begin(), nodes_.end(), [](const Node& n) { return n.v == 0.0; }) ||
        std::any_of(nodes_.begin(), nodes_.end(),
                    [](const Node& n) { return n.stop == hc::StopStatus::kStopping; })) {
      return;
    }
    if (std::any_of(nodes_.begin(), nodes_.end(),
                    [](const Node& n) { return n.stop == hc::StopStatus::kUnconfirmed; })) {
      fail_deactivate(t);
      return;
    }
    for (Node& n : nodes_) n.mode = Mode::kDisabled;
    active_ = false;
    ops_[deactivate_op_->op_id] = hc::OpState::kSucceeded;
    deactivate_op_.reset();
  }

  void take_command(rc::MonoTime t) {
    if (!box_.has_value()) return;
    command_ = *box_;
    received_at_ = t;
    box_.reset();
    for (Node& n : nodes_) {
      if (n.mode == Mode::kHolding) n.mode = Mode::kTracking;
    }
  }

  // Every axis decides its own stop before any axis moves, so a stop on one axis in this step
  // reaches the whole group before anything else is actuated.
  void local_stops(rc::MonoTime t) {
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
      n.stop = hc::StopStatus::kStopping;
      n.stop_from = t;
      emit(t, i, ht::DeviceEventKind::kLocalStop);
      any = true;
    }
    if (any) {
      faults_.set(hc::Fault::kPolicyStop);
      latch(t);  // an unexpected Quick Stop Active inhibits the group (C17)
    }
  }

  void move(rc::MonoTime t) {
    applied_.reset();
    bool all_tracked = command_.has_value();
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      Node& n = nodes_[i];
      switch (n.mode) {
        case Mode::kHolding:
          track(n, hold_[i]);
          emit(t, i, ht::DeviceEventKind::kActuated, hold_[i], std::nullopt, *armed_);
          all_tracked = false;
          break;
        case Mode::kTracking:
          track(n, command_->position[i]);
          emit(t, i, ht::DeviceEventKind::kActuated, command_->position[i], command_->source_seq,
               command_->id);
          break;
        case Mode::kDisabling:
        case Mode::kStopping:
          brake(n);
          all_tracked = false;
          break;
        case Mode::kDisabled:
          all_tracked = false;
          break;
      }
    }
    if (all_tracked) applied_ = command_->source_seq;
  }

  void sample(rc::MonoTime t) {
    Sample s{t, {}, {}, applied_};
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      s.position[i] = nodes_[i].x;
      s.velocity[i] = nodes_[i].v;
    }
    if (!stalled_) in_flight_.push_back(s);
    fresh_ = false;
    while (!in_flight_.empty() && in_flight_.front().at + delay_ <= t) {
      latest_ = in_flight_.front();
      in_flight_.pop_front();
      fresh_ = true;
    }
  }

  // C18: 5 distinct fresh samples at rest; the deadline runs from the start of the stop.
  // Without fresh samples nothing is confirmed.
  void confirm_stops(rc::MonoTime t) {
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      Node& n = nodes_[i];
      if (n.stop != hc::StopStatus::kStopping) continue;
      // The deadline first: a sample that arrives past it confirms nothing, as on CANopen.
      if (t - n.stop_from >= ht::kStopConfirmDeadline) {
        n.stop = hc::StopStatus::kUnconfirmed;
        faults_.set(hc::Fault::kStopUnconfirmed);
        continue;
      }
      // A sample older than max_feedback_age when it arrives is not fresh (C18).
      if (fresh_ && t - latest_->at <= kMaxFeedbackAge) {
        n.rest_samples =
            std::abs(latest_->velocity[i]) < ht::kRestVelocity ? n.rest_samples + 1 : 0;
        if (n.rest_samples >= kConfirmSamples) n.stop = hc::StopStatus::kConfirmed;
      }
    }
  }

  void check_feedback_age(rc::MonoTime t) {
    if (!armed_.has_value() || inhibited_) return;
    if (!latest_.has_value() || t - latest_->at > kMaxFeedbackAge) latch(t);
  }

  rc::MonoTime now_;
  std::array<Node, hc::kJoints> nodes_{};
  hc::Joints hold_{};
  rc::MonoTime hold_until_{};
  bool configured_{false};
  bool active_{false};
  bool inhibited_{false};
  hc::FaultSet faults_{};
  hc::CommandCounters counters_{};
  std::optional<hc::AuthorityId> armed_{};
  std::optional<ArmOp> arm_op_{};
  std::optional<hc::OpTicket> deactivate_op_{};
  std::optional<hc::ApprovedCommand> box_{};
  std::optional<hc::ApprovedCommand> command_{};
  rc::MonoTime received_at_{};
  std::optional<std::uint64_t> applied_{};
  bool dropping_{false};
  bool stalled_{false};
  rc::Duration delay_{};
  std::deque<Sample> in_flight_;
  std::optional<Sample> latest_{};
  bool fresh_{false};
  std::map<std::uint64_t, hc::OpState> ops_;
  std::vector<ht::DeviceEvent> events_;
};

class ReferenceFixture {
 public:
  struct Traits {
    static constexpr rc::Duration kArmLatency = rc::milliseconds(2);         // 1 W from rest
    static constexpr rc::Duration kAcceptanceLatency = rc::milliseconds(2);  // 1 W
  };

  std::unique_ptr<hc::Backend> make() {
    auto backend = std::make_unique<ReferenceBackend>(ht::kStartPosition, now_);
    backend_ = backend.get();
    return backend;
  }
  rc::MonoTime now() const { return now_; }
  void advance(rc::Duration d) {
    const rc::MonoTime until = now_ + d;
    while (next_step_ <= until) {
      backend_->step(next_step_);
      next_step_ = next_step_ + kW;
    }
    now_ = until;
  }
  void stall_feedback(bool on) { backend_->stall(on); }
  void delay_feedback(rc::Duration d) { backend_->delay(d); }
  void stop_accepting_commands() { backend_->drop_commands(); }
  void inject_node_stop(std::size_t node) { backend_->inject_stop(node); }
  ht::DeviceState device() const { return backend_->device(); }
  std::vector<ht::DeviceEvent> drain_events() { return backend_->drain(); }

 private:
  rc::MonoTime now_{1'000'000'000};
  rc::MonoTime next_step_{now_ + kW};
  ReferenceBackend* backend_{nullptr};  // owned by the Hal under test
};

static_assert(ht::BackendFixture<ReferenceFixture>);

}  // namespace

namespace hal_contract::testing {
using ReferenceFixtures = ::testing::Types<ReferenceFixture>;
INSTANTIATE_TYPED_TEST_SUITE_P(Reference, Contract, ReferenceFixtures);
}  // namespace hal_contract::testing
