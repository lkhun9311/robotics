// SPDX-License-Identifier: Apache-2.0
// Logic-time harness (test tooling, not production code).
// Drives the runtime_core contracts with a fake clock, a stub controller and a simplified
// drive, and writes a JSONL trace that the independent Python oracle judges. Defects stand
// in for production bugs so we can show the oracle catches them.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "runtime_core/deadline.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/ids.hpp"
#include "runtime_core/lease.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/permit_issuer.hpp"
#include "runtime_core/policy.hpp"
#include "runtime_core/time.hpp"
#include "runtime_core/trace.hpp"

namespace rc = runtime_core;

namespace {

constexpr std::size_t kJoints = 6;
using Joints = std::array<double, kJoints>;

constexpr rc::Duration kPeriod = rc::milliseconds(1);
constexpr rc::Duration kCommandTtl = rc::milliseconds(10);
constexpr rc::Duration kMaxFeedbackAge = rc::milliseconds(5);
constexpr rc::Duration kDriveTimeout = rc::milliseconds(2);
constexpr rc::Duration kGuardPeriodW = rc::milliseconds(2);
constexpr int kRenewEveryTicks = 10;
// Rest-to-rest bump over the whole duration T: peak velocity A*2*pi/T, peak acceleration
// A*(2*pi/T)^2. With A = 0.05 rad and the shortest T = 500 ms: 0.63 rad/s and 7.9 rad/s^2,
// inside the 1.0 rad/s and 10 rad/s^2 limits.
constexpr double kAmplitudeRad = 0.05;
constexpr rc::ManagerInstance kManager{1};
constexpr double kPi = 3.14159265358979323846;
constexpr double kVelMax = 1.0;      // rad/s, project-declared
constexpr double kAccStop = 10.0;    // rad/s^2, project-declared
constexpr double kAllowedErr = 0.1;  // rad, project-declared

enum class Scenario { kNormal, kManagerStall, kGenerationSwitch, kFaultResetReenable };
enum class Defect {
  kNone,
  kExpiryOffByOne,
  kSkipGenerationCheck,
  kAutoResume,
  kDropApplyEvent,
  kReuseIdentity,
  kReplayBypass,
};

bool expired_off_by_one(rc::MonoTime now, rc::MonoTime valid_until) { return now > valid_until; }

rc::ExpiryPredicate predicate_for(Defect d) {
  return d == Defect::kExpiryOffByOne ? &expired_off_by_one : &rc::is_expired;
}

rc::TraceField f(std::string_view key, std::int64_t value) { return {key, value}; }
rc::TraceField f(std::string_view key, std::uint64_t value) {
  return {key, static_cast<std::int64_t>(value)};
}
rc::TraceField f(std::string_view key, std::string_view value) { return {key, value}; }

rc::MonoTime at_ms(std::int64_t ms) { return rc::MonoTime{ms * 1'000'000}; }

std::optional<Scenario> parse_scenario(std::string_view s) {
  if (s == "normal") return Scenario::kNormal;
  if (s == "manager_stall") return Scenario::kManagerStall;
  if (s == "generation_switch") return Scenario::kGenerationSwitch;
  if (s == "fault_reset_reenable") return Scenario::kFaultResetReenable;
  return std::nullopt;
}

std::optional<Defect> parse_defect(std::string_view s) {
  if (s == "none") return Defect::kNone;
  if (s == "expiry_off_by_one") return Defect::kExpiryOffByOne;
  if (s == "skip_generation_check") return Defect::kSkipGenerationCheck;
  if (s == "auto_resume") return Defect::kAutoResume;
  if (s == "drop_apply_event") return Defect::kDropApplyEvent;
  if (s == "reuse_identity") return Defect::kReuseIdentity;
  if (s == "replay_bypass") return Defect::kReplayBypass;
  return std::nullopt;
}

std::string_view name(Scenario s) {
  switch (s) {
    case Scenario::kNormal:
      return "normal";
    case Scenario::kManagerStall:
      return "manager_stall";
    case Scenario::kGenerationSwitch:
      return "generation_switch";
    case Scenario::kFaultResetReenable:
      return "fault_reset_reenable";
  }
  return "unknown";
}

std::string_view name(Defect d) {
  switch (d) {
    case Defect::kNone:
      return "none";
    case Defect::kExpiryOffByOne:
      return "expiry_off_by_one";
    case Defect::kSkipGenerationCheck:
      return "skip_generation_check";
    case Defect::kAutoResume:
      return "auto_resume";
    case Defect::kDropApplyEvent:
      return "drop_apply_event";
    case Defect::kReuseIdentity:
      return "reuse_identity";
    case Defect::kReplayBypass:
      return "replay_bypass";
  }
  return "unknown";
}

std::int64_t end_ms(Scenario s) {
  switch (s) {
    case Scenario::kNormal:
      return 2200;
    case Scenario::kManagerStall:
      return 1200;
    case Scenario::kGenerationSwitch:
      return 2300;
    case Scenario::kFaultResetReenable:
      return 1500;
  }
  return 0;
}

rc::JointLimits<kJoints> joint_limits() {
  rc::JointLimits<kJoints> l;
  l.pos_min.fill(-2.0 * kPi);
  l.pos_max.fill(2.0 * kPi);
  l.vel_max.fill(kVelMax);
  l.acc_max.fill(kAccStop);
  return l;
}

// Stub controller: a rest-to-rest cosine bump per joint, one generation at a time.
class Controller {
 public:
  void start(rc::Generation generation, rc::MonoTime adopt_at, rc::Duration duration) {
    pending_ = generation;
    adopt_at_ = adopt_at;
    pending_duration_ = duration;
    executing_ = true;
  }
  void stop() {
    executing_ = false;
    pending_.reset();
    done_ = false;
  }
  bool done() const { return done_; }
  bool has_pending() const { return pending_.has_value(); }
  rc::Generation adopted() const { return adopted_; }
  std::uint64_t progress() const { return progress_; }

  std::optional<rc::SetpointSnapshot<kJoints>> step(rc::MonoTime now, const Joints& feedback) {
    if (!executing_) return std::nullopt;
    if (pending_ && now >= adopt_at_) {
      adopted_ = *pending_;
      pending_.reset();
      start_position_ = feedback;
      t0_ = now;
      duration_ = pending_duration_;
      done_ = false;
    }
    ++progress_;
    const rc::Duration elapsed = now - t0_;
    if (!pending_ && elapsed >= duration_) done_ = true;
    // Phase in [0, 1]; before the first adoption duration_ is zero and the phase stays 0.
    const double phase = duration_.ns > 0
                             ? static_cast<double>(std::min(elapsed.ns, duration_.ns)) /
                                   static_cast<double>(duration_.ns)
                             : 0.0;
    const double offset = kAmplitudeRad * (1.0 - std::cos(2.0 * kPi * phase));
    rc::SetpointSnapshot<kJoints> sp;
    sp.generation = adopted_;
    sp.source_seq = ++source_seq_;
    for (std::size_t i = 0; i < kJoints; ++i) sp.position[i] = start_position_[i] + offset;
    return sp;
  }

 private:
  bool executing_{false};
  bool done_{false};
  std::optional<rc::Generation> pending_;
  rc::MonoTime adopt_at_{};
  rc::Duration pending_duration_{};
  rc::Generation adopted_{};
  rc::MonoTime t0_{};
  rc::Duration duration_{};
  Joints start_position_{};
  std::uint64_t progress_{0};
  std::uint64_t source_seq_{0};
};

struct DriveCommand {
  std::uint64_t seq{0};
  rc::SessionId session{};
  rc::Generation generation{};
  rc::MonoTime valid_until{};
  Joints position{};
};

// Simplified drive: applies fresh commands at once, holds the last target otherwise, and
// marks a local stop when no fresh command arrives within kDriveTimeout.
class Drive {
 public:
  Drive(rc::ExpiryPredicate expired, bool emit_apply)
      : expired_(expired), emit_apply_(emit_apply) {}

  // The manager announces the session at ARMING; frames from any other session are stale.
  void set_session(rc::SessionId session) { session_ = session; }

  // A frame that arrives again (e.g. a delayed duplicate after a reconnect). `bypass` stands
  // in for a production bug where the replay path skips validation.
  void redeliver(rc::MonoTime now, const DriveCommand& cmd, bool bypass, rc::TraceSink& trace) {
    trace.emit(now, "command_redelivered", {f("seq", cmd.seq)});
    if (bypass) {
      apply(now, cmd, trace);
    } else {
      validate_and_apply(now, cmd, trace);
    }
  }

  void step(rc::MonoTime now, const std::optional<DriveCommand>& cmd, rc::TraceSink& trace) {
    if (cmd) validate_and_apply(now, *cmd, trace);
    if (!stopped_ && (!has_fresh_ || (now - last_fresh_) >= kDriveTimeout)) {
      stopped_ = true;
      trace.emit(now, "drive_local_stop", {});
    }
    sampled_at_ = now;
  }

  const Joints& position() const { return position_; }
  rc::MonoTime sampled_at() const { return sampled_at_; }
  bool stopped() const { return stopped_; }

 private:
  void validate_and_apply(rc::MonoTime now, const DriveCommand& cmd, rc::TraceSink& trace) {
    if (cmd.session != session_) {
      trace.emit(now, "command_rejected_drive",
                 {f("seq", cmd.seq), f("reason", std::string_view{"stale_session"})});
    } else if (expired_(now, cmd.valid_until)) {
      trace.emit(now, "command_rejected_drive",
                 {f("seq", cmd.seq), f("reason", std::string_view{"expired"})});
    } else {
      apply(now, cmd, trace);
    }
  }

  void apply(rc::MonoTime now, const DriveCommand& cmd, rc::TraceSink& trace) {
    position_ = cmd.position;
    last_fresh_ = now;
    has_fresh_ = true;
    stopped_ = false;
    if (emit_apply_) {
      trace.emit(now, "command_applied",
                 {f("seq", cmd.seq), f("session", cmd.session.value),
                  f("generation", cmd.generation.value), f("valid_until_ns", cmd.valid_until.ns)});
    }
  }

  rc::ExpiryPredicate expired_;
  bool emit_apply_;
  rc::SessionId session_{};
  Joints position_{};
  rc::MonoTime last_fresh_{};
  rc::MonoTime sampled_at_{};
  bool has_fresh_{false};
  bool stopped_{true};
};

class Simulation {
 public:
  Simulation(Scenario scenario, Defect defect, rc::TraceSink& trace, rc::Duration lease)
      : scenario_(scenario),
        defect_(defect),
        trace_(trace),
        lease_(lease),
        issuer_(kManager, lease, kMaxFeedbackAge),
        drive_(predicate_for(defect), defect != Defect::kDropApplyEvent),
        guard_(rc::GuardConfig{kPeriod, kMaxFeedbackAge}, joint_limits(), predicate_for(defect)) {}

  void run() {
    const std::int64_t last = end_ms(scenario_);
    trace_.emit(at_ms(0), "scenario_begin",
                {f("scenario", name(scenario_)), f("defect", name(defect_)),
                 f("harness_version", std::string_view{"1"}), f("lease_ns", lease_.ns),
                 f("stop_deadline_ns", std::int64_t{2} * kGuardPeriodW.ns)});
    for (std::int64_t ms = 0; ms <= last; ++ms) tick(at_ms(ms), ms);
    trace_.emit(at_ms(last), "scenario_end", {});
  }

 private:
  void transition(rc::PolicyEvent e, rc::MonoTime now) {
    const rc::RobotState from = policy_.state();
    const rc::PolicyResult r = policy_.handle(e, observed(now));
    if (!r.accepted) {
      trace_.emit(now, "event_rejected",
                  {f("event", rc::to_string(e)), f("state", rc::to_string(from))});
      return;
    }
    if (r.state != from) {
      trace_.emit(now, "state_transition",
                  {f("from", rc::to_string(from)), f("to", rc::to_string(r.state)),
                   f("event", rc::to_string(e))});
    }
  }

  rc::PolicyPreconditions observed(rc::MonoTime now) const {
    rc::PolicyPreconditions p;
    p.cause_cleared = true;    // harness faults are one-shot injections
    p.epochs_verified = true;  // one plant epoch in logic time
    p.queues_flushed = true;   // the harness keeps no queues
    const rc::Duration age = now - drive_.sampled_at();
    p.fresh_feedback = age.ns >= 0 && age <= kMaxFeedbackAge;
    p.stop_confirmed = drive_.stopped();
    return p;
  }

  void issue(rc::Generation g, rc::MonoTime now) {
    permit_ = issuer_.issue(session_, g, controller_.progress(), now);
    trace_.emit(now, "permit_issued",
                {f("manager", kManager.value), f("session", session_.value),
                 f("generation", g.value), f("expires_ns", permit_->expires_at.ns)});
    ticks_since_renew_ = 0;
    last_generation_ = g;
  }

  void revoke(rc::MonoTime now) {
    if (permit_) {
      trace_.emit(now, "permit_revoked",
                  {f("manager", permit_->manager.value), f("session", permit_->session.value),
                   f("generation", permit_->generation.value)});
      permit_.reset();
    }
    controller_.stop();
  }

  void request_enable(rc::MonoTime now, bool explicit_request, rc::Duration duration) {
    if (explicit_request) trace_.emit(now, "enable_request", {});
    transition(rc::PolicyEvent::kEnableRequest, now);
    if (policy_.state() == rc::RobotState::kArming) {
      if (defect_ == Defect::kReuseIdentity && session_.value != 0) {
        // Defect: recovery keeps the old session and generation instead of new ones.
        arming_generation_ = last_generation_;
      } else {
        session_ = ids_.next_session();
        arming_generation_ = ids_.next_generation();
      }
      drive_.set_session(session_);
      guard_.reset_reference();
      controller_.start(arming_generation_, now, duration);
    }
  }

  void inject_fault(rc::MonoTime now) {
    trace_.emit(now, "fault_injected", {f("name", std::string_view{"injected_fault"})});
    transition(rc::PolicyEvent::kFault, now);
    if (policy_.state() == rc::RobotState::kStopping) revoke(now);
  }

  void request_reset(rc::MonoTime now) {
    trace_.emit(now, "reset_request", {});
    transition(rc::PolicyEvent::kResetRequest, now);
    if (policy_.state() == rc::RobotState::kRecovering) {
      transition(rc::PolicyEvent::kRecoveryVerified, now);
    }
  }

  void switch_generation(rc::MonoTime now) {
    const rc::Generation g = ids_.next_generation();
    issue(g, now);
    controller_.start(g, now + rc::milliseconds(30), rc::milliseconds(1000));
  }

  void scenario_events(rc::MonoTime now, std::int64_t ms) {
    if (ms == 0) {
      transition(rc::PolicyEvent::kBootOk, now);
      transition(rc::PolicyEvent::kInitReady, now);
      return;
    }
    switch (scenario_) {
      case Scenario::kNormal:
        if (ms == 10) request_enable(now, true, rc::milliseconds(2000));
        break;
      case Scenario::kManagerStall:
        if (ms == 10) request_enable(now, true, rc::milliseconds(3000));
        if (ms == 1000) {
          trace_.emit(now, "fault_injected", {f("name", std::string_view{"manager_stall"})});
          manager_stalled_ = true;
        }
        break;
      case Scenario::kGenerationSwitch:
        if (ms == 10) request_enable(now, true, rc::milliseconds(2000));
        if (ms == 1000) switch_generation(now);
        break;
      case Scenario::kFaultResetReenable:
        if (ms == 10) request_enable(now, true, rc::milliseconds(3000));
        if (ms == 500) inject_fault(now);
        if (ms == 700) {
          request_reset(now);
          if (defect_ == Defect::kAutoResume) request_enable(now, false, rc::milliseconds(500));
        }
        if (ms == 800 && defect_ != Defect::kAutoResume) {
          request_enable(now, true, rc::milliseconds(500));
        }
        if (ms == 805 && last_before_fault_) {
          drive_.redeliver(now, *last_before_fault_, defect_ == Defect::kReplayBypass, trace_);
        }
        break;
    }
  }

  void manager_step(rc::MonoTime now) {
    switch (policy_.state()) {
      case rc::RobotState::kArming:
        if (controller_.adopted() == arming_generation_ && !controller_.has_pending()) {
          transition(rc::PolicyEvent::kArmingConfirmed, now);
          issue(arming_generation_, now);
        }
        break;
      case rc::RobotState::kEnabled: {
        if (!permit_ || rc::is_expired(now, permit_->expires_at)) {
          trace_.emit(now, "permit_lost", {});
          transition(rc::PolicyEvent::kFault, now);
          revoke(now);
          break;
        }
        if (controller_.done()) {
          transition(rc::PolicyEvent::kTrajectoryDone, now);
          revoke(now);
          break;
        }
        if (++ticks_since_renew_ < kRenewEveryTicks) break;
        ticks_since_renew_ = 0;
        rc::RenewalInputs in;
        in.state = policy_.state();
        in.controller_adopted = controller_.adopted();
        in.controller_progress_seq = controller_.progress();
        in.feedback_sampled_at = drive_.sampled_at();
        const rc::RenewalResult r = issuer_.renew(*permit_, in, now);
        if (r.permit) {
          permit_ = r.permit;
          trace_.emit(now, "permit_issued",
                      {f("manager", permit_->manager.value), f("session", permit_->session.value),
                       f("generation", permit_->generation.value),
                       f("expires_ns", permit_->expires_at.ns)});
        } else {
          trace_.emit(now, "permit_renew_rejected", {f("reason", rc::to_string(r.refusal))});
        }
        break;
      }
      case rc::RobotState::kStopping:
        if (drive_.stopped()) transition(rc::PolicyEvent::kStopConfirmed, now);
        break;
      default:
        break;
    }
  }

  void tick(rc::MonoTime now, std::int64_t ms) {
    scenario_events(now, ms);
    if (!manager_stalled_) manager_step(now);
    const auto sp = controller_.step(now, drive_.position());
    std::optional<DriveCommand> cmd;
    if (sp) {
      std::optional<rc::MotionPermit> view = permit_;
      if (defect_ == Defect::kSkipGenerationCheck && view) view->generation = sp->generation;
      rc::GuardInput<kJoints> in;
      in.hardware_active = true;
      in.permit = view ? &*view : nullptr;
      in.current_manager = kManager;
      in.current_session = session_;
      in.setpoint = *sp;
      in.feedback_sampled_at = drive_.sampled_at();
      in.feedback_position = drive_.position();
      const rc::GuardVerdict v = guard_.check(in, now);
      if (v == rc::GuardVerdict::kSend) {
        // kSend implies the guard saw a non-null permit.
        const rc::MonoTime valid_until = rc::clamp_expiry(now + kCommandTtl, in.permit->expires_at);
        trace_.emit(now, "command_sent",
                    {f("seq", sp->source_seq), f("session", session_.value),
                     f("generation", sp->generation.value), f("valid_until_ns", valid_until.ns)});
        cmd = DriveCommand{sp->source_seq, session_, sp->generation, valid_until, sp->position};
        if (ms < 500) last_before_fault_ = cmd;
      } else {
        trace_.emit(now, "guard_block", {f("seq", sp->source_seq), f("verdict", rc::to_string(v))});
      }
    }
    drive_.step(now, cmd, trace_);
  }

  Scenario scenario_;
  Defect defect_;
  rc::TraceSink& trace_;
  rc::Duration lease_;
  rc::PolicyStateMachine policy_;
  rc::PermitIssuer issuer_;
  rc::IdAllocator ids_;
  std::optional<rc::MotionPermit> permit_;
  rc::SessionId session_{};
  rc::Generation arming_generation_{};
  rc::Generation last_generation_{};
  std::optional<DriveCommand> last_before_fault_;
  bool manager_stalled_{false};
  int ticks_since_renew_{0};
  Controller controller_;
  Drive drive_;
  rc::HalGuard<kJoints> guard_;
};

int usage() {
  std::cerr << "usage: logic_sim --scenario NAME --defect NAME --out FILE\n";
  return 2;
}

}  // namespace

int run_main(int argc, char** argv) {
  if ((argc - 1) % 2 != 0) return usage();
  std::optional<Scenario> scenario;
  std::optional<Defect> defect = Defect::kNone;
  std::string out_path;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string_view key{argv[i]};
    const std::string_view value{argv[i + 1]};
    if (key == "--scenario") {
      scenario = parse_scenario(value);
    } else if (key == "--defect") {
      defect = parse_defect(value);
    } else if (key == "--out") {
      out_path = std::string{value};
    } else {
      return usage();
    }
  }
  if (!scenario || !defect || out_path.empty()) return usage();

  const auto raw_lease = rc::max_lease(kVelMax, kAccStop, kAllowedErr, kGuardPeriodW);
  if (!raw_lease) {
    std::cerr << "lease inputs leave no lease\n";
    return 2;
  }
  const rc::Duration lease = rc::floor_to_period(*raw_lease, kPeriod);

  std::ofstream out{out_path};
  if (!out) {
    std::cerr << "cannot open " << out_path << '\n';
    return 2;
  }
  rc::JsonlTraceSink sink{out, "fake_host_mono"};
  Simulation sim{*scenario, *defect, sink, lease};
  sim.run();
  out.flush();
  return out ? 0 : 1;
}

int main(int argc, char** argv) {
  try {
    return run_main(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "logic_sim: " << e.what() << '\n';
    return 1;
  }
}
