// SPDX-License-Identifier: Apache-2.0
// Our CANopen master as a process for the interop harness: the canopen backend's I/O worker
// (C10) on a SocketCAN interface, driven by a scenario or by console lines on stdin. Reports
// JSON lines on stdout (and, with --trace, the same lines to a file), one object per line, each
// with "t" (CLOCK_MONOTONIC seconds when written) and "event"; master events also carry "at_ns",
// the worker's instant of the event:
//   start {ifname, nodes, scenario}             the worker is running
//   state {node, state}                         a heartbeat reported a new NMT state (byte)
//   boot / boot_timeout / hb_lost / hb_resumed {node}   heartbeat consumer events (C7)
//   sdo_ok {node, index, sub, size, value}      an expedited SDO request completed (C8)
//   sdo_abort {node, index, sub, code}          ... the server aborted (its code)
//   sdo_timeout / sdo_protocol {node, index, sub, code}  ... timed out / answered wrongly (the
//                                               code the master sent, 0 if none)
//   emcy {node, code, register}                 an EMCY was received
//   sync {sync_index}                           the master produced a SYNC
//   pdo {node, cob, sync_index, fields:[{index, sub, raw}]}  a received PDO decoded through its
//                                               registered layout (C5)
//   pdo_length_error {node, cob, dlc, expected} a registered PDO with the wrong DLC
//   dispatch {seq, approved, source_seq}        the session sent a new wire command_seq (C6):
//                                               an approved command or an arm hold
//   stop {stop:[...]}                           the per-node stop status changed (C18)
//   feedback {fresh, skewed, age_ns}            the published feedback's age (now - the oldest
//                                               node sample) crossed 30 ms, or its skew flag
//                                               changed
//   faults {bits, faults:[...]}                 the Feedback fault set changed
//   verdict {seq, verdict, age_ns}              stream: the HalVerdict of a write differs from the
//                                               one before (the first write included)
//   cmd {n, line}                               console: line n read; scenarios: step n begins
//   cmd_done {n, ok[, error, missing]}          ... carried out (console: an SDO result is in its
//                                               own sdo_* line before) or not
//   stream_end {writes, sent, refused, superseded, expired_before_dispatch}  stream: the last
//                                               write was made; refusals by verdict
//   scenario_done {scenario, ...} / scenario_failed {scenario, reason}   exit 0 / exit 1
//   exit {wakes, ..., tx_eagain, tx_enobufs, ...}   last line: the worker's counters
// canopen_master_cli --if IF --nodes LIST --scenario NAME [--required LIST] [--duration-ms MS]
//                    [--arm-at-ns NS] [--target LIST] [--trace PATH]
// Scenarios:
//   console       a bare master over the nodes; the commands of parse_console (args.hpp), one
//                 per line; `quit` ends the run once every command before it is done (PR #25
//                 rule), so do SIGTERM/SIGINT and the duration end, and, without
//                 --duration-ms, the end of stdin (like `quit`); a line that does not parse
//                 fails the run, and so does an end with a command pending or not carried out
//   activate_hal  the Hal over the backend on six nodes: configure, then activate through the
//                 capability gate with --required (default none)
//   arm           activate_hal, then one arm request (hold 1 s); no permit, no command
//   stream        arm, then every T_bus an approved command at the held position under a
//                 synthetic permit (expires 100 ms after each write) for --duration-ms (default
//                 2000), then nothing for 0.5 s while the drives' own stop is observed; with
//                 --target (six rad, comma separated) the commands move from the held position to
//                 the target on one trapezoid (0.5 rad/s, 2 rad/s^2 on the longest joint, the
//                 others scaled) and then stay there; the stream then ends with a deactivate
//                 step (the group stops under control and is left Switched On, no policy stop
//                 latched) instead of the 0.5 s of observed expiry
// --arm-at-ns (CLOCK_MONOTONIC ns) holds the arm step until that instant, so a harness whose
// fault window is fixed to its proxy's start knows the fault comes after the arm.
// --duration-ms is the deadline of activate_hal and arm (default 15000) and of console (default
// none).
// The bring-up of stream (configure, activate, arm) has its own 15 s deadline; its deactivate has
// only the op's own deadline.
// SIGTERM/SIGINT end any scenario.
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "args.hpp"
#include "canopen_backend/canopen_backend.hpp"
#include "canopen_backend/socket_transport.hpp"
#include "canopen_backend/worker.hpp"
#include "canopen_master/master.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/time.hpp"
#include "socketcan_io/socket.hpp"

namespace {

namespace cb = canopen_backend;
namespace cli = canopen_master_cli;
namespace cm = canopen_master;
namespace hc = hal_contract;
namespace rc = runtime_core;

std::atomic<int> g_signal{0};

extern "C" void on_signal(int sig) { g_signal.store(sig); }

constexpr rc::Duration kLoopPeriod{2'000'000};  // W: events drained, stdin read, steps polled.
constexpr rc::Duration kBusPeriod{10'000'000};  // T_bus
constexpr rc::Duration kPermitLife{100'000'000};
constexpr rc::Duration kHold{1'000'000'000};        // The arm's hold (C12: at most 1 s).
constexpr rc::Duration kObserveAfter{500'000'000};  // stream: after the last write.
constexpr rc::Duration kFeedbackAge{30'000'000};    // The CLI's max_feedback_age.
constexpr rc::Duration kShutdownTimeout{1'000'000'000};
constexpr hc::AuthorityId kAuthority{rc::ManagerInstance{1}, rc::SessionId{1}, rc::Generation{1}};

rc::MonoTime now() { return rc::SystemMonotonicClock{}.now(); }

// The distance along a trapezoid of length `d` (rad) after `t` seconds: kRampSpeed, kRampAccel.
constexpr double kRampSpeed = 0.5;  // rad/s, half of v_max
constexpr double kRampAccel = 2.0;  // rad/s^2, a fifth of a_stop
double ramp(double d, double t) {
  double ta = kRampSpeed / kRampAccel;
  double cruise = (d - kRampSpeed * ta) / kRampSpeed;
  if (cruise < 0.0) {  // Too short to reach the speed: a triangle.
    ta = std::sqrt(d / kRampAccel);
    cruise = 0.0;
  }
  const double peak = kRampAccel * ta;
  const double end = 2.0 * ta + cruise;
  if (t <= 0.0) return 0.0;
  if (t >= end) return d;
  if (t < ta) return 0.5 * kRampAccel * t * t;
  if (t < ta + cruise) return 0.5 * peak * ta + peak * (t - ta);
  const double r = end - t;
  return d - 0.5 * kRampAccel * r * r;
}

double seconds(rc::MonoTime t) { return static_cast<double>(t.ns) * 1e-9; }

class Out {
 public:
  explicit Out(std::ofstream* trace) : trace_(trace) {}
  void emit(const char* event, const std::string& fields = "") {
    std::array<char, 64> head{};
    std::snprintf(head.data(), head.size(), "{\"t\":%.6f,\"event\":\"", seconds(now()));
    std::string line = head.data();
    line += event;
    line += '"';
    if (!fields.empty()) {
      line += ',';
      line += fields;
    }
    line += "}\n";
    std::fwrite(line.data(), 1, line.size(), stdout);
    std::fflush(stdout);
    if (trace_ != nullptr) {
      *trace_ << line;
      trace_->flush();
    }
  }

 private:
  std::ofstream* trace_;
};

std::string num(const char* key, std::uint64_t value) {
  return std::string("\"") + key + "\":" + std::to_string(value);
}
std::string at_ns(rc::MonoTime t) { return "\"at_ns\":" + std::to_string(t.ns); }

const char* master_event_name(cm::MasterEventKind k) {
  switch (k) {
    case cm::MasterEventKind::kState:
      return "state";
    case cm::MasterEventKind::kBoot:
      return "boot";
    case cm::MasterEventKind::kBootTimeout:
      return "boot_timeout";
    case cm::MasterEventKind::kHeartbeatLost:
      return "hb_lost";
    case cm::MasterEventKind::kHeartbeatResumed:
      return "hb_resumed";
    case cm::MasterEventKind::kSdoOk:
      return "sdo_ok";
    case cm::MasterEventKind::kSdoAbort:
      return "sdo_abort";
    case cm::MasterEventKind::kSdoTimeout:
      return "sdo_timeout";
    case cm::MasterEventKind::kSdoProtocol:
      return "sdo_protocol";
    case cm::MasterEventKind::kEmcy:
      return "emcy";
    case cm::MasterEventKind::kSync:
      return "sync";
    case cm::MasterEventKind::kPdo:
      return "pdo";
    case cm::MasterEventKind::kPdoLengthError:
      return "pdo_length_error";
  }
  return "unknown";
}

// One worker event as a JSON line. kCmdDone is the caller's (it knows the console state).
void emit_event(Out& out, const cb::WorkerEvent& e) {
  using K = cm::MasterEventKind;
  if (e.kind == cb::WorkerEvent::Kind::kDispatch) {
    out.emit("dispatch", at_ns(e.at) + "," + num("seq", e.wire_seq) +
                             ",\"approved\":" + (e.approved ? "true" : "false") + "," +
                             num("source_seq", e.source_seq));
    return;
  }
  std::string f = at_ns(e.at);
  if (e.master_kind != K::kSync) f += "," + num("node", e.node);
  switch (e.master_kind) {
    case K::kState:
      f += "," + num("state", static_cast<unsigned>(e.state));
      break;
    case K::kSdoOk:
      f += "," + num("index", e.sdo.index) + "," + num("sub", e.sdo.sub) + "," +
           num("size", e.sdo.size) + "," + num("value", e.sdo.value);
      break;
    case K::kSdoAbort:
    case K::kSdoTimeout:
    case K::kSdoProtocol:
      f += "," + num("index", e.sdo.index) + "," + num("sub", e.sdo.sub) + "," +
           num("code", e.sdo.abort_code);
      break;
    case K::kEmcy:
      f += "," + num("code", e.emcy.error_code) + "," + num("register", e.emcy.error_register);
      break;
    case K::kSync:
      f += "," + num("sync_index", e.sync_index);
      break;
    case K::kPdo: {
      f += "," + num("cob", e.cob) + "," + num("sync_index", e.sync_index) + ",\"fields\":[";
      for (std::size_t i = 0; i < e.field_count; ++i) {
        const cm::PdoValue& v = e.fields.at(i);
        f += std::string(i == 0 ? "" : ",") + "{" + num("index", v.index) + "," +
             num("sub", v.sub) + "," + num("raw", v.raw) + "}";
      }
      f += "]";
      if (e.fields_truncated) f += ",\"truncated\":true";
      break;
    }
    case K::kPdoLengthError:
      f += "," + num("cob", e.cob) + "," + num("dlc", e.dlc) + "," + num("expected", e.expected);
      break;
    default:
      break;
  }
  out.emit(master_event_name(e.master_kind), f);
}

std::string missing_list(const std::vector<rc::Missing>& missing) {
  std::string s = "[";
  for (std::size_t i = 0; i < missing.size(); ++i) {
    s += std::string(i == 0 ? "" : ",") + "{\"capability\":\"" +
         rc::to_string(missing[i].capability) + "\",\"support\":\"" +
         rc::to_string(missing[i].support) + "\"}";
  }
  return s + "]";
}

// Sleeps to the next multiple of the loop period after `t` (absolute; a signal ends it early).
void sleep_until(rc::MonoTime t) {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t.ns / 1'000'000'000);
  ts.tv_nsec = static_cast<long>(t.ns % 1'000'000'000);
  ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

struct Run {
  cli::Scenario scenario{};
  bool finished{false};
  bool ok{false};
  Out* out{nullptr};
  void finish(bool success, const std::string& fields) {
    if (finished) return;
    finished = true;
    ok = success;
    const std::string head = std::string("\"scenario\":\"") + cli::to_string(scenario) + "\"";
    if (success) {
      out->emit("scenario_done", head + (fields.empty() ? "" : ",") + fields);
    } else {
      out->emit("scenario_failed", head + ",\"reason\":\"" + cli::json_escape(fields) + "\"");
    }
  }
};

// ---- console (master-only worker) ----

// `duration` zero: no deadline. A plain time and flag, not an optional MonoTime: GCC 13's
// Release build reports a false maybe-uninitialized on an optional passed across this call.
int run_console(cb::Worker& worker, Run& run, Out& out, rc::Duration duration) {
  const bool timed = duration.ns > 0;
  const rc::MonoTime deadline = now() + duration;
  const int flags = ::fcntl(STDIN_FILENO, F_GETFL);
  if (flags < 0 || ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::fprintf(stderr, "canopen_master_cli: console: stdin: %s\n", std::strerror(errno));
    return 1;
  }
  std::string buf;
  std::uint64_t lines = 0;
  std::uint64_t pending = 0;
  std::uint64_t not_done = 0;
  bool quit = false;
  bool eof = false;
  const auto end = [&](const std::string& how) {
    if (pending > 0) {
      run.finish(false,
                 how + " with " + std::to_string(pending) + " console command(s) still pending");
    } else if (not_done > 0) {
      run.finish(false, std::to_string(not_done) + " console command(s) were not carried out");
    } else {
      run.finish(true, "");
    }
  };
  rc::MonoTime next = now();
  while (!run.finished) {
    while (const std::optional<cb::WorkerEvent> e = worker.pop_event()) {
      if (e->kind != cb::WorkerEvent::Kind::kCmdDone) {
        emit_event(out, *e);
        continue;
      }
      out.emit("cmd_done", num("n", e->cmd_id) + ",\"ok\":" + (e->ok ? "true" : "false"));
      --pending;
      if (!e->ok) ++not_done;
      if (quit && pending == 0) end("quit");
    }
    if (run.finished) break;
    if (g_signal.load() != 0) {
      end("stopped by a signal");
      break;
    }
    if (timed && now() >= deadline) {
      end("the duration ended");
      break;
    }
    if (!eof && !quit) {
      // The read ends with EAGAIN; errno is put back so a later error report is not this one's.
      const int saved_errno = errno;
      std::array<char, 512> chunk{};
      for (;;) {
        const ssize_t n = ::read(STDIN_FILENO, chunk.data(), chunk.size());
        if (n > 0) {
          buf.append(chunk.data(), static_cast<std::size_t>(n));
          continue;
        }
        eof = n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR);
        break;
      }
      errno = saved_errno;
      for (std::size_t nl = buf.find('\n'); nl != std::string::npos && !quit && !run.finished;
           nl = buf.find('\n')) {
        const std::string line = buf.substr(0, nl);
        buf.erase(0, nl + 1);
        const std::uint64_t n = ++lines;
        out.emit("cmd", num("n", n) + ",\"line\":\"" + cli::json_escape(line) + "\"");
        cli::ConsoleLine c = cli::parse_console(line);
        if (!c.ok) {
          run.finish(false, "console line " + std::to_string(n) + ": " + c.error);
          break;
        }
        if (c.quit) {
          quit = true;  // Lines after `quit` are not read; the last cmd_done ends the run.
          if (pending == 0) end("quit");
          break;
        }
        c.command.id = n;
        if (!worker.post_raw(c.command)) {
          run.finish(false, "console line " + std::to_string(n) + ": the worker's queue is full");
          break;
        }
        ++pending;
      }
      if (eof && !quit && !run.finished && !timed) {
        // stdin closed and no duration to end the run: end it as `quit` would (a partial last
        // line without a newline is not a command).
        quit = true;
        if (pending == 0) end("stdin closed");
      }
    }
    next = next + kLoopPeriod;
    sleep_until(next);
  }
  return 0;
}

// ---- HAL scenarios (backend worker) ----

class HalRun {
 public:
  HalRun(hc::Hal& hal, cb::Worker& worker, Run& run, Out& out, rc::Duration stream_for,
         std::optional<rc::MonoTime> arm_at, std::optional<hc::Joints> target)
      : hal_(hal),
        worker_(worker),
        run_(run),
        out_(out),
        stream_for_(stream_for),
        arm_at_(arm_at),
        target_(target) {}

  void loop(std::optional<rc::MonoTime> deadline) {
    rc::MonoTime next = now();
    start_step(now());
    while (!run_.finished) {
      drain();
      const rc::MonoTime t = now();
      feedback_ = hal_.read(t);
      report_stops();
      report_feedback(t);
      if (g_signal.load() != 0) {
        run_.finish(false, "stopped by a signal before the scenario ended");
        break;
      }
      // The deadline bounds the bring-up only.
      // The deactivate after a stream keeps its own op deadline.
      const bool bring_up =
          stage_ == Stage::kConfigure || stage_ == Stage::kActivate || stage_ == Stage::kArm;
      if (deadline && t >= *deadline && bring_up) {
        run_.finish(false, "the deadline passed in step " + std::to_string(step_n_));
        break;
      }
      progress(t);
      next = next + kLoopPeriod;
      sleep_until(next);
    }
    drain();
  }

  void drain() {
    while (const std::optional<cb::WorkerEvent> e = worker_.pop_event()) {
      if (e->kind == cb::WorkerEvent::Kind::kDispatch && e->approved) ++dispatched_;
      if (e->kind != cb::WorkerEvent::Kind::kCmdDone) emit_event(out_, *e);
    }
  }

 private:
  enum class Stage : std::uint8_t { kConfigure, kActivate, kArm, kStream, kObserve, kDeactivate };

  void start_step(rc::MonoTime t) {
    const char* line = "configure";
    switch (stage_) {
      case Stage::kConfigure:
        op_ = hal_.configure(t);
        break;
      case Stage::kActivate:
        line = "activate";
        op_ = hal_.activate(t);
        break;
      case Stage::kArm:
        line = "arm";
        op_ = hal_.arm(hc::ArmRequest{kAuthority, t + kHold, 1}, t);
        break;
      case Stage::kDeactivate:
        line = "deactivate";
        op_ = hal_.deactivate(t);
        break;
      case Stage::kStream:
      case Stage::kObserve:
        return;
    }
    out_.emit("cmd", num("n", ++step_n_) + ",\"line\":\"" + line + "\"");
  }

  void progress(rc::MonoTime t) {
    if (waiting_arm_) {
      if (arm_at_ && t < *arm_at_) return;
      waiting_arm_ = false;
      start_step(t);
      return;
    }
    if (stage_ == Stage::kStream) {
      stream(t);
      return;
    }
    if (stage_ == Stage::kObserve) {
      if (t >= observe_until_) finish_stream();
      return;
    }
    if (hal_.op_state(op_) == hc::OpState::kPending) return;
    const std::optional<hc::OpResult> r = hal_.wait(op_, rc::Duration{});
    const bool ok = r && r->ok;
    std::string f = num("n", step_n_) + ",\"ok\":" + (ok ? "true" : "false");
    std::string reason;
    if (!ok) {
      reason = r ? r->error : "no result";
      f += ",\"error\":\"" + cli::json_escape(reason) + "\"";
      if (r && !r->missing.empty()) {
        f += ",\"missing\":" + missing_list(r->missing);
        reason += " (missing " + std::to_string(r->missing.size()) + " capabilities)";
      }
    }
    out_.emit("cmd_done", f);
    if (!ok) {
      run_.finish(false, "step " + std::to_string(step_n_) + ": " + reason);
      return;
    }
    const cli::Scenario s = run_.scenario;
    if (stage_ == Stage::kDeactivate) {
      finish_stream();
      return;
    }
    if (stage_ == Stage::kConfigure) {
      stage_ = Stage::kActivate;
    } else if (stage_ == Stage::kActivate) {
      if (s == cli::Scenario::kActivateHal) return run_.finish(true, "");
      stage_ = Stage::kArm;
      if (arm_at_ && t < *arm_at_) {
        waiting_arm_ = true;
        return;
      }
    } else if (stage_ == Stage::kArm) {
      if (s == cli::Scenario::kArm) return run_.finish(true, "");
      stage_ = Stage::kStream;
      held_ = feedback_.position;
      stream_end_ = t + stream_for_;
      next_write_ = t;
      return;
    }
    start_step(t);
  }

  void stream(rc::MonoTime t) {
    if (t >= stream_end_) {
      const hc::Feedback fb = hal_.read(t);
      out_.emit("stream_end",
                num("writes", writes_) + "," + num("sent", sent_) + "," +
                    num("refused", writes_ - sent_) + "," +
                    num("superseded", fb.counters.superseded) + "," +
                    num("expired_before_dispatch", fb.counters.expired_before_dispatch) +
                    ",\"verdicts\":" + verdicts());
      if (target_) {
        // With --target the drives stop under control (C18) instead of by expiry, which would
        // latch their policy stop (D11) and keep the next master from activating them.
        stage_ = Stage::kDeactivate;
        start_step(t);
        return;
      }
      stage_ = Stage::kObserve;
      observe_until_ = t + kObserveAfter;
      return;
    }
    if (t < next_write_) return;
    while (next_write_ <= t) next_write_ = next_write_ + kBusPeriod;
    const rc::MotionPermit permit{kAuthority.manager, kAuthority.session, kAuthority.generation,
                                  t + kPermitLife};
    hc::HalSetpoint sp{};
    sp.id = kAuthority;
    sp.snapshot.generation = kAuthority.generation;
    sp.snapshot.source_seq = ++writes_;
    sp.snapshot.position = held_;
    if (target_) {
      double longest = 0.0;
      for (std::size_t i = 0; i < hc::kJoints; ++i) {
        longest = std::max(longest, std::fabs(target_->at(i) - held_.at(i)));
      }
      // Time along the ramp counts writes, not the clock: the guard measures velocity and
      // acceleration per T_bus, so a write late by a loop period must not bunch the profile.
      const double elapsed =
          static_cast<double>(writes_ - 1) * 1e-9 * static_cast<double>(kBusPeriod.ns);
      const double s = longest > 0.0 ? ramp(longest, elapsed) / longest : 1.0;
      for (std::size_t i = 0; i < hc::kJoints; ++i) {
        sp.snapshot.position.at(i) =
            s >= 1.0 ? target_->at(i) : held_.at(i) + (target_->at(i) - held_.at(i)) * s;
      }
    }
    const hc::HalVerdict v = hal_.write(&permit, sp, t);
    ++verdict_counts_.at(static_cast<std::size_t>(v));
    if (!last_verdict_ || *last_verdict_ != v) {
      last_verdict_ = v;
      out_.emit("verdict", num("seq", sp.snapshot.source_seq) + ",\"verdict\":\"" +
                               hc::to_string(v) + "\"," + num("age_ns", age_ns(t)));
    }
    if (v == hc::HalVerdict::kSend) ++sent_;
  }

  std::string verdicts() const {
    std::string s = "{";
    bool first = true;
    for (std::size_t i = 0; i < verdict_counts_.size(); ++i) {
      if (verdict_counts_.at(i) == 0) continue;
      s += std::string(first ? "" : ",") + "\"" + hc::to_string(static_cast<hc::HalVerdict>(i)) +
           "\":" + std::to_string(verdict_counts_.at(i));
      first = false;
    }
    return s + "}";
  }

  void finish_stream() {
    const hc::Feedback fb = hal_.read(now());
    run_.finish(true, num("writes", writes_) + "," + num("sent", sent_) + "," +
                          num("dispatched", dispatched_) + "," +
                          num("superseded", fb.counters.superseded) + "," +
                          num("expired_before_dispatch", fb.counters.expired_before_dispatch));
  }

  std::uint64_t age_ns(rc::MonoTime t) const {
    return feedback_.sampled_at.ns == 0 || t.ns < feedback_.sampled_at.ns
               ? 0
               : static_cast<std::uint64_t>(t.ns - feedback_.sampled_at.ns);
  }

  // The published feedback's age against the 30 ms the CLI configured, on every change.
  void report_feedback(rc::MonoTime t) {
    if (feedback_.sampled_at.ns != 0) {
      const bool fresh = age_ns(t) <= static_cast<std::uint64_t>(kFeedbackAge.ns);
      const int state = (fresh ? 1 : 0) | (feedback_.skewed ? 2 : 0);
      if (!last_fresh_ || *last_fresh_ != state) {
        last_fresh_ = state;
        out_.emit("feedback", std::string("\"fresh\":") + (fresh ? "true" : "false") +
                                  ",\"skewed\":" + (feedback_.skewed ? "true" : "false") + "," +
                                  num("age_ns", age_ns(t)));
      }
    }
    if (feedback_.faults.bits() != last_faults_) {
      last_faults_ = feedback_.faults.bits();
      std::string names;
      for (const hc::Fault f :
           {hc::Fault::kTransportLost, hc::Fault::kNodeLost, hc::Fault::kBootTimeout,
            hc::Fault::kDriveFault, hc::Fault::kEventOverflow, hc::Fault::kStopUnconfirmed,
            hc::Fault::kPolicyStop, hc::Fault::kInhibited, hc::Fault::kArmFailed}) {
        if (feedback_.faults.has(f)) {
          names += std::string(names.empty() ? "" : ",") + "\"" + hc::to_string(f) + "\"";
        }
      }
      out_.emit("faults", num("bits", last_faults_) + ",\"faults\":[" + names + "]");
    }
  }

  void report_stops() {
    if (feedback_.stop == last_stop_) return;
    last_stop_ = feedback_.stop;
    std::string s = "\"stop\":[";
    for (std::size_t i = 0; i < last_stop_.size(); ++i) {
      s += std::string(i == 0 ? "" : ",") + "\"" + hc::to_string(last_stop_.at(i)) + "\"";
    }
    out_.emit("stop", s + "]");
  }

  hc::Hal& hal_;
  cb::Worker& worker_;
  Run& run_;
  Out& out_;
  rc::Duration stream_for_;
  std::optional<rc::MonoTime> arm_at_;
  std::optional<hc::Joints> target_;
  bool waiting_arm_{false};
  std::optional<hc::HalVerdict> last_verdict_;
  std::optional<int> last_fresh_;
  std::uint32_t last_faults_{0};
  Stage stage_{Stage::kConfigure};
  hc::OpHandle op_{};
  std::uint64_t step_n_{0};
  hc::Feedback feedback_{};
  std::array<hc::StopStatus, hc::kJoints> last_stop_{};
  hc::Joints held_{};
  rc::MonoTime stream_end_{};
  rc::MonoTime next_write_{};
  rc::MonoTime observe_until_{};
  std::uint64_t writes_{0};
  std::uint64_t sent_{0};
  std::uint64_t dispatched_{0};
  std::array<std::uint64_t, 32> verdict_counts_{};
};

rc::JointLimits<hc::kJoints> limits() {
  rc::JointLimits<hc::kJoints> l{};
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    l.pos_min.at(i) = -6.0;
    l.pos_max.at(i) = 6.0;
    l.vel_max.at(i) = 1.0;   // v_max
    l.acc_max.at(i) = 10.0;  // a_stop
  }
  return l;
}

bool number_option(const cli::Options& opts, const char* key, std::uint64_t low, std::uint64_t high,
                   std::uint64_t& out) {
  const auto it = opts.values.find(key);
  if (it == opts.values.end()) return true;
  const cli::Number parsed = cli::parse_number(it->second, low, high);
  if (!parsed.ok) {
    std::fprintf(stderr, "canopen_master_cli: --%s: %s\n", key, parsed.error.c_str());
    return false;
  }
  out = parsed.value;
  return true;
}

int run(const cli::Options& opts) {
  const cli::ScenarioOption scenario = cli::parse_scenario(opts.values.at("scenario"));
  const cli::NodeList nodes = cli::parse_nodes(opts.values.at("nodes"));
  cli::RequiredOption required = cli::parse_required("none");
  if (const auto it = opts.values.find("required"); it != opts.values.end()) {
    required = cli::parse_required(it->second);
  }
  std::uint64_t duration_ms = 0;
  std::uint64_t arm_at_ns = 0;
  std::optional<hc::Joints> target;
  if (const auto it = opts.values.find("target"); it != opts.values.end()) {
    const cli::TargetOption parsed = cli::parse_target(it->second);
    if (!parsed.ok || scenario.scenario != cli::Scenario::kStream) {
      std::fprintf(stderr, "canopen_master_cli: %s\n",
                   parsed.ok ? "--target needs --scenario stream" : parsed.error.c_str());
      return 2;
    }
    target = parsed.position;
  }
  if (!scenario.ok || !nodes.ok || !required.ok ||
      !number_option(opts, "duration-ms", 1, 3'600'000, duration_ms) ||
      !number_option(opts, "arm-at-ns", 1, 0x7FFFFFFFFFFFFFFFULL, arm_at_ns)) {
    std::fprintf(stderr, "canopen_master_cli: %s\n",
                 (!scenario.ok ? scenario.error
                  : !nodes.ok  ? nodes.error
                               : required.error)
                     .c_str());
    return 2;
  }
  const bool console = scenario.scenario == cli::Scenario::kConsole;
  if (!console && nodes.nodes.size() != cm::kSessionNodes) {
    std::fprintf(stderr, "canopen_master_cli: scenario %s needs exactly six nodes\n",
                 cli::to_string(scenario.scenario));
    return 2;
  }
  std::unique_ptr<std::ofstream> trace;
  if (const auto it = opts.values.find("trace"); it != opts.values.end()) {
    trace = std::make_unique<std::ofstream>(it->second, std::ios::trunc);
    if (!*trace) {
      std::fprintf(stderr, "canopen_master_cli: --trace: cannot open %s\n", it->second.c_str());
      return 2;
    }
  }
  const std::string& ifname = opts.values.at("if");
  std::string error;
  std::optional<socketcan_io::CanSocket> socket = socketcan_io::CanSocket::open(ifname, &error);
  if (!socket) {
    std::fprintf(stderr, "canopen_master_cli: %s: %s\n", ifname.c_str(), error.c_str());
    return 1;
  }
  auto transport = std::make_unique<cb::SocketTransport>(std::move(*socket));

  std::vector<canopen_codec::NodeId> ids;
  ids.reserve(nodes.nodes.size());
  for (const std::uint8_t n : nodes.nodes) {
    const std::optional<canopen_codec::NodeId> id = canopen_codec::NodeId::make(n);
    if (!id) return 2;  // parse_nodes gives 1..127 only.
    ids.push_back(*id);
  }
  std::unique_ptr<cb::Worker> owned;
  cb::WorkerOptions events;
  events.forward_events = true;  // This process reads and reports every event.
  if (console) {
    cm::MasterConfig mc;
    mc.nodes = ids;
    owned = std::make_unique<cb::Worker>(mc, std::move(transport), events);
  } else {
    cb::CanopenConfig cfg;
    for (std::size_t i = 0; i < cm::kSessionNodes; ++i) cfg.session.nodes.at(i) = ids.at(i);
    cfg.session.max_feedback_age = rc::milliseconds(30);
    owned = std::make_unique<cb::Worker>(cfg, std::move(transport), events);
  }
  cb::Worker& worker = *owned;
  if (!worker.start(&error)) {
    std::fprintf(stderr, "canopen_master_cli: worker: %s\n", error.c_str());
    return 1;
  }

  struct sigaction sa {};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  ::sigaction(SIGINT, &sa, nullptr);
  ::sigaction(SIGTERM, &sa, nullptr);

  Out out(trace.get());
  Run r;
  r.scenario = scenario.scenario;
  r.out = &out;
  std::string list;
  for (const std::uint8_t n : nodes.nodes) list += (list.empty() ? "" : ",") + std::to_string(n);
  out.emit("start", "\"ifname\":\"" + cli::json_escape(ifname) + "\",\"nodes\":[" + list +
                        "],\"scenario\":\"" + cli::to_string(scenario.scenario) + "\"");

  int status = 0;
  std::unique_ptr<hc::Hal> hal;
  if (console) {
    status = run_console(worker, r, out, rc::milliseconds(static_cast<std::int64_t>(duration_ms)));
  } else {
    const bool stream = scenario.scenario == cli::Scenario::kStream;
    const std::uint64_t ms = duration_ms > 0 ? duration_ms : (stream ? 2000 : 15000);
    hal = std::make_unique<hc::Hal>(std::move(owned), required.required,
                                    rc::GuardConfig{kBusPeriod, rc::milliseconds(30)}, limits());
    HalRun hr(*hal, worker, r, out, rc::milliseconds(static_cast<std::int64_t>(ms)),
              arm_at_ns > 0
                  ? std::optional<rc::MonoTime>(rc::MonoTime{static_cast<std::int64_t>(arm_at_ns)})
                  : std::nullopt,
              target);
    // stream: the duration is the stream's length; the bring-up has its own 15 s deadline.
    const rc::Duration bring_up =
        stream ? rc::milliseconds(15000) : rc::milliseconds(static_cast<std::int64_t>(ms));
    hr.loop(now() + bring_up);
  }
  // The worker stops the group if needed, releases the socket and exits; then its counters.
  if (!worker.shutdown(kShutdownTimeout)) {
    std::fprintf(stderr, "canopen_master_cli: the worker did not exit within 1 s\n");
    status = 1;
  }
  while (const std::optional<cb::WorkerEvent> e = worker.pop_event()) {
    if (e->kind != cb::WorkerEvent::Kind::kCmdDone) emit_event(out, *e);
  }
  out.emit("exit", cb::stats_json(worker.stats()));
  if (status == 0 && !r.ok) status = 1;
  return status;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const cli::Options opts = cli::parse_options(
      args, {"if", "nodes", "scenario", "required", "duration-ms", "arm-at-ns", "target", "trace"},
      {"if", "nodes", "scenario"});
  if (!opts.ok) {
    std::fprintf(stderr,
                 "canopen_master_cli: %s\nusage: canopen_master_cli --if IF --nodes LIST "
                 "--scenario console|activate_hal|arm|stream [--required LIST] "
                 "[--duration-ms MS] [--arm-at-ns NS] [--target LIST] [--trace PATH]\n",
                 opts.error.c_str());
    return 2;
  }
  try {
    return run(opts);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "canopen_master_cli: %s\n", e.what());
    return 1;
  }
}
