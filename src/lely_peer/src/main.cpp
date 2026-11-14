// SPDX-License-Identifier: Apache-2.0
// Independent CANopen master for the interoperability tests: Lely AsyncMaster with one
// FiberDriver per slave assigned in the DCF (0x1F81). Reports what Lely decodes as JSON lines on
// stdout, one object per line, each with "t" (CLOCK_MONOTONIC seconds) and "event":
//   start  {node, ifname, slaves}         master reset, drivers registered
//   state  {node, state}                  Lely OnState for a slave
//   boot   {node, state, es, what}        Lely OnBoot ('boot slave' process result)
//   hb     {node, occurred}               Lely OnHeartbeat (consumer timeout occurred/resolved)
//   emcy   {node, code, register}         Lely OnEmcy
//   sync   {cnt}                          Lely OnSync (the master's own SYNC)
//   rpdo   {node, index, sub, value}      a changed value Lely decoded into its remote-mapped
//                                         objects (statusword 0x6041, applied_seq 0x2030:02)
//   nmt_sent {node, cs}                   an NMT command the scenario asked Lely to send
//   cw     {node, value}                  a controlword the scenario wrote for RPDO3
//   sdo_ok {node, op, index, sub, value}  an SDO transfer Lely's client completed
//   sdo_abort {node, op, index, sub, code}  ... ended with an abort code (Lely's or the server's)
//   sdo_timeout {node, op, index, sub}    ... timed out in Lely's client (0x05040000)
//   scenario_done {scenario, ...}         the scenario finished, with its own counters; exit 0
//   scenario_failed {scenario, reason}    a step could not be performed; exit 1
//   exit   {}                             last line
//   lely_peer --if IF --dcf PATH --scenario NAME [--node N] [--count K] [--duration-ms MS]
// Scenarios (the node scenarios start when Lely reports node N booted; --duration-ms is their
// deadline, default 15 s, after which they fail):
//   idle               send nothing of its own until the duration ends (default: SIGTERM/SIGINT)
//   emcy_watch         like idle, counting EMCY; scenario_done also on SIGTERM/SIGINT
//   nmt_sequence       Start, Stop, Enter Pre-operational, Start, Reset Communication, Reset
//                      Node to node N, each awaited in Lely's reported state or boot
//   sdo_suite          SDO uploads and downloads to node N with expected successes and aborts
//   commit_and_enable  NMT Start; controlwords 0x06, 0x07 by RPDO3; the drive extension's commit
//                      transaction through Lely's SDO client; controlword 0x0F
//   pdo_stream         commit_and_enable, then K updates of RPDO1/RPDO2 (one per SYNC, seq + 1,
//                      valid_until = now + 40 ms) that Lely sends as synchronous PDOs; then stop
// Every PDO is encoded by Lely from the remote-mapped objects of the master DCF (dcfgen -r); the
// peer writes values, never frames.
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
// The C++ form of the NMT service API: in C++ the C header needs the types this one declares.
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <functional>
#include <lely/co/nmt.hpp>
#include <lely/coapp/sdo_error.hpp>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "lely_peer/args.hpp"

namespace {

using namespace lely;

double now_s() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// D5: floor(CLOCK_MONOTONIC ns / 1000) mod 2^32, the domain the drive compares valid_until in.
std::uint32_t now_wire_us() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const auto us = (static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000U) +
                  (static_cast<std::uint64_t>(ts.tv_nsec) / 1000U);
  return static_cast<std::uint32_t>(us);
}

// Prints one JSON line: the caller passes the fields after "event" as a JSON fragment.
void emit(const char* event, const std::string& fields = "") {
  std::printf("{\"t\":%.6f,\"event\":\"%s\"%s%s}\n", now_s(), event, fields.empty() ? "" : ",",
              fields.c_str());
  std::fflush(stdout);
}

std::string node_field(std::uint8_t id) { return "\"node\":" + std::to_string(id); }

std::string num(const char* key, std::int64_t value) {
  return std::string(",\"") + key + "\":" + std::to_string(value);
}

// Objects of our EDS the scenarios use (the remote-mapped PDO entries and SDO targets).
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kRpdo1Ext = 0x2020;     // 01 command_seq, 02 exec_tag
constexpr std::uint16_t kRpdo2Ext = 0x2021;     // 01 valid_until_us, 02 command_seq, 03 exec_tag
constexpr std::uint16_t kFeedbackExt = 0x2030;  // 02 applied_seq
constexpr std::uint16_t kExecRequest = 0x2010;  // 01 gen_lo, 02 gen_hi, 03 session, 04 attempt
constexpr std::uint16_t kExecCommit = 0x2011;
constexpr std::uint16_t kExecTag = 0x2012;
constexpr std::uint16_t kCommittedAttempt = 0x2016;
constexpr std::uint32_t kValidityUs = 40'000;  // valid_until = now + 40 ms
constexpr std::chrono::milliseconds kSdoTimeout{1000};

// CiA 402 statusword states (mask 0x6F).
constexpr std::uint16_t kStateMask = 0x6F;
constexpr std::uint16_t kReadyToSwitchOn = 0x21;
constexpr std::uint16_t kSwitchedOn = 0x23;
constexpr std::uint16_t kOperationEnabled = 0x27;

// What the run shares between the drivers and main: the scenario and how it ends.
struct Run {
  std::string scenario;
  std::uint8_t node = 0;
  std::uint32_t count = 0;
  bool finished = false;
  std::uint64_t emcy = 0;
  // Prints scenario_done (ok) or scenario_failed and stops the master; only the first call counts.
  std::function<void(bool ok, const std::string& fields)> finish;
};

bool is_node_scenario(std::string_view s) {
  return s == "nmt_sequence" || s == "sdo_suite" || s == "commit_and_enable" || s == "pdo_stream";
}

class PeerDriver : public canopen::FiberDriver {
 public:
  PeerDriver(ev::Executor exec, canopen::AsyncMaster& master, std::uint8_t id, Run& run)
      : FiberDriver(exec, master, id), run_(run) {}

 private:
  void OnState(canopen::NmtState st) noexcept override {
    state_ = st;
    ++states_;
    emit("state", node_field(id()) + ",\"state\":" + std::to_string(static_cast<int>(st)));
  }

  void OnBoot(canopen::NmtState st, char es, const std::string& what) noexcept override {
    const std::string es_text = es != 0 ? std::string(1, es) : std::string();
    emit("boot", node_field(id()) + ",\"state\":" + std::to_string(static_cast<int>(st)) +
                     ",\"es\":\"" + lely_peer::json_escape(es_text) + "\",\"what\":\"" +
                     lely_peer::json_escape(what) + "\"");
    ++boots_;
    if (id() != run_.node || started_ || !is_node_scenario(run_.scenario)) {
      return;
    }
    started_ = true;
    if (es != 0) {
      run_.finish(false,
                  "boot of node " + std::to_string(id()) + " failed: es '" + es_text + "' " + what);
      return;
    }
    // A task on the driver's executor runs as a fiber, so it may suspend (USleep) while it
    // waits for SDO confirmations and decoded TPDOs.
    GetExecutor().post([this]() { Scenario(); });
  }

  void OnHeartbeat(bool occurred) noexcept override {
    emit("hb", node_field(id()) + ",\"occurred\":" + (occurred ? "true" : "false"));
  }

  void OnEmcy(std::uint16_t eec, std::uint8_t er, std::uint8_t /*msef*/[5]) noexcept override {
    ++run_.emcy;
    emit("emcy", node_field(id()) + ",\"code\":" + std::to_string(eec) +
                     ",\"register\":" + std::to_string(er));
  }

  // Values Lely decoded from the drive's TPDOs into the remote-mapped objects.
  void OnRpdoWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override {
    const bool statusword = idx == kStatusword && subidx == 0;
    const bool applied = idx == kFeedbackExt && subidx == 2;
    if (!statusword && !applied) {
      return;
    }
    std::error_code ec;
    const auto value = rpdo_mapped[idx][subidx].Read<std::uint16_t>(ec);
    if (ec) {
      return;
    }
    std::uint16_t& last = statusword ? statusword_ : applied_seq_;
    if (statusword) {
      statuswords_.push_back(value);
    }
    if (value != last || (statusword && statuswords_.size() == 1)) {
      emit("rpdo", node_field(id()) + num("index", idx) + num("sub", subidx) + num("value", value));
    }
    last = value;
  }

  // pdo_stream: one update of RPDO1/RPDO2 per SYNC; Lely sends the synchronous PDOs.
  void OnSync(std::uint8_t /*cnt*/, const time_point& /*t*/) noexcept override {
    if (!streaming_ || remaining_ == 0) {
      return;
    }
    ++seq_;
    const auto target = static_cast<std::int32_t>(seq_) * 100;  // µrad: a slow ramp
    const std::uint32_t until = now_wire_us() + kValidityUs;
    std::error_code ec;
    tpdo_mapped[kTargetPosition][0].Write(target, ec);
    if (!ec) tpdo_mapped[kRpdo1Ext][1].Write(seq_, ec);
    if (!ec) tpdo_mapped[kRpdo1Ext][2].Write(tag_, ec);
    if (!ec) tpdo_mapped[kRpdo2Ext][1].Write(until, ec);
    if (!ec) tpdo_mapped[kRpdo2Ext][2].Write(seq_, ec);
    if (!ec) tpdo_mapped[kRpdo2Ext][3].Write(tag_, ec);
    if (ec) {
      streaming_ = false;
      run_.finish(false, "writing the RPDO1/RPDO2 values: " + ec.message());
      return;
    }
    --remaining_;
    ++sent_;
    if (enable_pending_) {
      // Enable Operation right after the first update: its fragments go out at the next SYNC
      // and are actuated at the one after, inside the drive's 30 ms command timeout.
      enable_pending_ = false;
      if (!Controlword(0x0F)) {
        streaming_ = false;
      }
    }
  }

  // ---- steps, called from the scenario fiber ----

  bool WaitFor(const std::function<bool()>& done, int ms) {
    for (int i = 0; i < ms; ++i) {
      if (done()) {
        return true;
      }
      if (run_.finished) {
        return false;
      }
      std::error_code ec;
      USleep(1000, ec);
    }
    return done();
  }

  bool WaitStatus(std::uint16_t state, int ms) {
    return WaitFor(
        [this, state]() {
          for (const auto sw : statuswords_) {
            if ((sw & kStateMask) == state) {
              return true;
            }
          }
          return false;
        },
        ms);
  }

  bool Controlword(std::uint16_t value) {
    statuswords_.clear();  // WaitStatus looks at what Lely decodes after this write.
    std::error_code ec;
    tpdo_mapped[kControlword][0].Write(value, ec);
    if (!ec) {
      tpdo_mapped[kControlword][0].WriteEvent(ec);  // RPDO3 is event-driven (type 255).
    }
    if (ec) {
      run_.finish(false, "controlword " + std::to_string(value) + ": " + ec.message());
      return false;
    }
    emit("cw", node_field(id()) + num("value", value));
    return true;
  }

  bool Nmt(canopen::NmtCommand cs) {
    try {
      master.Command(cs, id());
    } catch (const std::exception& e) {
      run_.finish(false, "NMT command " + std::to_string(static_cast<int>(cs)) + ": " + e.what());
      return false;
    }
    emit("nmt_sent", node_field(id()) + num("cs", static_cast<int>(cs)));
    return true;
  }

  void SdoError(const char* op, std::uint16_t idx, std::uint8_t sub, const std::error_code& ec) {
    const std::string where =
        node_field(id()) + ",\"op\":\"" + op + "\"" + num("index", idx) + num("sub", sub);
    if (ec == canopen::SdoErrc::TIMEOUT) {
      ++sdo_timeout_;
      emit("sdo_timeout", where);
    } else {
      ++sdo_abort_;
      emit("sdo_abort", where + num("code", static_cast<std::uint32_t>(ec.value())));
    }
  }

  // An SDO transfer's confirmation from Lely's client; shared so a confirmation that arrives
  // after the scenario gave up writes into live memory.
  template <typename T>
  struct Outcome {
    bool done = false;
    std::error_code ec;
    T value{};
  };

  // Lely 2.3.5's FiberDriver::Wait(SdoFuture<T>, error_code&) crashed here (SIGSEGV in
  // __cxa_begin_catch) on the first SDO abort; its error path also has no return statement in
  // a non-void function. The transfers therefore use Lely's confirmation callbacks (no
  // exception) and the fiber polls for them.
  template <typename T>
  bool AwaitOutcome(const std::shared_ptr<Outcome<T>>& outcome) {
    const int budget_ms = static_cast<int>(kSdoTimeout.count()) + 500;
    return WaitFor([&outcome]() { return outcome->done; }, budget_ms);
  }

  template <typename T>
  std::optional<T> SdoRead(std::uint16_t idx, std::uint8_t sub) {
    auto outcome = std::make_shared<Outcome<T>>();
    std::error_code ec;
    SubmitRead<T>(
        idx, sub,
        [outcome](std::uint8_t, std::uint16_t, std::uint8_t, std::error_code result, T value) {
          outcome->ec = result;
          outcome->value = value;
          outcome->done = true;
        },
        kSdoTimeout, ec);
    if (ec || !AwaitOutcome(outcome)) {
      SdoError("read", idx, sub, ec ? ec : canopen::make_error_code(canopen::SdoErrc::TIMEOUT));
      return std::nullopt;
    }
    if (outcome->ec) {
      SdoError("read", idx, sub, outcome->ec);
      return std::nullopt;
    }
    ++sdo_ok_;
    emit("sdo_ok", node_field(id()) + ",\"op\":\"read\"" + num("index", idx) + num("sub", sub) +
                       num("value", static_cast<std::int64_t>(outcome->value)));
    return outcome->value;
  }

  template <typename T>
  bool SdoWrite(std::uint16_t idx, std::uint8_t sub, T value) {
    auto outcome = std::make_shared<Outcome<bool>>();
    std::error_code ec;
    SubmitWrite(
        idx, sub, T{value},
        [outcome](std::uint8_t, std::uint16_t, std::uint8_t, std::error_code result) {
          outcome->ec = result;
          outcome->done = true;
        },
        kSdoTimeout, ec);
    if (ec || !AwaitOutcome(outcome)) {
      SdoError("write", idx, sub, ec ? ec : canopen::make_error_code(canopen::SdoErrc::TIMEOUT));
      return false;
    }
    if (outcome->ec) {
      SdoError("write", idx, sub, outcome->ec);
      return false;
    }
    ++sdo_ok_;
    emit("sdo_ok", node_field(id()) + ",\"op\":\"write\"" + num("index", idx) + num("sub", sub) +
                       num("value", static_cast<std::int64_t>(value)));
    return true;
  }

  // ---- scenarios ----

  void Scenario() {
    if (run_.scenario == "nmt_sequence") {
      NmtSequence();
    } else if (run_.scenario == "sdo_suite") {
      SdoSuite();
    } else if (run_.scenario == "commit_and_enable") {
      std::string fields;
      if (CommitAndEnable(false, fields)) {
        run_.finish(true, fields);
      }
    } else if (run_.scenario == "pdo_stream") {
      PdoStream();
    }
  }

  void NmtSequence() {
    using canopen::NmtCommand;
    using canopen::NmtState;
    struct Step {
      NmtCommand cs;
      std::optional<NmtState> state;  // awaited in OnState; empty: await a new boot
    };
    const Step steps[] = {
        {NmtCommand::START, NmtState::START},       {NmtCommand::STOP, NmtState::STOP},
        {NmtCommand::ENTER_PREOP, NmtState::PREOP}, {NmtCommand::START, NmtState::START},
        {NmtCommand::RESET_COMM, std::nullopt},     {NmtCommand::RESET_NODE, std::nullopt}};
    int commands = 0;
    for (const Step& step : steps) {
      const std::uint64_t boots = boots_;
      if (!Nmt(step.cs)) {
        return;
      }
      ++commands;
      const bool reached = step.state
                               ? WaitFor([this, &step]() { return state_ == *step.state; }, 1000)
                               : WaitFor([this, boots]() { return boots_ > boots; }, 3000);
      if (!reached) {
        run_.finish(false, "node " + std::to_string(id()) +
                               " did not reach the state of NMT "
                               "command " +
                               std::to_string(static_cast<int>(step.cs)));
        return;
      }
    }
    run_.finish(true, num("commands", commands) + num("states", states_) +
                          num("boots", static_cast<std::int64_t>(boots_)));
  }

  void SdoSuite() {
    // Uploads the drive answers (CiA 301 and CiA 402 objects, the extension's read-only ones).
    SdoRead<std::uint32_t>(0x1000, 0);  // device type
    SdoRead<std::uint8_t>(0x1001, 0);   // error register
    SdoRead<std::uint16_t>(0x1017, 0);  // producer heartbeat time
    for (std::uint8_t sub = 1; sub <= 4; ++sub) {
      SdoRead<std::uint32_t>(0x1018, sub);  // identity
    }
    SdoRead<std::uint32_t>(0x2013, 0);  // boot_id
    SdoRead<std::uint16_t>(0x2014, 0);  // command_timeout_ms
    SdoRead<std::int8_t>(0x6061, 0);    // modes of operation display
    // A download the drive accepts: modes of operation = 8 (its only mode).
    SdoWrite<std::int8_t>(0x6060, 0, 8);
    // Transfers the drive must refuse, each with its own abort code.
    SdoRead<std::uint32_t>(0x5000, 0);            // no such object
    SdoRead<std::uint32_t>(0x1000, 1);            // no such sub-index
    SdoWrite<std::uint32_t>(0x2013, 0, 1);        // read-only
    SdoWrite<std::int8_t>(0x6060, 0, 7);          // value out of range
    SdoWrite<std::uint16_t>(kControlword, 0, 0);  // PDO-only object
    run_.finish(true, num("ok", sdo_ok_) + num("abort", sdo_abort_) + num("timeout", sdo_timeout_));
  }

  // NMT Start, 0x06, 0x07, the commit transaction (D17), then 0x0F (or, with `stream`, the
  // first RPDO1/RPDO2 update and 0x0F at the next SYNC). The drive accepts a commit only in
  // Switched On, so the transaction sits between 0x07 and 0x0F.
  bool CommitAndEnable(bool stream, std::string& fields) {
    const std::string node = std::to_string(id());
    if (!Nmt(canopen::NmtCommand::START)) {
      return false;
    }
    if (!WaitFor([this]() { return state_ == canopen::NmtState::START; }, 1000)) {
      run_.finish(false, "node " + node + " did not report Operational");
      return false;
    }
    if (!Controlword(0x06) || !WaitStatus(kReadyToSwitchOn, 500)) {
      run_.finish(false, "node " + node + " did not report Ready to Switch On after 0x06");
      return false;
    }
    if (!Controlword(0x07) || !WaitStatus(kSwitchedOn, 500)) {
      run_.finish(false, "node " + node + " did not report Switched On after 0x07");
      return false;
    }
    const auto previous = SdoRead<std::uint32_t>(kCommittedAttempt, 0);
    if (!previous) {
      run_.finish(false, "reading 0x2016 committed_attempt failed");
      return false;
    }
    const std::uint32_t attempt = *previous + 1;
    const std::uint32_t generation_lo = attempt;
    const std::uint32_t generation_hi = 0;
    const std::uint32_t session_lo = 1;
    const bool staged =
        SdoWrite(kExecRequest, 4, attempt) && SdoWrite(kExecRequest, 1, generation_lo) &&
        SdoWrite(kExecRequest, 2, generation_hi) && SdoWrite(kExecRequest, 3, session_lo) &&
        SdoWrite(kExecCommit, 0,
                 lely_peer::commit_crc(attempt, generation_lo, generation_hi, session_lo));
    if (!staged) {
      run_.finish(false, "the commit transaction was refused (see the sdo_abort line)");
      return false;
    }
    const auto committed = SdoRead<std::uint32_t>(kCommittedAttempt, 0);
    const auto tag = SdoRead<std::uint16_t>(kExecTag, 0);
    if (!committed || !tag || *committed != attempt || *tag == 0) {
      run_.finish(false, "0x2016/0x2012 do not show the committed attempt and a tag");
      return false;
    }
    if (stream) {
      tag_ = *tag;
      seq_ = 0;
      remaining_ = run_.count;
      enable_pending_ = true;
      streaming_ = true;
    } else if (!Controlword(0x0F)) {
      return false;
    }
    if (!WaitStatus(kOperationEnabled, 500)) {
      run_.finish(false, "node " + node + " did not report Operation Enabled after 0x0F");
      return false;
    }
    fields = node_field(id()) + num("attempt_id", attempt) + num("tag", *tag) +
             num("statusword", statusword_);
    return true;
  }

  void PdoStream() {
    std::string fields;
    if (!CommitAndEnable(true, fields)) {
      return;
    }
    const int budget_ms = static_cast<int>(run_.count) * 20 + 500;
    if (!WaitFor([this]() { return remaining_ == 0; }, budget_ms)) {
      run_.finish(false, "only " + std::to_string(sent_) + " of " + std::to_string(run_.count) +
                             " updates written");
      return;
    }
    // The last update goes out at the next SYNC and is applied at the one after; the drive's
    // TPDO2 reports it one SYNC later. Not required: the bank's trace is the judge.
    WaitFor([this]() { return applied_seq_ == seq_; }, 100);
    streaming_ = false;
    run_.finish(true, node_field(id()) + num("tag", tag_) + num("sent", sent_) +
                          num("applied_seq", applied_seq_));
  }

  Run& run_;
  bool started_ = false;
  canopen::NmtState state_ = canopen::NmtState::BOOTUP;
  std::int64_t states_ = 0;
  std::uint64_t boots_ = 0;
  std::uint16_t statusword_ = 0;
  std::vector<std::uint16_t> statuswords_;
  std::uint16_t applied_seq_ = 0;
  std::int64_t sdo_ok_ = 0;
  std::int64_t sdo_abort_ = 0;
  std::int64_t sdo_timeout_ = 0;
  bool streaming_ = false;
  bool enable_pending_ = false;
  std::uint16_t tag_ = 0;
  std::uint16_t seq_ = 0;
  std::uint32_t remaining_ = 0;
  std::uint32_t sent_ = 0;
};

// B15 override. Lely 2.3.5's default heartbeat indication (co_nmt_on_hb) answers a consumer
// timeout of a slave with the CiA 302 error handling (co_nmt_node_err_ind), which restarts the
// 'boot slave' process and puts NMT Reset Node (0x81) for that slave on the wire 0.25 s after
// the last heartbeat. No dcfgen option suppresses it (observed with start_nodes, mandatory and
// reset_communication each set; see tools/interop/test_peer_config.py). The peer therefore
// installs its own indication: a timeout that occurs is reported to the driver (`hb` event) and
// nothing else happens; every other heartbeat event goes to Lely's own handler unchanged. A
// slave that comes back is booted again by Lely on its boot-up message (co_nmt_on_st).
class PeerMaster : public canopen::AsyncMaster {
 public:
  PeerMaster(io::TimerBase& timer, io::CanChannelBase& chan, const std::string& dcf_txt)
      : AsyncMaster(timer, chan, dcf_txt, "") {
    // In C++ the NMT service type is lely::CONMT (co/nmt.hpp) while
    // Node::nmt() returns the C struct; both are the same object (Lely's own coapp casts too).
    auto* service = reinterpret_cast<co_nmt_t*>(nmt());
    co_nmt_get_hb_ind(service, &lely_hb_ind_, &lely_hb_data_);
    co_nmt_set_hb_ind(service, &PeerMaster::hb_ind, this);
  }

 private:
  static void hb_ind(co_nmt_t* nmt, co_unsigned8_t id, int state, int reason, void* data) noexcept {
    auto* self = static_cast<PeerMaster*>(data);
    if (state == CO_NMT_EC_OCCURRED && reason == CO_NMT_EC_TIMEOUT) {
      self->OnHeartbeat(id, true);
      return;
    }
    if (self->lely_hb_ind_ != nullptr) {
      self->lely_hb_ind_(nmt, id, state, reason, self->lely_hb_data_);
    }
  }

  co_nmt_hb_ind_t* lely_hb_ind_ = nullptr;
  void* lely_hb_data_ = nullptr;

  void OnSync(std::uint8_t cnt, const time_point& t) noexcept override {
    emit("sync", "\"cnt\":" + std::to_string(cnt));
    AsyncMaster::OnSync(cnt, t);
  }
};

// Slaves assigned in the master's DCF: 0x1F81 sub-index n, bit 0 set.
std::vector<std::uint8_t> assigned_slaves(canopen::AsyncMaster& master) {
  std::vector<std::uint8_t> ids;
  for (std::uint8_t id = 1; id <= 127; ++id) {
    std::error_code ec;
    const auto assignment = master.Read<std::uint32_t>(0x1F81, id, ec);
    if (!ec && (assignment & 0x01U) != 0 && id != master.id()) {
      ids.push_back(id);
    }
  }
  return ids;
}

// Reads an optional numeric option into `out` (left as is when absent); false after a message.
bool number_option(const lely_peer::Options& opts, const char* key, std::uint64_t low,
                   std::uint64_t high, std::uint64_t& out) {
  const auto it = opts.values.find(key);
  if (it == opts.values.end()) {
    return true;
  }
  const auto parsed = lely_peer::parse_number(it->second, low, high);
  if (!parsed.ok) {
    std::fprintf(stderr, "lely_peer: --%s: %s\n", key, parsed.error.c_str());
    return false;
  }
  out = parsed.value;
  return true;
}

int run(const lely_peer::Options& opts) {
  Run run;
  run.scenario = opts.values.at("scenario");
  const bool node_scenario = is_node_scenario(run.scenario);
  if (!node_scenario && run.scenario != "idle" && run.scenario != "emcy_watch") {
    std::fprintf(stderr,
                 "lely_peer: unknown scenario '%s' (known: idle, emcy_watch, nmt_sequence, "
                 "sdo_suite, commit_and_enable, pdo_stream)\n",
                 run.scenario.c_str());
    return 2;
  }
  std::uint64_t duration_ms = 0;
  std::uint64_t node = 0;
  std::uint64_t count = 0;
  if (!number_option(opts, "duration-ms", 1, 3600000, duration_ms) ||
      !number_option(opts, "node", 1, 127, node) ||
      !number_option(opts, "count", 1, 65535, count)) {
    return 2;
  }
  if (node_scenario && node == 0) {
    std::fprintf(stderr, "lely_peer: scenario %s needs --node\n", run.scenario.c_str());
    return 2;
  }
  if (run.scenario == "pdo_stream" && count == 0) {
    std::fprintf(stderr, "lely_peer: scenario pdo_stream needs --count\n");
    return 2;
  }
  run.node = static_cast<std::uint8_t>(node);
  run.count = static_cast<std::uint32_t>(count);
  if (node_scenario && duration_ms == 0) {
    duration_ms = 15000;  // the node scenarios' deadline
  }
  const std::string& ifname = opts.values.at("if");

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(ifname.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);
  PeerMaster master(timer, chan, opts.values.at("dcf"));

  std::vector<std::unique_ptr<PeerDriver>> drivers;
  std::string slaves;
  bool node_assigned = false;
  for (const auto id : assigned_slaves(master)) {
    drivers.push_back(std::make_unique<PeerDriver>(exec, master, id, run));
    slaves += (slaves.empty() ? "" : ",") + std::to_string(id);
    node_assigned = node_assigned || id == run.node;
  }

  bool stopping = false;
  auto stop = [&]() {
    if (stopping) {
      return;
    }
    stopping = true;
    master.AsyncDeconfig().submit(exec, [&]() { ctx.shutdown(); });
  };
  io::SignalSet sigset(poll, exec);
  io::Timer deadline(poll, exec, CLOCK_MONOTONIC);
  int status = 0;
  run.finish = [&](bool ok, const std::string& fields) {
    if (run.finished) {
      return;
    }
    run.finished = true;
    const std::string head = "\"scenario\":\"" + run.scenario + "\"";
    if (ok) {
      // Counters come as JSON fragments, with or without their leading comma.
      const bool comma = fields.empty() || fields.front() == ',';
      emit("scenario_done", head + (comma ? "" : ",") + fields);
    } else {
      status = 1;
      emit("scenario_failed", head + ",\"reason\":\"" + lely_peer::json_escape(fields) + "\"");
    }
    sigset.clear();
    // Not from inside a driver callback or a scenario fiber: the loop stops the master.
    exec.post([&]() { stop(); });
  };
  const auto counters = [&]() {
    return run.scenario == "emcy_watch" ? num("emcy", static_cast<std::int64_t>(run.emcy))
                                        : std::string();
  };

  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);
  sigset.submit_wait([&](int) {
    sigset.clear();
    if (run.scenario == "emcy_watch") {
      run.finish(true, counters());
    } else {
      stop();
    }
  });
  if (duration_ms > 0) {
    deadline.settime(std::chrono::milliseconds(duration_ms));
    deadline.submit_wait([&](int, std::error_code ec) {
      if (ec) {
        return;  // cancelled: the run is already stopping
      }
      if (node_scenario) {
        run.finish(false, "deadline of " + std::to_string(duration_ms) + " ms passed");
      } else {
        run.finish(true, counters());
      }
    });
  }

  master.Reset();
  emit("start", node_field(master.id()) + ",\"ifname\":\"" + lely_peer::json_escape(ifname) +
                    "\",\"slaves\":[" + slaves + "]");
  if (node_scenario && !node_assigned) {
    run.finish(false, "node " + std::to_string(run.node) + " is not a slave in the DCF");
  }
  loop.run();
  emit("exit");
  return status;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const auto opts = lely_peer::parse_options(
      args, {"if", "dcf", "scenario", "node", "count", "duration-ms"}, {"if", "dcf", "scenario"});
  if (!opts.ok) {
    std::fprintf(stderr,
                 "lely_peer: %s\nusage: lely_peer --if IF --dcf PATH --scenario NAME [--node N] "
                 "[--count K] [--duration-ms MS]\n",
                 opts.error.c_str());
    return 2;
  }
  try {
    return run(opts);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "lely_peer: %s\n", e.what());
    return 1;
  }
}
