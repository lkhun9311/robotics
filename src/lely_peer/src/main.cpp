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
//   scenario_done {scenario}              the scenario finished; the process then exits 0
//   exit   {}                             last line on a clean exit
//   lely_peer --if IF --dcf PATH --scenario NAME [--node N] [--duration-ms MS]
// Scenario `idle`: run without sending anything of its own until the duration ends (default:
// until SIGTERM/SIGINT).
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
#include <cstdio>
#include <ctime>
#include <exception>
#include <lely/co/nmt.hpp>
#include <memory>
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

// Prints one JSON line: the caller passes the fields after "event" as a JSON fragment.
void emit(const char* event, const std::string& fields = "") {
  std::printf("{\"t\":%.6f,\"event\":\"%s\"%s%s}\n", now_s(), event, fields.empty() ? "" : ",",
              fields.c_str());
  std::fflush(stdout);
}

std::string node_field(std::uint8_t id) { return "\"node\":" + std::to_string(id); }

class PeerDriver : public canopen::FiberDriver {
 public:
  using FiberDriver::FiberDriver;

 private:
  void OnState(canopen::NmtState st) noexcept override {
    emit("state", node_field(id()) + ",\"state\":" + std::to_string(static_cast<int>(st)));
  }

  void OnBoot(canopen::NmtState st, char es, const std::string& what) noexcept override {
    const std::string es_text = es != 0 ? std::string(1, es) : std::string();
    emit("boot", node_field(id()) + ",\"state\":" + std::to_string(static_cast<int>(st)) +
                     ",\"es\":\"" + lely_peer::json_escape(es_text) + "\",\"what\":\"" +
                     lely_peer::json_escape(what) + "\"");
  }

  void OnHeartbeat(bool occurred) noexcept override {
    emit("hb", node_field(id()) + ",\"occurred\":" + (occurred ? "true" : "false"));
  }

  void OnEmcy(std::uint16_t eec, std::uint8_t er, std::uint8_t /*msef*/[5]) noexcept override {
    emit("emcy", node_field(id()) + ",\"code\":" + std::to_string(eec) +
                     ",\"register\":" + std::to_string(er));
  }
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

int run(const lely_peer::Options& opts) {
  const std::string& scenario = opts.values.at("scenario");
  if (scenario != "idle") {
    std::fprintf(stderr, "lely_peer: unknown scenario '%s' (known: idle)\n", scenario.c_str());
    return 2;
  }
  std::uint64_t duration_ms = 0;
  if (const auto it = opts.values.find("duration-ms"); it != opts.values.end()) {
    const auto parsed = lely_peer::parse_number(it->second, 1, 3600000);
    if (!parsed.ok) {
      std::fprintf(stderr, "lely_peer: --duration-ms: %s\n", parsed.error.c_str());
      return 2;
    }
    duration_ms = parsed.value;
  }
  if (const auto it = opts.values.find("node"); it != opts.values.end()) {
    const auto parsed = lely_peer::parse_number(it->second, 1, 127);
    if (!parsed.ok) {
      std::fprintf(stderr, "lely_peer: --node: %s\n", parsed.error.c_str());
      return 2;
    }
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
  for (const auto id : assigned_slaves(master)) {
    drivers.push_back(std::make_unique<PeerDriver>(exec, master, id));
    slaves += (slaves.empty() ? "" : ",") + std::to_string(id);
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
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);
  sigset.submit_wait([&](int) {
    sigset.clear();
    stop();
  });
  io::Timer deadline(poll, exec, CLOCK_MONOTONIC);
  if (duration_ms > 0) {
    deadline.settime(std::chrono::milliseconds(duration_ms));
    deadline.submit_wait([&](int, std::error_code ec) {
      if (ec) {
        return;  // cancelled: the run is already stopping
      }
      emit("scenario_done", "\"scenario\":\"" + scenario + "\"");
      sigset.clear();
      stop();
    });
  }

  master.Reset();
  emit("start", node_field(master.id()) + ",\"ifname\":\"" + lely_peer::json_escape(ifname) +
                    "\",\"slaves\":[" + slaves + "]");
  loop.run();
  emit("exit");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const auto opts = lely_peer::parse_options(args, {"if", "dcf", "scenario", "node", "duration-ms"},
                                             {"if", "dcf", "scenario"});
  if (!opts.ok) {
    std::fprintf(stderr,
                 "lely_peer: %s\nusage: lely_peer --if IF --dcf PATH --scenario NAME [--node N] "
                 "[--duration-ms MS]\n",
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
