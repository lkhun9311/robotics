// SPDX-License-Identifier: Apache-2.0
// Calibration slaves: one Lely BasicSlave per node ID, all on one I/O context, each serving the
// given EDS. They answer SDO and produce heartbeat as their object dictionary says; nothing else.
//   probe_slave --if IF --eds PATH --nodes 1-6
// Prints one JSON line {"event":"ready",...} once every slave has been reset, and exits 0 on
// SIGTERM or SIGINT.
#include <csignal>
#include <cstdio>
#include <exception>
#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "lely_peer/args.hpp"

namespace {

using namespace lely;

// A CAN channel that is open on the controller by the time the slave is constructed on it.
struct OpenChannel : io::CanChannel {
  OpenChannel(io::Poll& poll, ev::Executor exec, io::CanController& ctrl)
      : io::CanChannel(poll, exec) {
    open(ctrl);
  }
};

// One slave with its own timer and channel.
struct Probe {
  Probe(io::Poll& poll, ev::Executor exec, io::CanController& ctrl, const std::string& eds,
        std::uint8_t id)
      : timer(poll, exec, CLOCK_MONOTONIC),
        chan(poll, exec, ctrl),
        slave(timer, chan, eds, "", id) {}
  io::Timer timer;
  OpenChannel chan;
  canopen::BasicSlave slave;
};

int run(const lely_peer::Options& opts) {
  const auto nodes = lely_peer::parse_nodes(opts.values.at("nodes"));
  if (!nodes.ok) {
    std::fprintf(stderr, "probe_slave: --nodes: %s\n", nodes.error.c_str());
    return 2;
  }
  const std::string& ifname = opts.values.at("if");
  const std::string& eds = opts.values.at("eds");

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::CanController ctrl(ifname.c_str());
  std::vector<std::unique_ptr<Probe>> probes;
  probes.reserve(nodes.nodes.size());
  for (const auto id : nodes.nodes) {
    probes.push_back(std::make_unique<Probe>(poll, exec, ctrl, eds, id));
  }
  io::SignalSet sigset(poll, exec);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);
  sigset.submit_wait([&](int) {
    sigset.clear();
    ctx.shutdown();
  });
  std::string list;
  for (const auto& probe : probes) {
    probe->slave.Reset();
    list += (list.empty() ? "" : ",") + std::to_string(probe->slave.id());
  }
  std::printf("{\"event\":\"ready\",\"if\":\"%s\",\"nodes\":[%s]}\n",
              lely_peer::json_escape(ifname).c_str(), list.c_str());
  std::fflush(stdout);
  loop.run();
  std::printf("{\"event\":\"exit\"}\n");
  std::fflush(stdout);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const auto opts = lely_peer::parse_options(args, {"if", "eds", "nodes"}, {"if", "eds", "nodes"});
  if (!opts.ok) {
    std::fprintf(stderr, "probe_slave: %s\nusage: probe_slave --if IF --eds PATH --nodes 1-6\n",
                 opts.error.c_str());
    return 2;
  }
  try {
    return run(opts);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "probe_slave: %s\n", e.what());
    return 1;
  }
}
