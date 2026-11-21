// SPDX-License-Identifier: Apache-2.0
// The bus fault proxy process: forwards frames between two CAN interfaces through the rule
// engine.
//   bus_fault_proxy --host IF --device IF [--rules FILE] --trace PATH
// A frame read on --host is written only to --device and the reverse (B9); each socket is bound
// to one interface with CAN_RAW_RECV_OWN_MSGS off, so a written frame never comes back to the
// proxy.
// The trace (JSON lines) has a `start` record once both sockets are open and an `exit` record on
// SIGTERM or SIGINT with the forwarding counts, the per-rule counts, the socket counters and the
// forwarding cost of the normal path (B10: CLOCK_MONOTONIC right after recvmsg to right after write
// returns; a measurement, not a gate), with its distribution as [value, count] pairs ("dist",
// ascending, summing to "samples").
// Frames still held at exit are not written and are reported as dropped_at_exit.
// Exit 2: bad arguments or rules (the line is named), or an interface or the trace cannot be
// opened.
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "bus_fault_proxy/cost.hpp"
#include "bus_fault_proxy/engine.hpp"
#include "bus_fault_proxy/rules.hpp"
#include "socketcan_io/socket.hpp"

namespace {

using bus_fault_proxy::Dir;
using runtime_core::MonoTime;

constexpr const char* kUsage =
    "usage: bus_fault_proxy --host IF --device IF [--rules FILE] --trace PATH\n";
constexpr int kRxBudget = 64;                       // Frames read per socket per wake.
constexpr std::size_t kMaxCostSamples = 1U << 20U;  // Beyond this, samples are counted only.
constexpr std::int64_t kSnapshotPeriodNs = 1'000'000'000;
// Wakes in a row with a receive error on a socket whose interface is still up: then the error is
// persistent and the proxy stops rather than poll it forever.
constexpr int kMaxErrorWakes = 100;

MonoTime mono_now() {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return MonoTime{static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec};
}

struct Args {
  std::string host;
  std::string device;
  std::string rules;
  std::string trace;
  bool help{false};
  std::string error;
};

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      a.help = true;
      return a;
    }
    std::string* target = nullptr;
    if (arg == "--host") {
      target = &a.host;
    } else if (arg == "--device") {
      target = &a.device;
    } else if (arg == "--rules") {
      target = &a.rules;
    } else if (arg == "--trace") {
      target = &a.trace;
    } else {
      a.error = "unknown argument '" + std::string(arg) + "'";
      return a;
    }
    if (i + 1 >= argc) {
      a.error = std::string(arg) + " needs a value";
      return a;
    }
    *target = argv[++i];
  }
  if (a.host.empty() || a.device.empty() || a.trace.empty()) {
    a.error = "--host, --device and --trace are required";
  } else if (a.host == a.device) {
    a.error = "--host and --device must be different interfaces";
  }
  return a;
}

std::string io_json_of(const socketcan_io::IoCounts& c) {
  using socketcan_io::RxClass;
  const auto rej = [&c](RxClass k) { return c.rx_rejected[static_cast<std::size_t>(k)]; };
  std::ostringstream o;
  o << R"({"rx":)" << c.rx << R"(,"rx_rejected":{"rtr":)" << rej(RxClass::kRtr) << R"(,"error":)"
    << rej(RxClass::kError) << R"(,"fd":)" << rej(RxClass::kFd) << R"(,"extended_id":)"
    << rej(RxClass::kExtendedId) << R"(,"bad_length":)" << rej(RxClass::kBadLength)
    << R"(},"rx_error":)" << c.rx_error << R"(,"rx_reject_budget_hits":)" << c.rx_reject_budget_hits
    << R"(,"tx":)" << c.tx << R"(,"tx_eagain":)" << c.tx_eagain << R"(,"tx_enobufs":)"
    << c.tx_enobufs << R"(,"tx_other_error":)" << c.tx_other_error << '}';
  return o.str();
}

std::string json_string(std::string_view s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    if (static_cast<unsigned char>(c) >= 0x20U) {
      out += c;
    }
  }
  return out + '"';
}

// The p-th percentile (p in (0, 100]) of sorted samples, nearest rank: the smallest sample with
// at least p % of the samples at or below it.
std::int64_t percentile(const std::vector<std::int64_t>& sorted, std::size_t p) {
  if (sorted.empty()) {
    return 0;
  }
  const std::size_t rank = std::max<std::size_t>(1, (p * sorted.size() + 99) / 100);
  return sorted[rank - 1];
}

// Whether the interface `ifname` is administratively up, asked through socket `fd`.
bool interface_up(int fd, const std::string& ifname) {
  ifreq req{};
  std::memcpy(req.ifr_name, ifname.c_str(), std::min(ifname.size(), sizeof req.ifr_name - 1));
  if (::ioctl(fd, SIOCGIFFLAGS, &req) < 0) {
    return false;  // Gone (ENODEV) counts as down.
  }
  return (static_cast<unsigned>(req.ifr_flags) & IFF_UP) != 0U;
}

class Proxy {
 public:
  Proxy(socketcan_io::CanSocket host, socketcan_io::CanSocket device,
        std::vector<bus_fault_proxy::Rule> rules, MonoTime epoch)
      : host_(std::move(host)), device_(std::move(device)), engine_(std::move(rules), epoch) {
    costs_.reserve(1U << 16U);
  }

  socketcan_io::CanSocket& host() { return host_; }
  socketcan_io::CanSocket& device() { return device_; }
  const bus_fault_proxy::Engine& engine() const { return engine_; }

  void poll(MonoTime now) { write(engine_.poll(now)); }

  // Reads up to kRxBudget frames from the source of `dir` and forwards them.
  void drain(Dir dir) {
    socketcan_io::CanSocket& from = dir == Dir::kH2D ? host_ : device_;
    for (int i = 0; i < kRxBudget; ++i) {
      const std::optional<socketcan_io::RxFrame> rx = from.read_one();
      if (!rx) {
        return;
      }
      // The normal path: nothing held and no rule acting on this frame.
      const bool idle = !engine_.next_due().has_value();
      const std::uint64_t matched_before = matched();
      const std::vector<bus_fault_proxy::Out> outs = engine_.on_frame(dir, rx->frame, rx->rx_mono);
      write(outs);
      if (idle && matched() == matched_before && outs.size() == 1) {
        record_cost((mono_now() - rx->rx_mono).ns);
      }
    }
  }

  // Periodic counters (B7-style snapshot): both sockets and the forwarding counts.
  std::string io_json(MonoTime at) const {
    std::ostringstream o;
    o << R"({"event":"io","t_ns":)" << at.ns << R"(,"host":)" << io_json_of(host_.counts())
      << R"(,"device":)" << io_json_of(device_.counts()) << R"(,"forwarded":{"h2d":)"
      << forwarded_[0] << R"(,"d2h":)" << forwarded_[1] << R"(},"trace_overflow":0})";
    return o.str();
  }

  std::size_t drop_held() {
    return engine_.poll(MonoTime{std::numeric_limits<std::int64_t>::max()}).size();
  }

  std::string exit_json(MonoTime at, int signo, std::size_t dropped_at_exit) {
    std::sort(costs_.begin(), costs_.end());
    std::ostringstream o;
    o << R"({"event":"exit","t_ns":)" << at.ns << R"(,"trace_overflow":0,"signal":)" << signo
      << R"(,"forwarded":{"h2d":)" << forwarded_[0] << R"(,"d2h":)" << forwarded_[1]
      << R"(},"send_failed":{"h2d":)" << failed_[0] << R"(,"d2h":)" << failed_[1]
      << R"(},"dropped_at_exit":)" << dropped_at_exit << R"(,"cost_ns":{"count":)"
      << costs_.size() + costs_uncounted_ << R"(,"samples":)" << costs_.size() << R"(,"p50":)"
      << percentile(costs_, 50) << R"(,"p99":)" << percentile(costs_, 99) << R"(,"max":)"
      << (costs_.empty() ? 0 : costs_.back()) << R"(,"dist":[)";
    const std::vector<bus_fault_proxy::ValueCount> dist = bus_fault_proxy::run_lengths(costs_);
    for (std::size_t i = 0; i < dist.size(); ++i) {
      o << (i == 0 ? "[" : ",[") << dist[i].value << ',' << dist[i].count << ']';
    }
    o << R"(]},"rules":[)";
    const std::vector<bus_fault_proxy::RuleCounts>& counts = engine_.counts();
    for (std::size_t i = 0; i < counts.size(); ++i) {
      const bus_fault_proxy::RuleCounts& c = counts[i];
      o << (i == 0 ? "" : ",") << R"({"matched":)" << c.matched << R"(,"dropped":)" << c.dropped
        << R"(,"delayed":)" << c.delayed << R"(,"duplicated":)" << c.duplicated
        << R"(,"corrupted":)" << c.corrupted << R"(,"reordered":)" << c.reordered
        << R"(,"flushed":)" << c.flushed << '}';
    }
    o << R"(],"io":{"host":)" << io_json_of(host_.counts()) << R"(,"device":)"
      << io_json_of(device_.counts()) << "}}";
    return o.str();
  }

 private:
  std::uint64_t matched() const {
    std::uint64_t n = 0;
    for (const bus_fault_proxy::RuleCounts& c : engine_.counts()) {
      n += c.matched;
    }
    return n;
  }

  void write(const std::vector<bus_fault_proxy::Out>& outs) {
    for (const bus_fault_proxy::Out& out : outs) {
      const std::size_t d = out.dir == Dir::kH2D ? 0 : 1;
      socketcan_io::CanSocket& to = out.dir == Dir::kH2D ? device_ : host_;
      if (to.send(out.frame)) {
        ++forwarded_[d];
      } else {
        ++failed_[d];
      }
    }
  }

  void record_cost(std::int64_t ns) {
    if (costs_.size() < kMaxCostSamples) {
      costs_.push_back(ns);
    } else {
      ++costs_uncounted_;
    }
  }

  socketcan_io::CanSocket host_;
  socketcan_io::CanSocket device_;
  bus_fault_proxy::Engine engine_;
  std::array<std::uint64_t, 2> forwarded_{};
  std::array<std::uint64_t, 2> failed_{};
  std::vector<std::int64_t> costs_;
  std::uint64_t costs_uncounted_{0};
};

std::optional<socketcan_io::CanSocket> open_socket(const char* flag, const std::string& ifname) {
  std::string error;
  std::optional<socketcan_io::CanSocket> s = socketcan_io::CanSocket::open(ifname, &error);
  if (!s) {
    std::fprintf(stderr, "bus_fault_proxy: %s %s: %s\n", flag, ifname.c_str(), error.c_str());
    return std::nullopt;
  }
  // The CAN_RAW default, set explicitly: our own writes are never read back (B9).
  const int off = 0;
  if (::setsockopt(s->fd(), SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &off, sizeof off) < 0) {
    std::fprintf(stderr, "bus_fault_proxy: %s %s: CAN_RAW_RECV_OWN_MSGS: %s\n", flag,
                 ifname.c_str(), std::strerror(errno));
    return std::nullopt;
  }
  return s;
}

int run(const Args& args) {
  std::vector<bus_fault_proxy::Rule> rules;
  if (!args.rules.empty()) {
    std::ifstream in(args.rules);
    if (!in) {
      std::fprintf(stderr, "bus_fault_proxy: --rules %s: cannot read\n", args.rules.c_str());
      return 2;
    }
    std::stringstream text;
    text << in.rdbuf();
    bus_fault_proxy::ParseResult parsed = bus_fault_proxy::parse_rules(text.str());
    if (!parsed.error.empty()) {
      std::fprintf(stderr, "bus_fault_proxy: --rules %s: %s\n", args.rules.c_str(),
                   parsed.error.c_str());
      return 2;
    }
    rules = std::move(parsed.rules);
  }
  std::optional<socketcan_io::CanSocket> host = open_socket("--host", args.host);
  std::optional<socketcan_io::CanSocket> device = open_socket("--device", args.device);
  if (!host || !device) {
    return 2;
  }
  std::FILE* trace = std::fopen(args.trace.c_str(), "w");
  if (trace == nullptr) {
    std::fprintf(stderr, "bus_fault_proxy: --trace %s: %s\n", args.trace.c_str(),
                 std::strerror(errno));
    return 2;
  }
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGTERM);
  sigaddset(&signals, SIGINT);
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  const int signal_fd = ::signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC);
  if (signal_fd < 0) {
    std::fprintf(stderr, "bus_fault_proxy: signalfd: %s\n", std::strerror(errno));
    return 1;
  }

  const std::size_t rule_count = rules.size();
  const MonoTime epoch = mono_now();
  Proxy proxy(std::move(*host), std::move(*device), std::move(rules), epoch);
  std::fprintf(trace,
               "{\"event\":\"start\",\"t_ns\":%lld,\"host\":%s,\"device\":%s,\"rules\":%zu}\n",
               static_cast<long long>(epoch.ns), json_string(args.host).c_str(),
               json_string(args.device).c_str(), rule_count);
  std::fflush(trace);

  int signo = 0;
  int rc = 0;
  std::string fatal;  // Why the proxy stops on its own; empty while it runs.
  MonoTime next_snapshot{epoch.ns + kSnapshotPeriodNs};
  std::array<int, 2> error_wakes{};  // Per source: wakes in a row with a new receive error.
  const std::array<const std::string*, 2> names{&args.host, &args.device};
  while (signo == 0 && fatal.empty()) {
    std::array<pollfd, 3> fds{
        {{proxy.host().fd(), POLLIN, 0}, {proxy.device().fd(), POLLIN, 0}, {signal_fd, POLLIN, 0}}};
    // Wake for the next held frame (a delay, a reorder hold, or a `delay 0` frame) and for the
    // next snapshot.
    MonoTime wake = next_snapshot;
    if (const std::optional<MonoTime> due = proxy.engine().next_due()) {
      wake = std::min(wake, *due);
    }
    const std::int64_t left = std::max<std::int64_t>(0, (wake - mono_now()).ns);
    timespec timeout{};
    timeout.tv_sec = static_cast<time_t>(left / 1'000'000'000);
    timeout.tv_nsec = static_cast<long>(left % 1'000'000'000);
    if (::ppoll(fds.data(), fds.size(), &timeout, nullptr) < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::fprintf(stderr, "bus_fault_proxy: ppoll: %s\n", std::strerror(errno));
      rc = 1;
      break;
    }
    if ((fds[2].revents & POLLIN) != 0) {
      signalfd_siginfo info{};
      if (::read(signal_fd, &info, sizeof info) == static_cast<ssize_t>(sizeof info)) {
        signo = static_cast<int>(info.ssi_signo);
      }
      break;
    }
    proxy.poll(mono_now());
    // POLLERR/POLLHUP are drained too: read_one consumes and counts the socket error (rx_error);
    // left unread, a pending error makes ppoll return at once forever.
    constexpr short kReadable = POLLIN | POLLERR | POLLHUP;
    for (std::size_t i = 0; i < 2; ++i) {
      if ((fds[i].revents & kReadable) == 0) {
        continue;
      }
      socketcan_io::CanSocket& sock = i == 0 ? proxy.host() : proxy.device();
      const std::uint64_t errors_before = sock.counts().rx_error;
      proxy.drain(i == 0 ? Dir::kH2D : Dir::kD2H);
      if (sock.counts().rx_error == errors_before) {
        error_wakes[i] = 0;
        continue;
      }
      const std::string what =
          *names[i] + ": receive error: " + std::strerror(sock.last_rx_errno());
      if (!interface_up(sock.fd(), *names[i])) {
        fatal = what + "; the interface is down";
      } else if (++error_wakes[i] >= kMaxErrorWakes) {
        fatal = what + "; persistent (" + std::to_string(kMaxErrorWakes) + " wakes in a row)";
      }
    }
    const MonoTime now = mono_now();
    if (now >= next_snapshot) {
      std::fprintf(trace, "%s\n", proxy.io_json(now).c_str());
      std::fflush(trace);
      next_snapshot =
          MonoTime{next_snapshot.ns +
                   ((now - next_snapshot).ns / kSnapshotPeriodNs + 1) * kSnapshotPeriodNs};
    }
  }
  if (!fatal.empty()) {
    std::fprintf(stderr, "bus_fault_proxy: %s\n", fatal.c_str());
    std::fprintf(trace, "{\"event\":\"error\",\"t_ns\":%lld,\"reason\":%s}\n",
                 static_cast<long long>(mono_now().ns), json_string(fatal).c_str());
    rc = 1;
  }
  std::fprintf(trace, "%s\n", proxy.io_json(mono_now()).c_str());

  const std::size_t dropped = proxy.drop_held();
  const std::string exit_line = proxy.exit_json(mono_now(), signo, dropped);
  std::fprintf(trace, "%s\n", exit_line.c_str());
  if (std::fflush(trace) != 0 || std::ferror(trace) != 0) {
    std::fprintf(stderr, "bus_fault_proxy: writing %s failed\n", args.trace.c_str());
    rc = 1;
  }
  std::fclose(trace);
  return rc;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.help) {
    std::fputs(kUsage, stdout);
    return 0;
  }
  if (!args.error.empty()) {
    std::fprintf(stderr, "bus_fault_proxy: %s\n%s", args.error.c_str(), kUsage);
    return 2;
  }
  try {
    return run(args);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "bus_fault_proxy: %s\n", e.what());
    return 1;
  }
}
