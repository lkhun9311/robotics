// SPDX-License-Identifier: Apache-2.0
// The vdrive_bank process: six virtual drives (node IDs 1..6) on one CAN interface.
//   vdrive_bank --if IF --trace PATH [--inject PATH] [--hb-muted NODES]
// One control thread waits in ppoll on the release timerfd (absolute CLOCK_MONOTONIC, period W),
// the CAN socket, the inject socket and a signalfd, and runs BankLoop::run_once on every wake
// (B1).
// A writer thread writes the trace as JSON lines to PATH (B7). --inject PATH binds a UNIX datagram
// socket for B6 commands; each is answered with "ok" or "err <reason>".
// At most kInjectBudget commands are served per wake, so a client that keeps the socket full cannot
// hold off the releases, the CAN drain or a signal.
// SIGTERM or SIGINT: the writer is flushed, the final snapshots and an `exit` record are written,
// exit 0.
// Exit 2: bad arguments, or the interface, trace or inject socket cannot be opened (the reason
// is on stderr).
// Exit 1: a failure while running; once the trace is open it still ends with the final snapshots
// and an `exit` record, and the inject socket file is removed on every exit.
// --hb-muted NODES (comma-separated node IDs, e.g. "2" or "2,5") applies B6 `hb_mute <node> 1`
// before power-on (B3): a peer started first sees the boot-up and then no periodic heartbeat.
// The mutes are traced as inject records.
#include <poll.h>
#include <pthread.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "socketcan_io/socket.hpp"
#include "vdrive_bank/bank.hpp"
#include "vdrive_bank/entropy.hpp"
#include "vdrive_bank/loop.hpp"
#include "vdrive_bank/trace_ring.hpp"

namespace {

using runtime_core::MonoTime;

constexpr const char* kUsage =
    "usage: vdrive_bank --if IF --trace PATH [--inject PATH] [--hb-muted NODES]\n";

MonoTime mono_now() {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return MonoTime{static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec};
}

timespec to_timespec(MonoTime t) {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t.ns / 1'000'000'000);
  ts.tv_nsec = static_cast<long>(t.ns % 1'000'000'000);
  return ts;
}

struct Args {
  std::string ifname;
  std::string trace;
  std::string inject;
  std::vector<std::uint8_t> hb_muted;  // Nodes muted before power-on.
  bool help{false};
  std::string error;  // Empty when the arguments are usable.
};

// "2" or "2,5": node IDs 1..kNodeCount, each once.
bool parse_muted(std::string_view text, std::vector<std::uint8_t>& out) {
  out.clear();
  while (true) {
    const std::size_t comma = text.find(',');
    const std::string_view item = text.substr(0, comma);
    if (item.size() != 1 || item[0] < '1' ||
        item[0] > static_cast<char>('0' + vdrive_bank::kNodeCount)) {
      return false;
    }
    const auto node = static_cast<std::uint8_t>(item[0] - '0');
    if (std::find(out.begin(), out.end(), node) != out.end()) {
      return false;
    }
    out.push_back(node);
    if (comma == std::string_view::npos) {
      return true;
    }
    text.remove_prefix(comma + 1);
  }
}

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      a.help = true;
      return a;
    }
    if (arg == "--hb-muted") {
      if (i + 1 >= argc) {
        a.error = "--hb-muted needs a value";
        return a;
      }
      ++i;
      if (!parse_muted(argv[i], a.hb_muted)) {
        a.error = "--hb-muted needs comma-separated node IDs 1..6";
        return a;
      }
      continue;
    }
    std::string* target = nullptr;
    if (arg == "--if") {
      target = &a.ifname;
    } else if (arg == "--trace") {
      target = &a.trace;
    } else if (arg == "--inject") {
      target = &a.inject;
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
  if (a.ifname.empty()) {
    a.error = "--if is required";
  } else if (a.trace.empty()) {
    a.error = "--trace is required";
  }
  return a;
}

// A bound, non-blocking UNIX datagram socket at `path` (an old socket file is replaced).
int bind_inject(const std::string& path, std::string* error) {
  sockaddr_un addr{};
  if (path.size() >= sizeof addr.sun_path) {
    *error = "path too long";
    return -1;
  }
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    *error = std::string("socket: ") + std::strerror(errno);
    return -1;
  }
  addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
  ::unlink(path.c_str());
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) < 0) {
    *error = std::string("bind: ") + std::strerror(errno);
    ::close(fd);
    return -1;
  }
  return fd;
}

// The bound inject socket: closed and its file removed on every way out of run(), exceptions
// included.
struct InjectSocket {
  int fd{-1};
  std::string path;
  InjectSocket() = default;
  InjectSocket(const InjectSocket&) = delete;
  InjectSocket& operator=(const InjectSocket&) = delete;
  ~InjectSocket() {
    if (fd >= 0) {
      ::close(fd);
      ::unlink(path.c_str());
    }
  }
};

struct CloseFile {
  void operator()(std::FILE* f) const { std::fclose(f); }
};

int run(const Args& args) {
  std::string error;
  std::optional<socketcan_io::CanSocket> sock = socketcan_io::CanSocket::open(args.ifname, &error);
  if (!sock) {
    std::fprintf(stderr, "vdrive_bank: --if %s: %s\n", args.ifname.c_str(), error.c_str());
    return 2;
  }
  // Blocked before the writer thread starts, so only the signalfd ever sees them.
  // The signalfd and the timerfd are made before the trace and the inject socket exist, so a
  // failure here leaves nothing behind.
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGTERM);
  sigaddset(&signals, SIGINT);
  pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  const int signal_fd = ::signalfd(-1, &signals, SFD_NONBLOCK | SFD_CLOEXEC);
  const int timer_fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  if (signal_fd < 0 || timer_fd < 0) {
    std::fprintf(stderr, "vdrive_bank: signalfd/timerfd: %s\n", std::strerror(errno));
    return 1;
  }
  // Declared before the writer, so the writer has stopped before the file is closed.
  const std::unique_ptr<std::FILE, CloseFile> trace_file(std::fopen(args.trace.c_str(), "w"));
  std::FILE* const trace = trace_file.get();
  if (trace == nullptr) {
    std::fprintf(stderr, "vdrive_bank: --trace %s: %s\n", args.trace.c_str(), std::strerror(errno));
    return 2;
  }
  InjectSocket inject;
  if (!args.inject.empty()) {
    inject.fd = bind_inject(args.inject, &error);
    if (inject.fd < 0) {
      std::fprintf(stderr, "vdrive_bank: --inject %s: %s\n", args.inject.c_str(), error.c_str());
      return 2;
    }
    inject.path = args.inject;
  }
  const int inject_fd = inject.fd;

  vdrive_bank::GetrandomEntropy entropy;
  const std::array<std::uint32_t, vdrive_bank::kNodeCount> boot_ids =
      vdrive_bank::draw_boot_ids(entropy);
  const MonoTime t0 = mono_now();
  vdrive_bank::Bank bank(boot_ids, t0);
  vdrive_bank::TraceRing ring;
  vdrive_bank::BankLoop loop(bank, *sock, ring, mono_now, t0);
  const auto write_record = [trace](const vdrive_bank::TraceRecord& r) {
    std::string line = vdrive_bank::to_json(r);
    line += '\n';
    std::fwrite(line.data(), 1, line.size(), trace);
  };
  vdrive_bank::TraceWriter writer(ring, write_record, [trace] { std::fflush(trace); });
  writer.start();
  // From here on a failure ends the run through the normal shutdown below (rc 1): the trace
  // still gets the final snapshots and its `exit` record.
  // A start that fails from here on is named in the trace (start_failed) before its exit record.
  int rc = 0;
  std::string start_error;
  for (const std::uint8_t node : args.hb_muted) {
    const std::string command = "hb_mute " + std::to_string(node) + " 1";
    if (!loop.inject(command).ok) {
      start_error = command + " was refused";
      break;
    }
  }
  if (start_error.empty()) {
    loop.start(boot_ids);
    // Release k at t0 + k W: absolute, so the timer keeps the phase whatever the loop does.
    itimerspec spec{};
    spec.it_value = to_timespec(loop.next_release());
    spec.it_interval = to_timespec(MonoTime{vdrive_bank::kReleasePeriod.ns});
    if (::timerfd_settime(timer_fd, TFD_TIMER_ABSTIME, &spec, nullptr) < 0) {
      start_error = std::string("timerfd_settime: ") + std::strerror(errno);
    }
  }
  if (!start_error.empty()) {
    std::fprintf(stderr, "vdrive_bank: %s\n", start_error.c_str());
    rc = 1;
  }

  int signo = 0;
  while (rc == 0 && signo == 0) {
    std::array<pollfd, 4> fds{{{timer_fd, POLLIN, 0},
                               {sock->fd(), POLLIN, 0},
                               {signal_fd, POLLIN, 0},
                               {inject_fd, POLLIN, 0}}};
    const nfds_t count = inject_fd >= 0 ? 4 : 3;
    if (::ppoll(fds.data(), count, nullptr, nullptr) < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::fprintf(stderr, "vdrive_bank: ppoll: %s\n", std::strerror(errno));
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
    if ((fds[0].revents & POLLIN) != 0) {
      std::uint64_t expirations = 0;
      // The count is not used: releases are counted from the clock (B1).
      [[maybe_unused]] const ssize_t n = ::read(timer_fd, &expirations, sizeof expirations);
    }
    // A backlog left by the inject budget keeps the socket readable: the next ppoll returns at
    // once and the next wake serves it.
    const bool inject_ready = inject_fd >= 0 && (fds[3].revents & POLLIN) != 0;
    loop.run_once(inject_ready ? inject_fd : -1);
  }

  writer.stop();
  const MonoTime end = mono_now();
  if (!start_error.empty()) {
    write_record(vdrive_bank::trace::start_failed(end, start_error));
  }
  for (const vdrive_bank::TraceRecord& r : loop.snapshot(end)) {
    write_record(r);
  }
  write_record(loop.exit_record(end, signo));
  if (std::fflush(trace) != 0 || std::ferror(trace) != 0) {
    std::fprintf(stderr, "vdrive_bank: writing %s failed\n", args.trace.c_str());
    rc = 1;
  }
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
    std::fprintf(stderr, "vdrive_bank: %s\n%s", args.error.c_str(), kUsage);
    return 2;
  }
  try {
    return run(args);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "vdrive_bank: %s\n", e.what());
    return 1;
  }
}
