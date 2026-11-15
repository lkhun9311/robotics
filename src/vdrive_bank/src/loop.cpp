// SPDX-License-Identifier: Apache-2.0
#include "vdrive_bank/loop.hpp"

#include <sys/socket.h>
#include <sys/un.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <string>
#include <utility>

namespace vdrive_bank {

using runtime_core::Duration;
using runtime_core::MonoTime;

BankLoop::BankLoop(Bank& bank, socketcan_io::CanSocket& socket, TraceRing& ring,
                   std::function<MonoTime()> now, MonoTime t0)
    : bank_(bank),
      socket_(socket),
      ring_(ring),
      now_(std::move(now)),
      t0_(t0),
      next_snapshot_(t0 + kSnapshotPeriod) {
  bank_.set_sink([this](const BankRecord& r) { ring_.push(r); });
}

MonoTime BankLoop::observe(Call call, MonoTime t) const {
  if (observer_) {
    observer_(call, t);
  }
  return t;
}

void BankLoop::send(const std::vector<canopen_codec::Frame>& frames) {
  // A failed send is counted by the socket (IoCounts) and is not retried: a late CANopen frame
  // is worse than a lost one, and the counters are in every snapshot.
  for (const canopen_codec::Frame& f : frames) {
    socket_.send(f);
  }
}

void BankLoop::start(const std::array<std::uint32_t, kNodeCount>& boot_ids) {
  const MonoTime now = observe(Call::kPowerOn, now_());
  ring_.push(trace::Start{now, t0_, boot_ids});
  send(bank_.power_on(now));
}

MonoTime BankLoop::next_release() const {
  return t0_ + Duration{static_cast<std::int64_t>(next_k_) * kReleasePeriod.ns};
}

void BankLoop::release(MonoTime now) {
  // Every release due by now: the latest runs, the ones before it are skipped. Drive::tick
  // advances the model by exactly one W, so a catch-up would compress time.
  const auto due = static_cast<std::uint64_t>((now - next_release()).ns / kReleasePeriod.ns) + 1U;
  next_k_ += due;
  counts_.releases_planned += due;
  counts_.releases_executed += 1;
  counts_.releases_skipped += due - 1U;
  counts_.consecutive_skipped = due - 1U;
  counts_.max_consecutive_skipped =
      std::max(counts_.max_consecutive_skipped, counts_.consecutive_skipped);
  send(bank_.tick(observe(Call::kTick, now)));
  if (counts_.consecutive_skipped >= kOverloadSkips) {
    overload(now);
  }
  if (now >= next_snapshot_) {
    for (const TraceRecord& r : snapshot(now)) {
      ring_.push(r);
    }
    const auto periods = (now - next_snapshot_).ns / kSnapshotPeriod.ns + 1;
    next_snapshot_ = next_snapshot_ + Duration{periods * kSnapshotPeriod.ns};
  }
}

void BankLoop::overload(MonoTime now) {
  // Spec §10.2 three-miss block: the bank cannot keep its period, so every drive faults with
  // CiA 301 0x6100 (internal software error). A drive already handling a fault keeps its first.
  ++counts_.overloads;
  ring_.push(trace::Overload{now, counts_.consecutive_skipped, kOverloadErrorCode});
  char hex[8];
  std::snprintf(hex, sizeof hex, "%04X", static_cast<unsigned>(kOverloadErrorCode));
  for (std::size_t node = 1; node <= kNodeCount; ++node) {
    const std::string command = "fault " + std::to_string(node) + " " + hex;
    send(bank_.inject(command, observe(Call::kInject, now)).frames);
  }
}

void BankLoop::run_once(int inject_fd) {
  ++counts_.wakes;
  const MonoTime now = now_();
  if (now >= next_release()) {
    release(now);
  }
  // The inject drain comes after the release and before the CAN drain: the CAN drain runs to
  // the release deadline under sustained traffic, so served before the release or after the CAN
  // drain the injects would get only the one datagram a late wake allows.
  if (inject_fd >= 0) {
    serve_inject(inject_fd);
  }
  // The drain: bounded by the next release deadline and by kRxBudget frames. Backlog waits for
  // the next wake (the socket stays readable, so ppoll returns at once).
  int reads = 0;
  while (now_() < next_release()) {
    if (reads == kRxBudget) {
      ++counts_.rx_budget_hits;
      break;
    }
    const std::optional<socketcan_io::RxFrame> rx = socket_.read_one();
    if (!rx) {
      break;  // Drained, a receive error or the reject budget: the counters tell which.
    }
    ++reads;
    ++counts_.frames_dispatched;
    send(bank_.on_frame(rx->frame, observe(Call::kFrame, rx->rx_mono)));
  }
}

Bank::InjectResult BankLoop::inject(std::string_view command) {
  Bank::InjectResult res = bank_.inject(command, observe(Call::kInject, now_()));
  send(res.frames);
  return res;
}

int BankLoop::serve_inject(int fd) {
  using Kind = trace::InjectIo::Kind;
  // Bounded like the CAN drain: at most kInjectBudget datagrams, and none after the next release
  // is due, except the first, so a wake that starts late still makes progress. The rest stays
  // queued in the socket, in order, for the next wake (the socket stays readable).
  int served = 0;
  while (served < kInjectBudget && (served == 0 || now_() < next_release())) {
    std::array<char, 128> buf{};
    sockaddr_un peer{};
    socklen_t peer_len = sizeof peer;
    // MSG_TRUNC: the return value is the datagram's real length, even when it did not fit.
    const ssize_t n = ::recvfrom(fd, buf.data(), buf.size(), MSG_TRUNC,
                                 reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        note_inject_io(Kind::kRecvError, errno);
      }
      return served;
    }
    ++served;
    std::string reply;
    if (static_cast<std::size_t>(n) > buf.size()) {
      note_inject_io(Kind::kTruncated, n);
      reply = "err datagram truncated: " + std::to_string(n) + " bytes, at most " +
              std::to_string(buf.size());
    } else {
      std::string_view command(buf.data(), static_cast<std::size_t>(n));
      while (!command.empty() && (command.back() == '\n' || command.back() == '\r')) {
        command.remove_suffix(1);
      }
      const Bank::InjectResult res = inject(command);
      reply = res.ok ? std::string("ok") : "err " + res.error;
    }
    if (peer_len <= sizeof(sa_family_t)) {
      continue;  // An unbound sender cannot be answered.
    }
    if (::sendto(fd, reply.data(), reply.size(), MSG_DONTWAIT,
                 reinterpret_cast<const sockaddr*>(&peer), peer_len) < 0) {
      note_inject_io(Kind::kSendError, errno);
    }
  }
  return served;
}

std::vector<TraceRecord> BankLoop::snapshot(MonoTime at) const {
  const std::uint64_t overflow = ring_.overflow();
  std::vector<TraceRecord> out;
  out.reserve(kNodeCount + 2U);
  out.emplace_back(trace::LoopSnapshot{at, counts_, bank_.ignored(), overflow});
  for (std::size_t node = 1; node <= kNodeCount; ++node) {
    const auto id = static_cast<std::uint8_t>(node);
    out.emplace_back(trace::DispatchSnapshot{at, id, bank_.drive(id).counts(), overflow});
  }
  out.emplace_back(trace::IoSnapshot{at, socket_.counts(), overflow});
  return out;
}

TraceRecord BankLoop::exit_record(MonoTime at, int signal) const {
  return trace::Exit{at, ring_.overflow(), signal, inject_counts_};
}

void BankLoop::note_inject_io(trace::InjectIo::Kind kind, std::int64_t value) {
  switch (kind) {
    case trace::InjectIo::Kind::kRecvError:
      ++inject_counts_.recv_errors;
      break;
    case trace::InjectIo::Kind::kSendError:
      ++inject_counts_.send_errors;
      break;
    case trace::InjectIo::Kind::kTruncated:
      ++inject_counts_.truncated;
      break;
  }
  ring_.push(trace::InjectIo{now_(), kind, value, inject_counts_});
}

}  // namespace vdrive_bank
