// SPDX-License-Identifier: Apache-2.0
#include "vdrive_bank/loop.hpp"

#include <algorithm>
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

void BankLoop::run_once() {
  ++counts_.wakes;
  const MonoTime now = now_();
  if (now >= next_release()) {
    release(now);
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
  return trace::Exit{at, ring_.overflow(), signal};
}

}  // namespace vdrive_bank
