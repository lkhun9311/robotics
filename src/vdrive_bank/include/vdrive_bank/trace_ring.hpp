// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <variant>

#include "drive_core/drive.hpp"
#include "runtime_core/time.hpp"
#include "socketcan_io/socket.hpp"
#include "vdrive_bank/bank.hpp"

// The trace of the vdrive_bank process (B7). The control thread pushes fixed-size records into a
// bounded single-producer single-consumer ring and never waits; a writer thread serializes them
// as JSON lines. A full ring drops the record and counts it in trace_overflow, and a trace with
// overflow is incomplete.
namespace vdrive_bank {

// Loop counters of B1.
struct LoopCounts {
  std::uint64_t releases_planned{};         // Releases whose due time has passed.
  std::uint64_t releases_executed{};        // Releases that ran the tick of all six drives.
  std::uint64_t releases_skipped{};         // Releases that passed without a tick; never caught up.
  std::uint64_t consecutive_skipped{};      // Skipped releases right before the latest tick.
  std::uint64_t max_consecutive_skipped{};  // The largest such run so far.
  std::uint64_t rx_budget_hits{};           // Wakes whose drain stopped at kRxBudget frames.
  std::uint64_t overloads{};                // Overload reactions (B1, spec §10.2).
  std::uint64_t frames_dispatched{};        // Received frames handed to the bank.
  std::uint64_t wakes{};                    // run_once calls.
};

namespace trace {

// Process start: the first boot_id of nodes 1..6 and the release origin t0.
struct Start {
  runtime_core::MonoTime at;
  runtime_core::MonoTime t0;
  std::array<std::uint32_t, kNodeCount> boot_ids;
};

// kOverloadSkips consecutive releases were skipped; every drive got the overload fault.
struct Overload {
  runtime_core::MonoTime at;
  std::uint64_t consecutive_skipped;
  std::uint16_t error_code;
};

struct LoopSnapshot {
  runtime_core::MonoTime at;
  LoopCounts loop;
  IgnoredCounts ignored;
  std::uint64_t trace_overflow;
};

// One node's drive_core::DispatchCounts.
struct DispatchSnapshot {
  runtime_core::MonoTime at;
  std::uint8_t node;
  drive_core::DispatchCounts counts;
  std::uint64_t trace_overflow;
};

// The CAN socket's counters.
struct IoSnapshot {
  runtime_core::MonoTime at;
  socketcan_io::IoCounts io;
  std::uint64_t trace_overflow;
};

// Failures of the inject socket (B6) itself, not of a command: never silent.
struct InjectCounts {
  std::uint64_t recv_errors{};  // recvfrom failures other than EAGAIN/EWOULDBLOCK/EINTR.
  std::uint64_t send_errors{};  // Replies that could not be sent.
  std::uint64_t truncated{};    // Datagrams longer than the command buffer: rejected, not applied.
};

// One inject socket failure, with the totals after it.
struct InjectIo {
  enum class Kind : std::uint8_t { kRecvError, kSendError, kTruncated };
  runtime_core::MonoTime at;
  Kind kind;
  std::int64_t value;  // errno for the errors; the datagram's length for kTruncated.
  InjectCounts totals;
};

// Normal end of the process: the last record of a complete trace.
struct Exit {
  runtime_core::MonoTime at;
  std::uint64_t trace_overflow;
  int signal;  // The signal that ended the process; 0 if none.
  InjectCounts inject;
};

}  // namespace trace

// A tagged union: the variant index is the tag.
using TraceRecord =
    std::variant<BankRecord, trace::Start, trace::Overload, trace::LoopSnapshot,
                 trace::DispatchSnapshot, trace::IoSnapshot, trace::InjectIo, trace::Exit>;
static_assert(std::is_trivially_copyable_v<TraceRecord>);
static_assert(sizeof(TraceRecord) <= 128);

// One JSON object without a trailing newline. Every record has "event" and "t_ns".
std::string to_json(const TraceRecord& record);

// Bounded SPSC ring. push is called by one thread (the control thread) and pop by one other
// (the writer); neither ever waits for the other.
class TraceRing {
 public:
  static constexpr std::size_t kCapacity = 4096;

  TraceRing();
  TraceRing(const TraceRing&) = delete;
  TraceRing& operator=(const TraceRing&) = delete;

  // false when the ring is full: the record is dropped and counted in overflow().
  bool push(const TraceRecord& record);
  std::optional<TraceRecord> pop();
  std::uint64_t overflow() const { return overflow_.load(std::memory_order_relaxed); }

 private:
  std::unique_ptr<TraceRecord[]> slots_;
  alignas(64) std::atomic<std::uint64_t> head_{0};  // Next slot to write; producer only.
  alignas(64) std::atomic<std::uint64_t> tail_{0};  // Next slot to read; consumer only.
  alignas(64) std::atomic<std::uint64_t> overflow_{0};
};

// The consumer of a ring on its own thread. `sink` gets every record in order; `flush` runs after
// each batch, so a reader of the file sees records within one idle period.
class TraceWriter {
 public:
  using Sink = std::function<void(const TraceRecord&)>;
  using Flush = std::function<void()>;

  TraceWriter(TraceRing& ring, Sink sink, Flush flush = {},
              std::chrono::microseconds idle = std::chrono::milliseconds(2));
  TraceWriter(const TraceWriter&) = delete;
  TraceWriter& operator=(const TraceWriter&) = delete;
  ~TraceWriter();

  void start();
  // Writes every record still in the ring, then joins the thread. Idempotent.
  void stop();
  std::uint64_t written() const { return written_.load(std::memory_order_relaxed); }

 private:
  void run();
  bool drain();

  TraceRing& ring_;
  Sink sink_;
  Flush flush_;
  std::chrono::microseconds idle_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> written_{0};
  std::thread thread_;
};

}  // namespace vdrive_bank
