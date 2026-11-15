// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string_view>
#include <vector>

#include "drive_core/joint.hpp"
#include "runtime_core/time.hpp"
#include "socketcan_io/socket.hpp"
#include "vdrive_bank/bank.hpp"
#include "vdrive_bank/trace_ring.hpp"

// The control loop of the vdrive_bank process (B1). One thread; the caller waits (ppoll on the
// release timerfd, the CAN socket and the inject socket) and calls run_once on every wake. Each
// wake first runs a release that is due, then drains received frames until the next release is
// due or kRxBudget frames were read. Time comes from `now` only (CLOCK_MONOTONIC in the
// process); a received frame is dispatched with the instant read right after its recvmsg.
namespace vdrive_bank {

inline constexpr runtime_core::Duration kReleasePeriod = drive_core::kTickPeriod;  // W
inline constexpr int kRxBudget = 64;                // Frames read per wake at most.
inline constexpr std::uint64_t kOverloadSkips = 3;  // Consecutive skipped releases (spec §10.2).
inline constexpr std::uint16_t kOverloadErrorCode = 0x6100;  // CiA 301 internal software error.
inline constexpr runtime_core::Duration kSnapshotPeriod{1'000'000'000};

class BankLoop {
 public:
  // What the bank was given an instant for; reported to the time observer (a test seam).
  enum class Call : std::uint8_t { kPowerOn, kTick, kFrame, kInject };
  using TimeObserver = std::function<void(Call, runtime_core::MonoTime)>;

  // Release k (k >= 1) is due at t0 + k * W. The loop installs the bank's record sink.
  BankLoop(Bank& bank, socketcan_io::CanSocket& socket, TraceRing& ring,
           std::function<runtime_core::MonoTime()> now, runtime_core::MonoTime t0);
  BankLoop(const BankLoop&) = delete;
  BankLoop& operator=(const BankLoop&) = delete;

  // Traces `start` with the boot_ids, then powers on the six nodes and sends their boot-ups.
  void start(const std::array<std::uint32_t, kNodeCount>& boot_ids);
  // One wake (B1): the due release, if any, then the bounded drain.
  void run_once();
  // B6: applies one command at the current instant and sends the frames it produces.
  Bank::InjectResult inject(std::string_view command);
  // Every datagram waiting on the non-blocking UNIX datagram socket `fd`: applied in order
  // (inject), each answered to its sender with "ok" or "err <reason>". Failures of the socket
  // itself are counted and traced (note_inject_io): a receive error, a reply that cannot be
  // sent, and a datagram longer than the buffer, which is rejected rather than applied cut
  // short. Returns the number of datagrams read.
  int serve_inject(int fd);

  runtime_core::MonoTime next_release() const;
  const LoopCounts& counts() const { return counts_; }
  // The loop, dispatch (per node) and I/O snapshot records at `at`.
  std::vector<TraceRecord> snapshot(runtime_core::MonoTime at) const;
  TraceRecord exit_record(runtime_core::MonoTime at, int signal) const;
  // A failure of the inject socket (the process's I/O around inject()): counted and traced.
  void note_inject_io(trace::InjectIo::Kind kind, std::int64_t value);
  const trace::InjectCounts& inject_counts() const { return inject_counts_; }

  void set_time_observer(TimeObserver observer) { observer_ = std::move(observer); }

 private:
  runtime_core::MonoTime observe(Call call, runtime_core::MonoTime t) const;
  void send(const std::vector<canopen_codec::Frame>& frames);
  void release(runtime_core::MonoTime now);
  void overload(runtime_core::MonoTime now);

  Bank& bank_;
  socketcan_io::CanSocket& socket_;
  TraceRing& ring_;
  std::function<runtime_core::MonoTime()> now_;
  runtime_core::MonoTime t0_;
  std::uint64_t next_k_{1};  // Index of the next release.
  runtime_core::MonoTime next_snapshot_;
  LoopCounts counts_;
  trace::InjectCounts inject_counts_;
  TimeObserver observer_;
};

}  // namespace vdrive_bank
