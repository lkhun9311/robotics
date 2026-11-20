// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "canopen_backend/canopen_backend.hpp"
#include "canopen_backend/transport.hpp"
#include "canopen_codec/codec.hpp"
#include "canopen_master/events.hpp"
#include "canopen_master/master.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

// The I/O worker (C10, C13): one thread that owns the bus and runs the CANopen backend on it.
//
// THREAD OWNERSHIP
//   worker thread  The only thread that touches the Transport (the CAN socket), the Master and
//                  the VdriveSession (inside CanopenBackend). It is the thread start() creates,
//                  or, in logic-time tests, the caller of run_once() (never both: run_once
//                  throws once start() succeeded).
//   control thread Every other public member, and only one thread at a time (like any Backend).
//                  It touches only the channels below, never the bus or the session.
//   channels       control -> worker: the one-slot command box (an overwrite before the worker
//                  took the previous command is counted as superseded), the op queue (bounded
//                  kOpQueueCapacity), the cancel queue, the raw command queue (master-only) and
//                  the inhibit and shutdown requests. worker -> control: the one-slot feedback,
//                  capability and stats boxes, the op table, the last SYNC instant and the event
//                  queue (bounded kEventQueueCapacity; full is the fault kEventOverflow). Every
//                  channel is single-producer single-consumer and lock-free; only wait() and
//                  await() block, on a condition variable the worker signals after each wake.
//
// EACH WAKE of the worker (ppoll on an absolute timerfd, the socket and a wake eventfd; the
// first step waits for the first timer expiry, so the SYNC grid lies on the timer grid). The
// timer period is W (2 ms), not T_bus as C10 states: the master still produces SYNC every T_bus
// (the grid is a multiple of W), and W is the drive's own granularity for op deadlines, retries,
// stop polling and the dispatch margin, so none of them waits up to a whole T_bus. the inhibit and
// shutdown requests, the queued ops, the cancels, the raw commands, the newest command from the
// box, then one backend step reading at most `rx_budget` frames (B1), then the dispatch records,
// the op table, feedback, capabilities and stats are published.
//
// OPS (C10): every op carries its id and absolute deadline from the OpTicket the caller made
// when it posted it. The session fails an op whose deadline passed whether or not anyone waits;
// a wait() that times out fails it too: wait() posts the cancel and blocks (at most
// cancel_ack_timeout, 3 W) until a wake has put it into the session (the epoch moves on, so that
// wake and every later one send nothing of the op; a cancelled arm fences); only then does it
// return. If no wake takes it in time, the inhibit is latched instead. An op the worker had already
// completed when the cancel arrived latches the inhibit (counted as late_completions): the caller
// was told it failed.
//
// TEARDOWN (C13): the Worker owns everything the thread touches through one shared state that
// the thread also holds. shutdown() posts the shutdown (a group stop when arming, armed or
// deactivating, then exit once no stop is pending or kShutdownStopLimit passed), wakes the
// worker and joins within a timeout; false (retryable) while it still runs. The destructor does
// the same with a 1 s timeout and otherwise detaches the thread, which keeps the shared state
// alive until it exits. The worker releases the transport (closes the socket) when it exits.
// Ops still queued, or posted, once the shutdown is seen fail without starting.
//
// FAULTS of the worker itself latch for the Worker's lifetime (a configure does not clear them;
// a new configure cycle uses a new Worker): a full event queue (kEventOverflow) and the loss of
// the transport (kTransportLost: a receive error, or a write error other than EAGAIN or
// ENOBUFS, which are a full queue and only counted, spec §4 IPC policy). Both inhibit.
// on_error_nonblocking() requests the inhibit and the shutdown without waiting.
//
// A master-only worker (the console of canopen_master_cli) runs a bare Master over the
// transport instead of the backend: every Backend op is refused, raw commands drive the master,
// and every master event is forwarded.
namespace canopen_backend {

inline constexpr std::size_t kOpQueueCapacity = 8;
inline constexpr std::size_t kEventQueueCapacity = 256;
inline constexpr std::size_t kRawQueueCapacity = 32;
inline constexpr std::size_t kRxBudget = 64;                              // Frames per wake (B1).
inline constexpr runtime_core::Duration kWakePeriod{2'000'000};           // W
inline constexpr runtime_core::Duration kShutdownStopLimit{250'000'000};  // > C18's 172 ms
inline constexpr std::size_t kMaxEventPdoFields = 8;

// One record for the control side, fixed size (the queue never allocates).
struct WorkerEvent {
  enum class Kind : std::uint8_t {
    kMaster,    // A master event (master_kind and its detail below).
    kDispatch,  // The session sent a new wire command_seq (C6), once per seq.
    kCmdDone,   // A raw command was carried out (ok) or refused.
  };
  Kind kind{Kind::kMaster};
  runtime_core::MonoTime at{};
  // kMaster
  canopen_master::MasterEventKind master_kind{};
  std::uint8_t node{};
  canopen_codec::NmtState state{};  // kState
  canopen_master::SdoResult sdo{};  // kSdo*; also kCmdDone of an SDO request
  canopen_master::EmcyDetail emcy{};
  std::uint64_t sync_index{};  // kSync, kPdo
  std::uint16_t cob{};         // kPdo, kPdoLengthError
  std::uint8_t dlc{};          // kPdoLengthError
  std::uint8_t expected{};     // kPdoLengthError
  std::uint8_t field_count{};  // kPdo: fields used
  bool fields_truncated{};     // kPdo: the PDO had more than kMaxEventPdoFields fields
  std::array<canopen_master::PdoValue, kMaxEventPdoFields> fields{};
  // kDispatch
  std::uint16_t wire_seq{};
  bool approved{};  // An approved command (else an arm hold).
  std::uint64_t source_seq{};
  // kCmdDone
  std::uint64_t cmd_id{};
  bool ok{};
  bool has_sdo{};
};

// A command for the master of a master-only worker, carried out at the next wake.
struct RawCommand {
  enum class Verb : std::uint8_t {
    kNmt,
    kSdoUpload,
    kSdoDownload,
    kSync,
    kHeartbeatConsumer,
    kRegisterPdo,
    kSend,  // One frame as given, straight to the transport.
  };
  std::uint64_t id{0};  // Reported back in kCmdDone.
  Verb verb{Verb::kNmt};
  std::uint8_t node{};
  canopen_codec::NmtCommand nmt{};
  std::uint16_t index{};
  std::uint8_t sub{};
  std::array<std::uint8_t, 4> bytes{};  // kSdoDownload: the first `size` bytes.
  std::uint8_t size{};
  canopen_master::SyncControl sync{};
  runtime_core::Duration consumer{};  // kHeartbeatConsumer; zero disables.
  std::uint32_t cob{};                // kRegisterPdo
  std::array<canopen_master::PdoField, kMaxEventPdoFields> fields{};  // kRegisterPdo
  std::uint8_t field_count{};
  canopen_codec::Frame frame{};  // kSend
};

struct WorkerStats {
  std::uint64_t wakes{};
  std::uint64_t timer_wakes{};
  std::uint64_t rx_frames{};
  std::uint64_t rx_budget_hits{};      // Wakes that read their whole budget (B1).
  std::uint64_t commands_taken{};      // Commands the worker took from the box.
  std::uint64_t command_overwrites{};  // Commands overwritten in the box (superseded).
  std::uint64_t events_forwarded{};
  std::uint64_t event_overflows{};      // Events lost to a full queue (kEventOverflow).
  std::uint64_t ops_refused{};          // Refused on the control side (queue full, master-only).
  std::uint64_t ops_cancelled{};        // Failed because a wait timed out.
  std::uint64_t late_completions{};     // Cancelled after they had completed (inhibit latched).
  std::uint64_t raw_refused{};          // Raw commands refused (queue full, backend mode).
  std::uint64_t cancel_ack_timeouts{};  // Timed-out waits whose cancel no wake took in time.
  TransportCounts transport{};
};

// The stats as JSON members ("key":value pairs, comma-separated, no braces), for the exit record
// of canopen_master_cli.
std::string stats_json(const WorkerStats& stats);

struct WorkerOptions {
  std::size_t rx_budget{kRxBudget};
  // Forward master events and dispatch records to the event queue. Off by default: a worker
  // with no reader would fill the queue, and a full queue is a fault. A reader turns it on.
  bool forward_events{false};
  // How long a timed-out wait() blocks for the worker to take its cancel (C10); past it the
  // inhibit is requested instead.
  runtime_core::Duration cancel_ack_timeout{3 * 2'000'000};
  // Test seam: called on the control thread when a post is refused for a full op queue, before
  // post() returns.
  // A logic-time test runs a wake from it, so an older op ends inside the refusal.
  std::function<void(std::uint64_t op_id)> on_post_refused{};
};

struct WorkerState;  // The channels and everything the thread owns (worker.cpp).

class Worker final : public hal_contract::Backend {
 public:
  // The backend on `transport`. Throws std::invalid_argument for a null transport; a refused
  // configuration is reported by every op, as by CanopenBackend.
  Worker(CanopenConfig config, std::unique_ptr<Transport> transport, WorkerOptions options = {});
  // A master-only worker. Throws what Master throws, and std::invalid_argument for a null
  // transport.
  Worker(canopen_master::MasterConfig config, std::unique_ptr<Transport> transport,
         WorkerOptions options = {});
  Worker(const Worker&) = delete;
  Worker& operator=(const Worker&) = delete;
  // shutdown(1 s); if the thread still runs, it is detached and keeps the state alive.
  ~Worker() override;

  // ---- thread lifecycle (control thread) ----
  // Creates the worker thread; false with `error` if it cannot (timerfd, thread), or if a thread
  // was already started on this Worker (a worker is never created while another exists).
  bool start(std::string* error);
  // Posts the shutdown, wakes the worker and joins within `timeout`. True once the thread has
  // ended (or none was started); false while it still runs (retryable).
  bool shutdown(runtime_core::Duration timeout);
  bool running() const;
  // Logic-time test seam: one wake at `now` on the calling thread. Throws std::logic_error once
  // start() succeeded.
  void run_once(runtime_core::MonoTime now);

  // ---- hal_contract::Backend (control thread) ----
  void configure(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  runtime_core::DeviceCapabilities capabilities() const override;
  void activate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  void arm(const hal_contract::OpTicket& ticket, const hal_contract::ArmRequest& request,
           runtime_core::MonoTime now) override;
  // Into the command box; never blocks.
  void submit(const hal_contract::ApprovedCommand& command, runtime_core::MonoTime now) override;
  // The newest published feedback, its superseded count including the box overwrites.
  hal_contract::Feedback poll(runtime_core::MonoTime now) override;
  void deactivate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  // Inhibit (reported by inhibited() at once) and shutdown, without waiting.
  void on_error_nonblocking() override;
  hal_contract::OpState op_state(std::uint64_t op_id) const override;
  // Blocks up to `timeout`; std::nullopt when the op was still pending, which fails it (C10).
  std::optional<hal_contract::OpResult> wait(std::uint64_t op_id,
                                             runtime_core::Duration timeout) override;
  bool inhibited() const override;
  bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const override;

  // ---- master-only and observation (control thread) ----
  // False (counted in raw_refused) on a backend worker or a full queue.
  bool post_raw(const RawCommand& command);
  std::optional<WorkerEvent> pop_event();
  WorkerStats stats() const;
  // Blocks until `predicate` holds, re-evaluating it after every wake the worker publishes (and
  // once at the start); false when `timeout` passed first. For tests and the CLI's steps.
  bool await(const std::function<bool()>& predicate, runtime_core::Duration timeout);
  bool master_only() const;

 private:
  struct Local;  // Control-thread state (refusals, cancels, the readers' buffers).
  void post(std::uint8_t kind, const hal_contract::OpTicket& ticket,
            const hal_contract::ArmRequest& request);
  void wake() const;

  std::shared_ptr<WorkerState> state_;
  std::unique_ptr<Local> local_;
};

}  // namespace canopen_backend
