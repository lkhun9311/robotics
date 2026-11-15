// SPDX-License-Identifier: Apache-2.0
#include "canopen_backend/worker.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <span>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace canopen_backend {

namespace cc = canopen_codec;
namespace cm = canopen_master;
namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

// A single-writer single-reader box holding the newest value (a triple buffer). Lock-free; the
// writer never waits for the reader.
template <class T>
class LatestBox {
 public:
  // Returns true when the previous value was never read (it is overwritten).
  bool write(const T& value) {
    slots_[back_] = value;
    const std::uint8_t prev =
        shared_.exchange(static_cast<std::uint8_t>(back_ | kFresh), std::memory_order_acq_rel);
    back_ = static_cast<std::uint8_t>(prev & kIndex);
    return (prev & kFresh) != 0U;
  }
  // The newest value written since the last read, if any.
  bool take(T& out) {
    if ((shared_.load(std::memory_order_relaxed) & kFresh) == 0U) {
      return false;
    }
    const std::uint8_t prev = shared_.exchange(front_, std::memory_order_acq_rel);
    front_ = static_cast<std::uint8_t>(prev & kIndex);
    out = slots_[front_];
    return true;
  }

 private:
  static constexpr std::uint8_t kFresh = 4;
  static constexpr std::uint8_t kIndex = 3;
  std::array<T, 3> slots_{};
  std::atomic<std::uint8_t> shared_{1};
  std::uint8_t back_{0};   // Writer's.
  std::uint8_t front_{2};  // Reader's.
};

// Bounded single-producer single-consumer queue.
template <class T, std::size_t N>
class Ring {
 public:
  bool push(const T& value) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail - head_.load(std::memory_order_acquire) == N) {
      return false;
    }
    slots_[tail % N] = value;
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }
  bool pop(T& out) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    out = slots_[head % N];
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

 private:
  std::array<T, N> slots_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

enum class OpKind : std::uint8_t { kConfigure, kActivate, kArm, kDeactivate };

struct PostedOp {
  OpKind kind{OpKind::kConfigure};
  hc::OpTicket ticket{};
  hc::ArmRequest request{};
};

// The op table: slot op_id % kOpTable, word = op_id << 2 | state. The control side writes a
// pending word when it posts; only the worker ends one (compare-exchange from pending).
constexpr std::size_t kOpTable = 16;
constexpr std::uint64_t kStatePending = 0;
constexpr std::uint64_t kStateSucceeded = 1;
constexpr std::uint64_t kStateFailed = 2;

struct OpSlot {
  std::atomic<std::uint64_t> word{0};
  std::atomic<const char*> error{nullptr};
};

constexpr std::int64_t kNoSync = std::numeric_limits<std::int64_t>::min();

constexpr const char* kWaitTimedOut = "the wait for the op timed out (C10)";
constexpr const char* kWorkerExited = "the worker exited before the op ended";
constexpr const char* kQueueFull = "the worker's op queue is full";
constexpr const char* kMasterOnly = "a master-only worker has no session";

WorkerEvent from_master(const cm::MasterEvent& e) {
  WorkerEvent w;
  w.kind = WorkerEvent::Kind::kMaster;
  w.at = e.at;
  w.master_kind = e.kind;
  w.node = e.node;
  if (const auto* d = std::get_if<cm::StateDetail>(&e.detail)) {
    w.state = d->state;
  } else if (const auto* d = std::get_if<cm::SdoResult>(&e.detail)) {
    w.sdo = *d;
  } else if (const auto* d = std::get_if<cm::EmcyDetail>(&e.detail)) {
    w.emcy = *d;
  } else if (const auto* d = std::get_if<cm::SyncDetail>(&e.detail)) {
    w.sync_index = d->sync_index;
  } else if (const auto* d = std::get_if<cm::PdoDetail>(&e.detail)) {
    w.cob = d->cob;
    w.sync_index = d->sync_index;
    const std::size_t n = std::min(d->fields.size(), kMaxEventPdoFields);
    for (std::size_t i = 0; i < n; ++i) {
      w.fields[i] = d->fields[i];
    }
    w.field_count = static_cast<std::uint8_t>(n);
    w.fields_truncated = d->fields.size() > kMaxEventPdoFields;
  } else if (const auto* d = std::get_if<cm::PdoLengthDetail>(&e.detail)) {
    w.cob = d->cob;
    w.dlc = d->dlc;
    w.expected = d->expected;
  }
  return w;
}

std::chrono::steady_clock::time_point deadline_after(rc::Duration timeout) {
  const std::int64_t ns = timeout.ns < 0 ? 0 : timeout.ns;
  return std::chrono::steady_clock::now() + std::chrono::nanoseconds(ns);
}

}  // namespace

// Everything the worker thread owns and every channel; held by the Worker and by the thread.
struct WorkerState {
  WorkerOptions options;
  // ---- owned by the worker thread (or run_once's caller) ----
  Transport* transport{nullptr};                    // Owned by backend, or by bare_transport.
  std::unique_ptr<CanopenBackend> backend;          // Backend mode.
  std::unique_ptr<Transport> bare_transport;        // Master-only mode.
  std::unique_ptr<cm::Master> master;               // Master-only mode.
  std::array<std::uint64_t, kOpTable> inflight{};   // Op ids posted to the backend, not ended.
  std::array<std::uint64_t, kOpTable> cancelled{};  // Op ids a wait gave up on (ring).
  std::size_t cancelled_next{0};
  std::vector<WorkerEvent> pending_done;  // kCmdDone of this wake, after its master events.
  std::uint16_t last_dispatch{0};
  bool overflowed{false};
  bool overflow_new{false};
  bool lost{false};
  bool shutting_down{false};
  rc::MonoTime shutdown_at{};
  bool finished{false};
  WorkerStats stats{};

  // ---- channels ----
  LatestBox<hc::ApprovedCommand> commands;
  Ring<PostedOp, kOpQueueCapacity> ops;
  Ring<std::uint64_t, kOpTable> cancels;
  Ring<RawCommand, kRawQueueCapacity> raw;
  std::atomic<bool> inhibit_request{false};
  std::atomic<bool> shutdown_request{false};
  LatestBox<hc::Feedback> feedback;
  LatestBox<rc::DeviceCapabilities> caps;
  LatestBox<WorkerStats> published_stats;
  std::array<OpSlot, kOpTable> table{};
  std::atomic<std::int64_t> last_sync_ns{kNoSync};
  std::atomic<bool> session_inhibited{false};
  Ring<WorkerEvent, kEventQueueCapacity> events;
  std::atomic<bool> exited{false};
  std::atomic<bool> fatal{false};  // The thread ended on an error of its own (ppoll, timerfd).
  int event_fd{-1};
  int timer_fd{-1};
  std::mutex mu;
  std::condition_variable cv;

  ~WorkerState() {
    if (event_fd >= 0) ::close(event_fd);
    if (timer_fd >= 0) ::close(timer_fd);
  }

  void forward(const WorkerEvent& e) {
    if (!options.forward_events) {
      return;
    }
    if (events.push(e)) {
      ++stats.events_forwarded;
    } else {
      ++stats.event_overflows;
      if (!overflowed) overflow_new = true;
      overflowed = true;
    }
  }

  void publish_end(std::uint64_t id, std::uint64_t state, const char* error) {
    OpSlot& slot = table[id % kOpTable];
    slot.error.store(error, std::memory_order_release);
    std::uint64_t expected = (id << 2U) | kStatePending;
    slot.word.compare_exchange_strong(expected, (id << 2U) | state, std::memory_order_acq_rel);
  }

  bool was_cancelled(std::uint64_t id) const {
    for (const std::uint64_t c : cancelled) {
      if (c == id) return true;
    }
    return false;
  }

  void post_op(const PostedOp& op, rc::MonoTime now) {
    if (op.kind == OpKind::kConfigure) {
      overflowed = false;  // A new configure starts a fresh fault picture.
    }
    switch (op.kind) {
      case OpKind::kConfigure:
        backend->configure(op.ticket, now);
        break;
      case OpKind::kActivate:
        backend->activate(op.ticket, now);
        break;
      case OpKind::kArm:
        backend->arm(op.ticket, op.request, now);
        break;
      case OpKind::kDeactivate:
        backend->deactivate(op.ticket, now);
        break;
    }
    for (std::uint64_t& slot : inflight) {
      if (slot == 0) {
        slot = op.ticket.op_id;
        return;
      }
    }
    // More ops in flight than the table holds: the oldest is forgotten (the Hal keeps 8).
    inflight[op.ticket.op_id % kOpTable] = op.ticket.op_id;
  }

  void cancel(std::uint64_t id, rc::MonoTime now) {
    cancelled[cancelled_next] = id;
    cancelled_next = (cancelled_next + 1) % kOpTable;
    ++stats.ops_cancelled;
    for (const std::uint64_t f : inflight) {
      if (f == id) {
        backend->cancel(id, kWaitTimedOut, now);
        return;
      }
    }
    // Already ended and published: if it succeeded, the caller was told otherwise.
    const std::uint64_t word = table[id % kOpTable].word.load(std::memory_order_acquire);
    if (word == ((id << 2U) | kStateSucceeded)) {
      ++stats.late_completions;
      backend->on_error_nonblocking();
    }
  }

  void apply_raw(const RawCommand& c, rc::MonoTime now) {
    WorkerEvent done;
    done.kind = WorkerEvent::Kind::kCmdDone;
    done.at = now;
    done.cmd_id = c.id;
    done.node = c.node;
    const std::optional<cc::NodeId> node = cc::NodeId::make(c.node);
    const auto sdo_done = [this, id = c.id](const cm::SdoResult& r) {
      WorkerEvent e;
      e.kind = WorkerEvent::Kind::kCmdDone;
      e.cmd_id = id;
      e.node = r.node;
      e.ok = true;  // Carried out; the outcome is in the SDO result.
      e.has_sdo = true;
      e.sdo = r;
      pending_done.push_back(e);
    };
    switch (c.verb) {
      case RawCommand::Verb::kNmt:
        done.ok = node && master->nmt(c.nmt, *node);
        break;
      case RawCommand::Verb::kSdoUpload:
        if (node && master->sdo_upload(*node, c.index, c.sub, sdo_done)) return;
        break;
      case RawCommand::Verb::kSdoDownload:
        if (node && c.size >= 1 && c.size <= 4 &&
            master->sdo_download(*node, c.index, c.sub,
                                 std::span<const std::uint8_t>(c.bytes.data(), c.size), sdo_done)) {
          return;
        }
        break;
      case RawCommand::Verb::kSync:
        done.ok = master->sync_control(c.sync);
        break;
      case RawCommand::Verb::kHeartbeatConsumer:
        done.ok = node && master->set_heartbeat_consumer(*node, c.consumer);
        break;
      case RawCommand::Verb::kRegisterPdo: {
        cm::PdoLayout layout;
        for (std::size_t i = 0; i < c.field_count && i < kMaxEventPdoFields; ++i) {
          layout.fields.push_back(c.fields[i]);
        }
        done.ok = node && master->register_rx_pdo(*node, c.cob, std::move(layout));
        break;
      }
      case RawCommand::Verb::kSend:
        transport->send(c.frame);
        done.ok = true;
        break;
    }
    pending_done.push_back(done);
  }

  void send_all(const std::vector<cc::Frame>& frames) {
    for (const cc::Frame& f : frames) transport->send(f);
  }

  // Master-only step: the CanopenBackend::step order on a bare master, events drained after.
  std::size_t step_master(rc::MonoTime now) {
    std::size_t handled = 0;
    const auto drain = [&] {
      while (handled < options.rx_budget) {
        const std::vector<cc::Frame> received = transport->poll();
        if (received.empty()) return;
        handled += received.size();
        for (const cc::Frame& f : received) send_all(master->on_frame(f, now));
      }
    };
    drain();
    send_all(master->tick(now));
    drain();
    for (const cm::MasterEvent& e : master->drain_events()) forward(from_master(e));
    return handled;
  }

  bool has_session() const { return backend && backend->config_error().empty(); }

  void forward_dispatches(rc::MonoTime now) {
    if (!has_session() || !options.forward_events) return;
    const cm::VdriveSession& session = backend->session();
    for (;;) {
      const auto next = static_cast<std::uint16_t>(last_dispatch + 1U);
      const std::optional<cm::VdriveSession::Dispatch> d = session.dispatched(next);
      if (!d) return;
      WorkerEvent e;
      e.kind = WorkerEvent::Kind::kDispatch;
      e.at = now;
      e.wire_seq = next;
      e.approved = d->approved;
      e.source_seq = d->source_seq;
      forward(e);
      last_dispatch = next;
    }
  }

  void publish_ops() {
    for (std::uint64_t& id : inflight) {
      if (id == 0) continue;
      const hc::OpState st = backend->op_state(id);
      if (st == hc::OpState::kPending) continue;
      if (st == hc::OpState::kSucceeded) {
        publish_end(id, kStateSucceeded, nullptr);
        if (was_cancelled(id)) {
          ++stats.late_completions;  // Never happens through cancel(); kept as a backstop.
          backend->on_error_nonblocking();
        }
      } else {
        const char* error = backend->op_error(id);
        publish_end(id, kStateFailed, error != nullptr ? error : "failed");
      }
      id = 0;
    }
  }

  bool stops_pending(const hc::Feedback& fb) const {
    for (const hc::StopStatus s : fb.stop) {
      if (s == hc::StopStatus::kStopping) return true;
    }
    return false;
  }

  bool needs_group_stop() const {
    if (!has_session() || backend->inhibited()) return false;
    switch (backend->session().phase()) {
      case cm::SessionPhase::kArming:
      case cm::SessionPhase::kArmed:
      case cm::SessionPhase::kDeactivating:
        return true;
      default:
        return false;
    }
  }

  void notify() {
    { const std::lock_guard<std::mutex> lock(mu); }
    cv.notify_all();
  }

  // One wake of the worker (see the header).
  void wake(rc::MonoTime now) {
    if (finished) return;
    ++stats.wakes;
    if (inhibit_request.exchange(false, std::memory_order_acq_rel) && backend) {
      backend->on_error_nonblocking();
    }
    if (shutdown_request.load(std::memory_order_acquire) && !shutting_down) {
      shutting_down = true;
      shutdown_at = now;
      if (needs_group_stop()) backend->on_error_nonblocking();
    }
    PostedOp op;
    while (ops.pop(op)) {
      if (backend) post_op(op, now);
    }
    std::uint64_t cancel_id = 0;
    while (cancels.pop(cancel_id)) {
      if (backend) cancel(cancel_id, now);
    }
    RawCommand rc_cmd;
    while (raw.pop(rc_cmd)) {
      if (master) apply_raw(rc_cmd, now);
    }
    hc::ApprovedCommand command;
    if (commands.take(command)) {
      ++stats.commands_taken;
      if (backend) backend->submit(command, now);
    }

    std::size_t handled = 0;
    if (backend) {
      handled = backend->step(now, options.rx_budget);
      if (overflow_new) {
        overflow_new = false;
        backend->on_error_nonblocking();  // C10: a full event queue is a fault.
      }
    } else {
      handled = step_master(now);
      overflow_new = false;
    }
    for (const WorkerEvent& e : pending_done) forward(e);
    pending_done.clear();
    stats.rx_frames += handled;
    if (handled >= options.rx_budget) ++stats.rx_budget_hits;

    stats.transport = transport->counts();
    if (!lost && stats.transport.rx_error > 0) {
      lost = true;  // The socket reported an error: the bus is gone for us.
      if (backend) backend->on_error_nonblocking();
    }

    forward_dispatches(now);
    hc::Feedback fb{};
    if (backend) {
      publish_ops();
      fb = backend->poll(now);
      caps.write(backend->capabilities());
      const std::optional<rc::MonoTime> sync = backend->last_sync();
      last_sync_ns.store(sync ? sync->ns : kNoSync, std::memory_order_release);
      session_inhibited.store(backend->inhibited(), std::memory_order_release);
    }
    if (overflowed) fb.faults.set(hc::Fault::kEventOverflow);
    if (lost) fb.faults.set(hc::Fault::kTransportLost);
    feedback.write(fb);

    if (shutting_down && (!stops_pending(fb) || now - shutdown_at >= kShutdownStopLimit)) {
      finish();
    }
    published_stats.write(stats);
    notify();
  }

  // Ends the worker: ops still in flight fail, the transport is released.
  void finish() {
    finished = true;
    for (std::uint64_t& id : inflight) {
      if (id != 0) publish_end(id, kStateFailed, kWorkerExited);
      id = 0;
    }
    stats.transport = transport->counts();
    transport->release();
  }

  // The thread body.
  static void run(const std::shared_ptr<WorkerState>& s) {
    const rc::SystemMonotonicClock clock;
    bool timer_seen = false;
    while (!s->finished) {
      // Until the first timer expiry only the timer and the wake: the first step, and with it
      // the SYNC grid, lands on the timer grid. A transport that failed is no longer polled
      // (its pending error would make ppoll return at once forever); the timer still wakes.
      const int bus_fd = timer_seen && !s->lost ? s->transport->wait_fd() : -1;
      std::array<pollfd, 3> fds{
          {{s->timer_fd, POLLIN, 0}, {s->event_fd, POLLIN, 0}, {bus_fd, POLLIN, 0}}};
      if (::ppoll(fds.data(), fds.size(), nullptr, nullptr) < 0) {
        if (errno == EINTR) continue;
        s->fatal.store(true, std::memory_order_release);
        s->finish();
        break;
      }
      std::uint64_t count = 0;
      if ((fds[0].revents & POLLIN) != 0) {
        if (::read(s->timer_fd, &count, sizeof count) == sizeof count) {
          timer_seen = true;
          ++s->stats.timer_wakes;
        }
      }
      if ((fds[1].revents & POLLIN) != 0) {
        [[maybe_unused]] const ssize_t n = ::read(s->event_fd, &count, sizeof count);
      }
      if (!timer_seen) continue;
      s->wake(clock.now());
    }
    s->published_stats.write(s->stats);
    s->exited.store(true, std::memory_order_release);
    s->notify();
  }
};

// Control-thread state.
struct Worker::Local {
  struct Refusal {
    std::uint64_t op_id{0};
    const char* error{nullptr};
  };
  std::array<Refusal, kOpTable> refusals{};  // Refused here, or cancelled by a timed-out wait.
  std::size_t refusals_next{0};
  bool master_only{false};
  rc::Duration bus_period{};
  bool has_session{false};
  bool local_inhibit{false};
  std::uint64_t command_overwrites{0};
  std::uint64_t ops_refused{0};
  std::uint64_t raw_refused{0};
  hc::Feedback feedback{};
  rc::DeviceCapabilities caps{};
  WorkerStats stats{};
  std::thread thread;
  bool started{false};

  void refuse(std::uint64_t op_id, const char* error) {
    refusals[refusals_next] = Refusal{op_id, error};
    refusals_next = (refusals_next + 1) % refusals.size();
  }
  const Refusal* refusal(std::uint64_t op_id) const {
    for (const Refusal& r : refusals) {
      if (r.op_id != 0 && r.op_id == op_id) return &r;
    }
    return nullptr;
  }
};

namespace {

std::unique_ptr<Transport> require(std::unique_ptr<Transport> transport) {
  if (!transport) throw std::invalid_argument("canopen_backend::Worker: a transport is required");
  return transport;
}

int make_eventfd() {
  const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error(std::string("canopen_backend::Worker: eventfd: ") +
                             std::strerror(errno));
  }
  return fd;
}

}  // namespace

Worker::Worker(CanopenConfig config, std::unique_ptr<Transport> transport, WorkerOptions options)
    : state_(std::make_shared<WorkerState>()), local_(std::make_unique<Local>()) {
  std::unique_ptr<Transport> t = require(std::move(transport));
  state_->options = options;
  state_->transport = t.get();
  state_->backend = std::make_unique<CanopenBackend>(config, std::move(t));
  state_->event_fd = make_eventfd();
  local_->bus_period = config.session.bus_period;
  local_->has_session = state_->backend->config_error().empty();
  WorkerState* s = state_.get();
  state_->backend->set_event_observer(
      [s](const cm::MasterEvent& e) { s->forward(from_master(e)); });
}

Worker::Worker(cm::MasterConfig config, std::unique_ptr<Transport> transport, WorkerOptions options)
    : state_(std::make_shared<WorkerState>()), local_(std::make_unique<Local>()) {
  state_->bare_transport = require(std::move(transport));
  state_->options = options;
  state_->transport = state_->bare_transport.get();
  state_->master = std::make_unique<cm::Master>(std::move(config));
  state_->event_fd = make_eventfd();
  local_->master_only = true;
}

Worker::~Worker() {
  if (!shutdown(rc::milliseconds(1000)) && local_->thread.joinable()) {
    // C13: the thread holds the state through its own shared_ptr until it exits.
    local_->thread.detach();
  }
}

bool Worker::master_only() const { return local_->master_only; }

void Worker::wake() const {
  const std::uint64_t one = 1;
  [[maybe_unused]] const ssize_t n = ::write(state_->event_fd, &one, sizeof one);
}

bool Worker::start(std::string* error) {
  const auto fail = [&](const std::string& why) {
    if (error != nullptr) *error = why;
    return false;
  };
  if (local_->started) return fail("a worker thread was already started on this backend");
  const int tfd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  if (tfd < 0) return fail(std::string("timerfd_create: ") + std::strerror(errno));
  timespec now{};
  ::clock_gettime(CLOCK_MONOTONIC, &now);
  itimerspec spec{};
  spec.it_interval.tv_nsec = kWakePeriod.ns;
  spec.it_value = now;
  spec.it_value.tv_nsec += kWakePeriod.ns;
  if (spec.it_value.tv_nsec >= 1'000'000'000) {
    spec.it_value.tv_sec += 1;
    spec.it_value.tv_nsec -= 1'000'000'000;
  }
  if (::timerfd_settime(tfd, TFD_TIMER_ABSTIME, &spec, nullptr) < 0) {
    const std::string why = std::string("timerfd_settime: ") + std::strerror(errno);
    ::close(tfd);
    return fail(why);
  }
  state_->timer_fd = tfd;
  try {
    local_->thread = std::thread([s = state_] { WorkerState::run(s); });
  } catch (const std::system_error& e) {
    return fail(std::string("thread: ") + e.what());
  }
  local_->started = true;
  return true;
}

bool Worker::running() const {
  return local_->started && !state_->exited.load(std::memory_order_acquire);
}

bool Worker::shutdown(rc::Duration timeout) {
  if (!local_->started) return true;
  if (!local_->thread.joinable()) return true;  // Joined before.
  state_->shutdown_request.store(true, std::memory_order_release);
  wake();
  {
    std::unique_lock<std::mutex> lock(state_->mu);
    if (!state_->cv.wait_until(lock, deadline_after(timeout),
                               [&] { return state_->exited.load(std::memory_order_acquire); })) {
      return false;
    }
  }
  local_->thread.join();
  return true;
}

void Worker::run_once(rc::MonoTime now) {
  if (local_->started) {
    throw std::logic_error("canopen_backend::Worker::run_once after start()");
  }
  state_->wake(now);
}

// ---- ops ----

void Worker::post(std::uint8_t kind, const hc::OpTicket& ticket, const hc::ArmRequest& request) {
  if (local_->master_only) {
    ++local_->ops_refused;
    local_->refuse(ticket.op_id, kMasterOnly);
    return;
  }
  OpSlot& slot = state_->table[ticket.op_id % kOpTable];
  const std::uint64_t before = slot.word.load(std::memory_order_acquire);
  slot.word.store((ticket.op_id << 2U) | kStatePending, std::memory_order_release);
  if (!state_->ops.push(PostedOp{static_cast<OpKind>(kind), ticket, request})) {
    slot.word.store(before, std::memory_order_release);
    ++local_->ops_refused;
    local_->refuse(ticket.op_id, kQueueFull);
    return;
  }
  wake();
}

void Worker::configure(const hc::OpTicket& ticket, rc::MonoTime /*now*/) {
  post(static_cast<std::uint8_t>(OpKind::kConfigure), ticket, {});
}
void Worker::activate(const hc::OpTicket& ticket, rc::MonoTime /*now*/) {
  post(static_cast<std::uint8_t>(OpKind::kActivate), ticket, {});
}
void Worker::arm(const hc::OpTicket& ticket, const hc::ArmRequest& request, rc::MonoTime /*now*/) {
  post(static_cast<std::uint8_t>(OpKind::kArm), ticket, request);
}
void Worker::deactivate(const hc::OpTicket& ticket, rc::MonoTime /*now*/) {
  post(static_cast<std::uint8_t>(OpKind::kDeactivate), ticket, {});
}

hc::OpState Worker::op_state(std::uint64_t op_id) const {
  if (local_->refusal(op_id) != nullptr) return hc::OpState::kFailed;
  const std::uint64_t word = state_->table[op_id % kOpTable].word.load(std::memory_order_acquire);
  if (word == 0 || (word >> 2U) != op_id) return hc::OpState::kUnknown;
  switch (word & 3U) {
    case kStatePending:
      return hc::OpState::kPending;
    case kStateSucceeded:
      return hc::OpState::kSucceeded;
    default:
      return hc::OpState::kFailed;
  }
}

std::optional<hc::OpResult> Worker::wait(std::uint64_t op_id, rc::Duration timeout) {
  if (const Local::Refusal* r = local_->refusal(op_id)) return hc::OpResult{false, r->error, {}};
  {
    std::unique_lock<std::mutex> lock(state_->mu);
    state_->cv.wait_until(lock, deadline_after(timeout), [&] {
      return op_state(op_id) != hc::OpState::kPending ||
             state_->exited.load(std::memory_order_acquire);
    });
  }
  OpSlot& slot = state_->table[op_id % kOpTable];
  switch (op_state(op_id)) {
    case hc::OpState::kSucceeded:
      return hc::OpResult{true, "", {}};
    case hc::OpState::kFailed: {
      const char* error = slot.error.load(std::memory_order_acquire);
      return hc::OpResult{false, error != nullptr ? error : "failed", {}};
    }
    case hc::OpState::kUnknown:
      return hc::OpResult{false, "unknown operation", {}};
    case hc::OpState::kPending:
      break;
  }
  if (state_->exited.load(std::memory_order_acquire)) {
    local_->refuse(op_id, kWorkerExited);
    return hc::OpResult{false, kWorkerExited, {}};
  }
  // C10: the wait timed out, so the op fails: here at once, in the session at the next wake.
  local_->refuse(op_id, kWaitTimedOut);
  if (!state_->cancels.push(op_id)) {
    state_->inhibit_request.store(true, std::memory_order_release);  // Fail safe.
  }
  wake();
  return std::nullopt;
}

// ---- control path ----

void Worker::submit(const hc::ApprovedCommand& command, rc::MonoTime /*now*/) {
  if (state_->commands.write(command)) ++local_->command_overwrites;
}

hc::Feedback Worker::poll(rc::MonoTime /*now*/) {
  state_->feedback.take(local_->feedback);
  hc::Feedback fb = local_->feedback;
  fb.counters.superseded += local_->command_overwrites;
  if (state_->fatal.load(std::memory_order_acquire)) fb.faults.set(hc::Fault::kTransportLost);
  if (inhibited()) fb.faults.set(hc::Fault::kInhibited);
  return fb;
}

rc::DeviceCapabilities Worker::capabilities() const {
  state_->caps.take(local_->caps);
  return local_->caps;
}

void Worker::on_error_nonblocking() {
  local_->local_inhibit = true;
  state_->inhibit_request.store(true, std::memory_order_release);
  state_->shutdown_request.store(true, std::memory_order_release);
  wake();
}

bool Worker::inhibited() const {
  return local_->master_only || !local_->has_session || local_->local_inhibit ||
         state_->session_inhibited.load(std::memory_order_acquire);
}

bool Worker::can_apply(rc::MonoTime valid_until, rc::MonoTime now) const {
  if (!local_->has_session) return false;
  const std::int64_t ns = state_->last_sync_ns.load(std::memory_order_acquire);
  const std::optional<rc::MonoTime> last =
      ns == kNoSync ? std::nullopt : std::optional<rc::MonoTime>(rc::MonoTime{ns});
  return cm::command_can_apply(last, local_->bus_period, valid_until, now);
}

// ---- master-only and observation ----

bool Worker::post_raw(const RawCommand& command) {
  if (!local_->master_only || !state_->raw.push(command)) {
    ++local_->raw_refused;
    return false;
  }
  wake();
  return true;
}

std::optional<WorkerEvent> Worker::pop_event() {
  WorkerEvent e;
  if (state_->events.pop(e)) return e;
  return std::nullopt;
}

WorkerStats Worker::stats() const {
  state_->published_stats.take(local_->stats);
  WorkerStats s = local_->stats;
  s.command_overwrites = local_->command_overwrites;
  s.ops_refused = local_->ops_refused;
  s.raw_refused = local_->raw_refused;
  return s;
}

bool Worker::await(const std::function<bool()>& predicate, rc::Duration timeout) {
  std::unique_lock<std::mutex> lock(state_->mu);
  return state_->cv.wait_until(lock, deadline_after(timeout), predicate);
}

std::string stats_json(const WorkerStats& s) {
  std::string out;
  const auto add = [&](const char* key, std::uint64_t value) {
    if (!out.empty()) out += ',';
    out += '"';
    out += key;
    out += "\":";
    out += std::to_string(value);
  };
  add("wakes", s.wakes);
  add("timer_wakes", s.timer_wakes);
  add("rx_frames", s.rx_frames);
  add("rx_budget_hits", s.rx_budget_hits);
  add("commands_taken", s.commands_taken);
  add("command_overwrites", s.command_overwrites);
  add("events_forwarded", s.events_forwarded);
  add("event_overflows", s.event_overflows);
  add("ops_refused", s.ops_refused);
  add("ops_cancelled", s.ops_cancelled);
  add("late_completions", s.late_completions);
  add("raw_refused", s.raw_refused);
  add("tx", s.transport.tx);
  add("tx_eagain", s.transport.tx_eagain);
  add("tx_enobufs", s.transport.tx_enobufs);
  add("tx_other_error", s.transport.tx_other_error);
  add("rx", s.transport.rx);
  add("rx_rejected", s.transport.rx_rejected);
  add("rx_error", s.transport.rx_error);
  return out;
}

}  // namespace canopen_backend
