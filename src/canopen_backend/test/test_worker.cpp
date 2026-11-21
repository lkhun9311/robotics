// SPDX-License-Identifier: Apache-2.0
// The I/O worker (C10): the boxes and queues between the control and worker threads, the rx
// budget per wake (B1), op deadlines and cancellation, and the socket's write errors.
// The logic-time cases call run_once on the test thread; the others run the real worker thread and
// synchronize through Worker::await and the fake bus's condition variable, never by sleeping.
#include <gtest/gtest.h>
#include <linux/can.h>
#include <sys/socket.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "canopen_backend/canopen_backend.hpp"
#include "canopen_backend/socket_transport.hpp"
#include "canopen_backend/worker.hpp"
#include "canopen_codec/codec.hpp"
#include "hal_contract/faults.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/time.hpp"
#include "socketcan_io/socket.hpp"

namespace {

namespace cb = canopen_backend;
namespace cc = canopen_codec;
namespace hc = hal_contract;
namespace rc = runtime_core;

constexpr rc::MonoTime kT0{1'000'000'000'000};
constexpr rc::Duration kW{2'000'000};

struct Sent {
  rc::MonoTime at{};
  cc::Frame frame{};
};

// A CAN bus behind the fake system calls of socketcan_io: frames the test delivers are read by
// the worker; frames the worker writes are recorded.
// Safe to use from both threads.
class FakeBus {
 public:
  explicit FakeBus(std::function<rc::MonoTime()> clock) : clock_(std::move(clock)) {}

  socketcan_io::SysOps ops() {
    socketcan_io::SysOps o;
    o.recvmsg = [this](int, msghdr* msg, int) -> ssize_t {
      std::lock_guard<std::mutex> lock(mu_);
      if (rx_.empty()) {
        errno = EAGAIN;
        return -1;
      }
      const cc::Frame f = rx_.front();
      rx_.pop_front();
      canfd_frame out{};
      out.can_id = f.cob_id;
      out.len = f.dlc;
      std::memcpy(out.data, f.data.data(), f.dlc);
      std::memcpy(msg->msg_iov[0].iov_base, &out, CAN_MTU);
      msg->msg_controllen = 0;
      return static_cast<ssize_t>(CAN_MTU);
    };
    o.write = [this](int, const void* data, std::size_t size) -> ssize_t {
      can_frame in{};
      std::memcpy(&in, data, sizeof in);
      cc::Frame f;
      f.cob_id = static_cast<std::uint16_t>(in.can_id);
      f.dlc = in.len;
      std::memcpy(f.data.data(), in.data, in.len);
      std::unique_lock<std::mutex> lock(mu_);
      if (gate_ && gate_(f)) {
        gate_ = nullptr;  // Only the first matching write waits.
        blocked_ = true;
        cv_.notify_all();
        cv_.wait(lock, [&] { return gate_open_; });
      }
      if (fail_next_ > 0) {
        --fail_next_;
        errno = fail_errno_.front();
        fail_errno_.pop_front();
        return -1;
      }
      tx_.push_back(Sent{clock_(), f});
      cv_.notify_all();
      return static_cast<ssize_t>(size);
    };
    o.now = clock_;
    return o;
  }

  std::unique_ptr<cb::Transport> transport() {
    return std::make_unique<cb::SocketTransport>(socketcan_io::CanSocket(-1, ops()));
  }

  void deliver(const cc::Frame& f) {
    std::lock_guard<std::mutex> lock(mu_);
    rx_.push_back(f);
  }
  bool rx_empty() {
    std::lock_guard<std::mutex> lock(mu_);
    return rx_.empty();
  }
  void fail_writes(std::initializer_list<int> errnos) {
    std::lock_guard<std::mutex> lock(mu_);
    for (int e : errnos) fail_errno_.push_back(e);
    fail_next_ = fail_errno_.size();
  }
  // The first write matching `pred` blocks until open_gate(): a wake held inside its step.
  void close_gate(std::function<bool(const cc::Frame&)> pred) {
    std::lock_guard<std::mutex> lock(mu_);
    gate_ = std::move(pred);
    gate_open_ = false;
  }
  bool wait_blocked(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock, timeout, [&] { return blocked_; });
  }
  void open_gate() {
    std::lock_guard<std::mutex> lock(mu_);
    gate_open_ = true;
    cv_.notify_all();
  }
  std::vector<Sent> sent() {
    std::lock_guard<std::mutex> lock(mu_);
    return tx_;
  }
  // Waits until at least `count` written frames satisfy `pred`.
  bool wait_sent(const std::function<bool(const cc::Frame&)>& pred, std::size_t count,
                 std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock, timeout, [&] {
      std::size_t n = 0;
      for (const Sent& s : tx_) n += pred(s.frame) ? 1U : 0U;
      return n >= count;
    });
  }

 private:
  std::function<rc::MonoTime()> clock_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<cc::Frame> rx_;
  std::vector<Sent> tx_;
  std::deque<int> fail_errno_;
  std::size_t fail_next_{0};
  std::function<bool(const cc::Frame&)> gate_;
  bool gate_open_{true};
  bool blocked_{false};
};

rc::MonoTime system_now() { return rc::SystemMonotonicClock{}.now(); }

cb::CanopenConfig config() {
  cb::CanopenConfig c;
  c.session.max_feedback_age = rc::milliseconds(30);
  return c;
}

bool is_sdo_request(const cc::Frame& f) { return f.cob_id > 0x600 && f.cob_id <= 0x67F; }

cc::Frame heartbeat(std::uint8_t node, std::uint8_t state) {
  cc::Frame f;
  f.cob_id = static_cast<std::uint16_t>(0x700 + node);
  f.dlc = 1;
  f.data[0] = state;
  return f;
}

// The expedited upload response of `node` for the request `req` (same index and sub).
cc::Frame upload_answer(const cc::Frame& req, std::uint32_t value) {
  cc::Frame f;
  f.cob_id = static_cast<std::uint16_t>(req.cob_id - 0x600 + 0x580);
  f.dlc = 8;
  f.data = {0x43,
            req.data[1],
            req.data[2],
            req.data[3],
            static_cast<std::uint8_t>(value),
            static_cast<std::uint8_t>(value >> 8U),
            static_cast<std::uint8_t>(value >> 16U),
            static_cast<std::uint8_t>(value >> 24U)};
  return f;
}

hc::ApprovedCommand command(std::uint64_t seq) {
  hc::ApprovedCommand c;
  c.source_seq = seq;
  return c;
}

}  // namespace

TEST(Worker, CommandBoxOverwritesAreCounted) {
  // Logic time: five submissions between two wakes leave one command and four overwrites, and
  // the overwrites are reported as superseded.
  {
    rc::MonoTime now = kT0;
    FakeBus bus([&] { return now; });
    cb::Worker w(config(), bus.transport());
    for (std::uint64_t k = 1; k <= 5; ++k) w.submit(command(k), now);
    w.run_once(now);
    EXPECT_EQ(w.stats().commands_taken, 1U);
    EXPECT_EQ(w.stats().command_overwrites, 4U);
    EXPECT_EQ(w.poll(now).counters.superseded, 4U);
    now = now + kW;
    w.submit(command(6), now);
    w.run_once(now);
    EXPECT_EQ(w.stats().commands_taken, 2U);
    EXPECT_EQ(w.stats().command_overwrites, 4U) << "a command taken in time is no overwrite";
  }
  // The real thread: every submission is either taken by the worker or counted as overwritten,
  // whatever the interleaving.
  FakeBus bus(system_now);
  cb::Worker w(config(), bus.transport(), cb::WorkerOptions{cb::kRxBudget, false});
  std::string error;
  ASSERT_TRUE(w.start(&error)) << error;
  constexpr std::uint64_t kSubmissions = 5000;
  for (std::uint64_t k = 1; k <= kSubmissions; ++k) w.submit(command(k), system_now());
  ASSERT_TRUE(w.await(
      [&] {
        const cb::WorkerStats s = w.stats();
        return s.commands_taken + s.command_overwrites == kSubmissions;
      },
      rc::milliseconds(2000)))
      << "taken " << w.stats().commands_taken << " + overwritten " << w.stats().command_overwrites
      << " != " << kSubmissions;
  const cb::WorkerStats s = w.stats();
  EXPECT_GT(s.commands_taken, 0U);
  EXPECT_GT(s.command_overwrites, 0U) << "INCONCLUSIVE: no submission was overwritten";
  EXPECT_EQ(w.poll(system_now()).counters.superseded, s.command_overwrites);
  EXPECT_TRUE(w.shutdown(rc::milliseconds(1000)));
}

TEST(Worker, EventQueueOverflowIsAFault) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });
  cb::Worker w(config(), bus.transport(), cb::WorkerOptions{cb::kRxBudget, true});
  // A configure that nobody answers keeps the session out of idle (where nothing latches).
  w.configure(hc::OpTicket{1, now + hc::kConfigureTimeout}, now);
  w.run_once(now);
  // Node 1's heartbeat alternating Operational and Pre-operational: one state event per frame.
  // A configuring session ignores the state of a node it has not commanded, so nothing but the
  // overflow can latch.
  constexpr int kFrames = 400;
  for (int k = 0; k < kFrames; ++k) bus.deliver(heartbeat(1, k % 2 == 0 ? 0x05 : 0x7F));
  now = now + kW;
  w.run_once(now);
  ASSERT_LE(w.stats().events_forwarded, cb::kEventQueueCapacity);
  ASSERT_EQ(w.stats().event_overflows, 0U) << "INCONCLUSIVE: overflow in the first wake";
  ASSERT_FALSE(w.inhibited()) << "INCONCLUSIVE: inhibited before any overflow";
  ASSERT_FALSE(w.poll(now).faults.has(hc::Fault::kEventOverflow));
  std::optional<rc::MonoTime> first_overflow;
  while (!bus.rx_empty()) {
    now = now + kW;
    w.run_once(now);
    if (!first_overflow && w.stats().event_overflows > 0) first_overflow = now;
  }
  ASSERT_TRUE(first_overflow.has_value()) << "INCONCLUSIVE: the queue never filled";
  const cb::WorkerStats s = w.stats();
  EXPECT_EQ(s.events_forwarded, cb::kEventQueueCapacity);
  EXPECT_GT(s.event_overflows, 0U);
  EXPECT_TRUE(w.poll(now).faults.has(hc::Fault::kEventOverflow));
  EXPECT_TRUE(w.inhibited()) << "a full event queue must latch the inhibit";
  // The queue kept the oldest events, in order, and exactly its capacity.
  std::size_t popped = 0;
  rc::MonoTime last{};
  while (const std::optional<cb::WorkerEvent> e = w.pop_event()) {
    EXPECT_GE(e->at, last);
    last = e->at;
    ++popped;
  }
  EXPECT_EQ(popped, cb::kEventQueueCapacity);
  EXPECT_LE(last, *first_overflow) << "a newer event displaced an older one";
  // The fault is the worker's own: a new configure does not clear it.
  now = now + kW;
  w.configure(hc::OpTicket{2, now + hc::kConfigureTimeout}, now);
  w.run_once(now);
  EXPECT_TRUE(w.poll(now).faults.has(hc::Fault::kEventOverflow));
  EXPECT_TRUE(w.inhibited());
}

TEST(Worker, RxBudgetPerWake) {
  // The socket is always readable and every read costs 10 us of logic time.
  // Bounded at kRxBudget frames per wake, the worker still produces SYNC every T_bus +- 1 ms.
  constexpr rc::Duration kReadCost{10'000};
  rc::MonoTime clock = kT0;
  socketcan_io::SysOps ops;
  std::vector<Sent> syncs;
  std::uint64_t reads = 0;
  ops.recvmsg = [&](int, msghdr* msg, int) -> ssize_t {
    clock = clock + kReadCost;
    ++reads;
    canfd_frame out{};
    out.can_id = 0x555;  // A COB-ID the master does not consume.
    out.len = 1;
    std::memcpy(msg->msg_iov[0].iov_base, &out, CAN_MTU);
    msg->msg_controllen = 0;
    return static_cast<ssize_t>(CAN_MTU);
  };
  ops.write = [&](int, const void* data, std::size_t size) -> ssize_t {
    can_frame in{};
    std::memcpy(&in, data, sizeof in);
    if (in.can_id == 0x080) syncs.push_back(Sent{clock, {}});
    return static_cast<ssize_t>(size);
  };
  ops.now = [&] { return clock; };
  cb::Worker w(config(), std::make_unique<cb::SocketTransport>(socketcan_io::CanSocket(-1, ops)),
               cb::WorkerOptions{cb::kRxBudget, false});
  const rc::MonoTime end = kT0 + rc::milliseconds(300);
  std::uint64_t wakes = 0;
  while (clock < end) {
    const std::uint64_t before = reads;
    w.run_once(clock);  // ppoll returns at once: the socket stays readable.
    ++wakes;
    ASSERT_LE(reads - before, cb::kRxBudget + 1) << "a wake read past its budget";
  }
  ASSERT_GE(syncs.size(), 25U) << "INCONCLUSIVE: too few SYNCs to judge";
  for (std::size_t k = 1; k < syncs.size(); ++k) {
    const std::int64_t gap = (syncs[k].at - syncs[k - 1].at).ns;
    EXPECT_GE(gap, 9'000'000) << "SYNC " << k;
    EXPECT_LE(gap, 11'000'000) << "SYNC " << k;
  }
  EXPECT_EQ(w.stats().rx_budget_hits, wakes);
}

TEST(Worker, LifecycleWaitTimesOut) {
  FakeBus bus(system_now);  // Nobody answers.
  cb::Worker w(config(), bus.transport(), cb::WorkerOptions{cb::kRxBudget, false});
  std::string error;
  ASSERT_TRUE(w.start(&error)) << error;
  // A wait shorter than the op's deadline gives up, and the op fails with it (C10).
  w.configure(hc::OpTicket{1, system_now() + hc::kConfigureTimeout}, system_now());
  const auto t0 = std::chrono::steady_clock::now();
  const std::optional<hc::OpResult> r = w.wait(1, rc::milliseconds(100));
  const auto waited = std::chrono::steady_clock::now() - t0;
  EXPECT_FALSE(r.has_value()) << "a pending op must time out the wait";
  EXPECT_GE(waited, std::chrono::milliseconds(100));
  EXPECT_LT(waited, std::chrono::milliseconds(1000));
  EXPECT_EQ(w.op_state(1), hc::OpState::kFailed);
  const std::optional<hc::OpResult> again = w.wait(1, rc::Duration{});
  ASSERT_TRUE(again.has_value());
  EXPECT_FALSE(again->ok);
  EXPECT_NE(again->error.find("timed out"), std::string::npos) << again->error;
  ASSERT_TRUE(w.await([&] { return w.stats().ops_cancelled == 1; }, rc::milliseconds(1000)));

  // Nobody waits: the op still fails at its own deadline.
  w.configure(hc::OpTicket{2, system_now() + rc::milliseconds(50)}, system_now());
  ASSERT_TRUE(
      w.await([&] { return w.op_state(2) != hc::OpState::kPending; }, rc::milliseconds(1000)));
  EXPECT_EQ(w.op_state(2), hc::OpState::kFailed);
  const std::optional<hc::OpResult> late = w.wait(2, rc::Duration{});
  ASSERT_TRUE(late.has_value());
  EXPECT_NE(late->error.find("deadline"), std::string::npos) << late->error;
  EXPECT_TRUE(w.shutdown(rc::milliseconds(1000)));
}

TEST(Worker, TimedOutOpCannotActLater) {
  FakeBus bus(system_now);
  cb::WorkerOptions options{cb::kRxBudget, false};
  options.cancel_ack_timeout = rc::milliseconds(500);  // Generous: the gate below opens in 5 ms.
  cb::Worker w(config(), bus.transport(), options);
  std::string error;
  ASSERT_TRUE(w.start(&error)) << error;
  // The wake that starts configure is held inside its step, at the first probe upload, while
  // the caller's wait times out; it is let go 5 ms after the timeout.
  // Whatever that wake still sends must leave before wait() returns, and nothing of the op after.
  bus.close_gate(is_sdo_request);
  w.configure(hc::OpTicket{1, system_now() + hc::kConfigureTimeout}, system_now());
  ASSERT_TRUE(bus.wait_blocked(std::chrono::milliseconds(1000)))
      << "INCONCLUSIVE: configure sent no probe";
  constexpr auto kTimeout = std::chrono::milliseconds(20);
  const auto t_call = std::chrono::steady_clock::now();
  std::thread opener([&] {
    std::this_thread::sleep_until(t_call + kTimeout + std::chrono::milliseconds(5));
    bus.open_gate();
  });
  const std::optional<hc::OpResult> r = w.wait(1, rc::milliseconds(kTimeout.count()));
  const std::size_t at_return = bus.sent().size();
  opener.join();
  ASSERT_FALSE(r.has_value());
  std::vector<cc::Frame> requests;
  for (const Sent& s : bus.sent()) {
    if (is_sdo_request(s.frame)) requests.push_back(s.frame);
  }
  ASSERT_EQ(requests.size(), 6U) << "INCONCLUSIVE: expected one request per node";
  // The answers to the requests on the wire arrive after the cancel.
  // A live op would send each node's next probe (0x1018:01) in reply; a cancelled one sends
  // nothing.
  for (const cc::Frame& req : requests) bus.deliver(upload_answer(req, 0x00020192));
  const std::uint64_t rx_before = w.stats().rx_frames;
  ASSERT_TRUE(w.await([&] { return w.stats().rx_frames >= rx_before + 6; }, rc::milliseconds(1000)))
      << "INCONCLUSIVE: the answers were not read";
  const std::uint64_t wakes = w.stats().wakes;
  ASSERT_TRUE(w.await([&] { return w.stats().wakes >= wakes + 5; }, rc::milliseconds(1000)));
  std::vector<cc::Frame> later;
  const std::vector<Sent> all = bus.sent();
  for (std::size_t k = at_return; k < all.size(); ++k) {
    if (is_sdo_request(all[k].frame) || all[k].frame.cob_id == 0x000) {
      later.push_back(all[k].frame);
    }
  }
  EXPECT_TRUE(later.empty()) << later.size() << " SDO/NMT frames of the op sent after wait() "
                             << "returned; first COB 0x" << std::hex
                             << (later.empty() ? 0 : later.front().cob_id);
  EXPECT_EQ(w.stats().ops_cancelled, 1U);
  EXPECT_EQ(w.op_state(1), hc::OpState::kFailed);
  EXPECT_TRUE(w.inhibited()) << "a failed op latches the inhibit (C17)";
  EXPECT_TRUE(w.shutdown(rc::milliseconds(1000)));
}

TEST(Worker, UnackedCancelLatchesTheInhibit) {
  FakeBus bus(system_now);
  cb::Worker w(config(), bus.transport());  // cancel_ack_timeout: the default 3 W.
  std::string error;
  ASSERT_TRUE(w.start(&error)) << error;
  // The worker is held inside a step for longer than the wait and its ack bound together.
  bus.close_gate(is_sdo_request);
  w.configure(hc::OpTicket{1, system_now() + hc::kConfigureTimeout}, system_now());
  ASSERT_TRUE(bus.wait_blocked(std::chrono::milliseconds(1000)))
      << "INCONCLUSIVE: configure sent no probe";
  ASSERT_FALSE(w.inhibited()) << "INCONCLUSIVE: inhibited before the wait";
  const auto t_call = std::chrono::steady_clock::now();
  const std::optional<hc::OpResult> r = w.wait(1, rc::milliseconds(20));
  const auto waited = std::chrono::steady_clock::now() - t_call;
  EXPECT_FALSE(r.has_value());
  EXPECT_GE(waited, std::chrono::milliseconds(26)) << "returned before the ack bound";
  EXPECT_EQ(w.stats().cancel_ack_timeouts, 1U);
  EXPECT_TRUE(w.inhibited()) << "an unacknowledged cancel must latch the inhibit at once";
  bus.open_gate();
  EXPECT_TRUE(w.shutdown(rc::milliseconds(1000)));
}

// An op that ends after its table slot went to a newer op (kOpTable = 16 ids later) must leave
// the newer op's error alone.
TEST(Worker, StaleOpKeepsTheNewerOpsError) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });  // Nobody answers: a configure stays in its probe.
  cb::Worker w(config(), bus.transport());
  const auto wake = [&] {
    now = now + kW;
    w.run_once(now);
  };
  // Op 5 takes the first in-flight entry; op 1 supersedes it and takes the second.
  w.configure(hc::OpTicket{5, now + hc::kConfigureTimeout}, now);
  wake();
  w.configure(hc::OpTicket{1, now + hc::kConfigureTimeout}, now);
  wake();
  ASSERT_EQ(w.op_state(5), hc::OpState::kFailed) << "INCONCLUSIVE: op 5 was not superseded";
  ASSERT_EQ(w.op_state(1), hc::OpState::kPending) << "INCONCLUSIVE: op 1 is not in flight";
  // Op 17 shares op 1's table slot and the first in-flight entry, so it is published first.
  // It supersedes op 1, which ends in the same wake.
  w.activate(hc::OpTicket{17, now + hc::kActivateTimeout}, now);
  wake();
  ASSERT_EQ(w.op_state(17), hc::OpState::kFailed) << "INCONCLUSIVE: op 17 did not end";
  const std::optional<hc::OpResult> r = w.wait(17, rc::Duration{});
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->error.find("superseded"), std::string::npos)
      << "op 17 reports the error of the op it superseded: " << r->error;
}

// A post refused for a full op queue never touches a live table slot.
// Op 17 shares op 1's slot; it is refused while op 1 is still in flight, and op 1 ends inside
// that refusal (the seam runs a wake there).
// Op 1's end must still be published, never overwritten by the refused post.
TEST(Worker, RefusedPostKeepsAnOlderOpsEnd) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });  // Nobody answers: a configure stays in its probe.
  cb::Worker* worker = nullptr;
  int refused = 0;
  cb::WorkerOptions options;
  options.on_post_refused = [&](std::uint64_t op_id) {
    ++refused;
    EXPECT_EQ(op_id, 17U);
    now = now + kW;
    worker->run_once(now);
  };
  cb::Worker w(config(), bus.transport(), options);
  worker = &w;
  w.configure(hc::OpTicket{1, now + hc::kConfigureTimeout}, now);
  now = now + kW;
  w.run_once(now);
  ASSERT_EQ(w.op_state(1), hc::OpState::kPending) << "INCONCLUSIVE: op 1 is not in flight";
  // Ops 2 to 9 fill the queue; the next wake starts each, and the first supersedes op 1.
  for (std::uint64_t id = 2; id < 2 + cb::kOpQueueCapacity; ++id) {
    w.activate(hc::OpTicket{id, now + hc::kActivateTimeout}, now);
  }
  w.activate(hc::OpTicket{17, now + hc::kActivateTimeout}, now);
  ASSERT_EQ(refused, 1) << "INCONCLUSIVE: op 17 was not refused for a full queue";
  EXPECT_EQ(w.op_state(17), hc::OpState::kFailed);
  for (int i = 0; i < 3; ++i) {
    now = now + kW;
    w.run_once(now);
  }
  EXPECT_EQ(w.op_state(1), hc::OpState::kFailed)
      << "op 1 ended inside the refused post, and its end was lost";
  const std::optional<hc::OpResult> r = w.wait(1, rc::Duration{});
  ASSERT_TRUE(r.has_value()) << "op 1 reads as still pending";
  EXPECT_FALSE(r->ok);
}

// The same refusal with the worker thread running, for the thread sanitizer: the control thread
// checks the queue's room while the worker drains it.
// The worker is held inside the wake that started op 1 until the queue is full and op 17 refused.
TEST(Worker, RefusedPostWithTheWorkerThread) {
  FakeBus bus(system_now);
  cb::Worker w(config(), bus.transport());
  std::string error;
  ASSERT_TRUE(w.start(&error)) << error;
  bus.close_gate(is_sdo_request);
  w.configure(hc::OpTicket{1, system_now() + hc::kConfigureTimeout}, system_now());
  ASSERT_TRUE(bus.wait_blocked(std::chrono::milliseconds(1000)))
      << "INCONCLUSIVE: configure sent no probe";
  for (std::uint64_t id = 2; id < 2 + cb::kOpQueueCapacity; ++id) {
    w.activate(hc::OpTicket{id, system_now() + hc::kActivateTimeout}, system_now());
  }
  w.activate(hc::OpTicket{17, system_now() + hc::kActivateTimeout}, system_now());
  ASSERT_EQ(w.stats().ops_refused, 1U) << "INCONCLUSIVE: op 17 was not refused";
  EXPECT_EQ(w.op_state(17), hc::OpState::kFailed);
  bus.open_gate();
  ASSERT_TRUE(
      w.await([&] { return w.op_state(1) != hc::OpState::kPending; }, rc::milliseconds(1000)))
      << "op 1 never ended after a refused post shared its slot";
  EXPECT_EQ(w.op_state(1), hc::OpState::kFailed);
  EXPECT_TRUE(w.shutdown(rc::milliseconds(1000)));
}

TEST(Worker, SocketWriteErrorsAreCounted) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });
  bus.fail_writes({ENOBUFS, ENOBUFS, ENOBUFS, EAGAIN, EAGAIN});
  cb::Worker w(config(), bus.transport(), cb::WorkerOptions{cb::kRxBudget, false});
  for (int k = 0; k < 60; ++k) {  // 120 ms: about twelve SYNCs, the first five fail.
    w.run_once(now);
    now = now + kW;
  }
  const cb::WorkerStats s = w.stats();
  EXPECT_EQ(s.transport.tx_enobufs, 3U);
  EXPECT_EQ(s.transport.tx_eagain, 2U);
  EXPECT_EQ(s.transport.tx_other_error, 0U);
  EXPECT_GE(s.transport.tx, 5U) << "INCONCLUSIVE: too few frames after the failures";
  EXPECT_EQ(s.transport.tx, bus.sent().size());
  // The CLI's exit record carries the same counters (spec §4 IPC policy: counted, not retried).
  const std::string json = cb::stats_json(s);
  EXPECT_NE(json.find("\"tx_enobufs\":3"), std::string::npos) << json;
  EXPECT_NE(json.find("\"tx_eagain\":2"), std::string::npos) << json;
  EXPECT_NE(json.find("\"tx_other_error\":0"), std::string::npos) << json;
  // A full queue is transient: counted, never a transport loss.
  EXPECT_FALSE(w.poll(now).faults.has(hc::Fault::kTransportLost));
}

TEST(Worker, OtherWriteErrorIsTransportLoss) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });
  cb::Worker w(config(), bus.transport());
  w.run_once(now);
  ASSERT_FALSE(w.poll(now).faults.has(hc::Fault::kTransportLost)) << "INCONCLUSIVE: lost at start";
  ASSERT_FALSE(w.inhibited()) << "INCONCLUSIVE: inhibited at start";
  bus.fail_writes({ENETDOWN});
  for (int k = 0; k < 10; ++k) {  // 20 ms: at least one SYNC is written, the first fails.
    now = now + kW;
    w.run_once(now);
  }
  ASSERT_EQ(w.stats().transport.tx_other_error, 1U) << "INCONCLUSIVE: the failure was not hit";
  EXPECT_TRUE(w.poll(now).faults.has(hc::Fault::kTransportLost));
  EXPECT_TRUE(w.inhibited());
}

TEST(Worker, OpsQueuedWithShutdownDoNotStart) {
  rc::MonoTime now = kT0;
  FakeBus bus([&] { return now; });
  cb::Worker w(config(), bus.transport());
  w.run_once(now);
  // on_error_nonblocking requests the shutdown; a configure posted with it must not start.
  w.on_error_nonblocking();
  w.configure(hc::OpTicket{1, now + hc::kConfigureTimeout}, now);
  now = now + kW;
  w.run_once(now);
  EXPECT_EQ(w.op_state(1), hc::OpState::kFailed);
  const std::optional<hc::OpResult> r = w.wait(1, rc::Duration{});
  ASSERT_TRUE(r.has_value());
  EXPECT_NE(r->error.find("shutting down"), std::string::npos) << r->error;
  for (const Sent& sent : bus.sent()) {
    EXPECT_FALSE(is_sdo_request(sent.frame) || sent.frame.cob_id == 0x000)
        << "the configure sent COB 0x" << std::hex << sent.frame.cob_id;
  }
}
