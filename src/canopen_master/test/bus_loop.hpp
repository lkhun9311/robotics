// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "canopen_master/master.hpp"
#include "canopen_master/vdrive_session.hpp"
#include "drive_core/drive.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/time.hpp"

// The test bus: the master core with its vdrive session on one side, six in-memory
// drive_core::Drive on the other, in logic time. Every frame either side sends is recorded and
// then offered to a filter that delivers it, drops it or holds it for later re-delivery. Frames
// are delivered at the instant they are sent (zero transport delay); the drives tick every W and
// before any frame of the same instant. The loop produces each drive's heartbeat (100 ms, phase
// from its latest boot-up) and its EMCY for an accepted fault, as the vdrive_bank process does.
namespace canopen_master::testing {

namespace cc = canopen_codec;

inline constexpr runtime_core::MonoTime kT0{1'000'000'000'000};  // 1000 s
inline constexpr runtime_core::Duration kW = drive_core::kTickPeriod;
inline constexpr runtime_core::Duration kHeartbeat{100'000'000};

// runtime_core adds durations to instants only.
constexpr runtime_core::Duration plus(runtime_core::Duration a, runtime_core::Duration b) {
  return runtime_core::Duration{a.ns + b.ns};
}

struct WireFrame {
  std::uint64_t order{};  // One counter for frames and drive events: their order of happening.
  runtime_core::MonoTime at{};
  bool from_master{};
  cc::Frame frame{};
};

struct DriveRecord {
  std::uint64_t order{};
  std::size_t node{};  // 0..5
  drive_core::DriveEvent event{};
};

enum class Fate : std::uint8_t { kDeliver, kDrop, kHold };

inline cc::NodeId node_id(std::size_t index) {
  return cc::NodeId::make(static_cast<std::uint8_t>(index + 1)).value();
}

inline MasterConfig default_master_config() {
  MasterConfig c;
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    c.nodes.push_back(node_id(i));
  }
  c.sync_period = kBusPeriod;
  return c;
}

class BusLoop {
 public:
  explicit BusLoop(MasterConfig master_config = default_master_config(),
                   SessionConfig session_config = {})
      : master(std::move(master_config)), session(master, session_config) {
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      // Distinct boot ids: each drive starts its tags elsewhere (D3).
      drives[i] = std::make_unique<drive_core::Drive>(
          node_id(i), 0x5EED'0000U + static_cast<std::uint32_t>(i) * 7919U, now);
      drives[i]->set_event_sink([this, i](const drive_core::DriveEvent& e) {
        pending_events_.push_back(DriveRecord{++order_, i, e});
      });
    }
  }
  BusLoop(const BusLoop&) = delete;
  BusLoop& operator=(const BusLoop&) = delete;

  // Powers every drive on at `now`; the boot-ups reach the session.
  void power_on() {
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      from_drive(i, drives[i]->power_on(now));
    }
    pump();
  }

  // One W: the drives tick, heartbeats fall due, then the session ticks.
  void step() { advance(kW); }

  // Time jumps by `d` in one call (a worker that slept): one drive tick, one session tick.
  void advance(runtime_core::Duration d) {
    now = now + d;
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      drives[i]->tick(now);
      flush_events(i);
      if (now >= next_heartbeat_[i] && powered_[i]) {
        const std::int64_t periods = (now - next_heartbeat_[i]).ns / kHeartbeat.ns + 1;
        next_heartbeat_[i] = next_heartbeat_[i] + runtime_core::Duration{periods * kHeartbeat.ns};
        queue_.push_back({false, i, cc::encode_heartbeat({node_id(i), drives[i]->nmt().state()})});
      }
    }
    pump();
    // SDO answers held back by sdo_round_trip reach the master one W after their request.
    std::vector<cc::Frame> late;
    late.swap(late_sdo_);
    for (const cc::Frame& f : late) {
      queue_.push_back({false, 0, f});
      pump(true);
    }
    const std::uint64_t syncs = master.syncs_produced();
    from_master(session.tick(now));
    pump();
    sync_this_step = master.syncs_produced() != syncs;
    if (after_step) {
      after_step();
    }
  }

  void run(runtime_core::Duration d) {
    const runtime_core::MonoTime end = now + d;
    while (now < end) {
      step();
    }
  }

  // Steps until `done` holds (checked after each step) or `limit` passes; returns `done`.
  template <class Pred>
  bool run_until(Pred done, runtime_core::Duration limit) {
    const runtime_core::MonoTime end = now + limit;
    while (!done()) {
      if (now >= end) {
        return false;
      }
      step();
    }
    return true;
  }

  // Steps until the step that produced a SYNC.
  void to_next_sync() {
    do {
      step();
    } while (!sync_this_step);
  }

  // A frame as if the master had sent it now (re-delivery of a held frame).
  void deliver_from_master(const cc::Frame& f) {
    queue_.push_back({true, 0, f});
    pump(true);
  }
  void deliver_from_drive(const cc::Frame& f) {
    queue_.push_back({false, 0, f});
    pump(true);
  }

  // A frame straight to one drive, outside the bus: its replies come back here, not to the
  // master (a test setting the drive up before the session runs).
  std::vector<cc::Frame> to_drive(std::size_t i, const cc::Frame& f) {
    std::vector<cc::Frame> out = drives[i]->on_frame(f, now);
    for (const DriveRecord& r : pending_events_) {
      drive_events.push_back(r);
    }
    pending_events_.clear();
    return out;
  }

  void inject_fault(std::size_t i, std::uint16_t code) {
    drives[i]->inject_fault(code, now);
    flush_events(i);
    pump();
  }
  void inject_torque_loss(std::size_t i) {
    drives[i]->inject_torque_loss(now);
    flush_events(i);
    pump();
  }

  // ---- ops ----
  hal_contract::OpTicket ticket(runtime_core::Duration timeout) {
    return hal_contract::OpTicket{next_op_++, now + timeout};
  }
  hal_contract::OpState wait(std::uint64_t op, runtime_core::Duration limit) {
    run_until([&] { return session.op_state(op) != hal_contract::OpState::kPending; }, limit);
    return session.op_state(op);
  }
  hal_contract::OpState configure() {
    const hal_contract::OpTicket t = ticket(hal_contract::kConfigureTimeout);
    session.configure(t);
    return wait(t.op_id, plus(hal_contract::kConfigureTimeout, kW));
  }
  hal_contract::OpState activate() {
    const hal_contract::OpTicket t = ticket(hal_contract::kActivateTimeout);
    session.activate(t);
    return wait(t.op_id, plus(hal_contract::kActivateTimeout, kW));
  }
  hal_contract::OpState arm(const hal_contract::AuthorityId& id, runtime_core::Duration hold) {
    const hal_contract::OpTicket t = ticket(hal_contract::kArmTimeout);
    session.arm(t, hal_contract::ArmRequest{id, now + hold, t.op_id});
    return wait(t.op_id, plus(hal_contract::kArmTimeout, kW));
  }
  hal_contract::OpState deactivate() {
    const hal_contract::OpTicket t = ticket(hal_contract::kDeactivateTimeout);
    session.deactivate(t);
    return wait(t.op_id, plus(hal_contract::kDeactivateTimeout, kW));
  }

  Master master;
  VdriveSession session;
  runtime_core::MonoTime now{kT0};
  std::array<std::unique_ptr<drive_core::Drive>, kSessionNodes> drives;
  std::vector<WireFrame> wire;
  std::vector<DriveRecord> drive_events;
  std::vector<WireFrame> held;
  // Decides each frame's fate; empty delivers everything.
  std::function<Fate(const WireFrame&)> filter;
  // Per node: answers a frame addressed to that drive instead of it (a scripted peer for one
  // object); std::nullopt lets the drive handle the frame.
  std::array<std::function<std::optional<cc::Frame>(const cc::Frame&)>, kSessionNodes> intercept;
  std::function<void()> after_step;
  bool sync_this_step{false};
  // When set, every SDO answer arrives at the next step instead of at once: an SDO round trip
  // takes one W, so a chain of requests spans SYNCs as on a real bus.
  bool sdo_round_trip{false};

 private:
  struct Pending {
    bool from_master{};
    std::size_t node{};  // The sending drive, for frames from a drive.
    cc::Frame frame{};
  };

  void from_master(const std::vector<cc::Frame>& frames) {
    for (const cc::Frame& f : frames) {
      queue_.push_back({true, 0, f});
    }
  }
  void from_drive(std::size_t i, const std::vector<cc::Frame>& frames) {
    for (const cc::Frame& f : frames) {
      if (f.cob_id == 0x700U + node_id(i).value() && f.dlc == 1 && f.data[0] == 0) {
        powered_[i] = true;
        next_heartbeat_[i] = now + kHeartbeat;  // A boot-up restarts the heartbeat phase.
      }
      queue_.push_back({false, i, f});
    }
    flush_events(i);
  }

  // Records the drive's events; an accepted fault sends its EMCY (Pre-op and Operational).
  void flush_events(std::size_t i) {
    std::vector<DriveRecord> events;
    events.swap(pending_events_);
    for (const DriveRecord& r : events) {
      drive_events.push_back(r);
      const cc::NmtState s = drives[r.node]->nmt().state();
      if (r.event.kind == drive_core::DriveEventKind::kFault &&
          (s == cc::NmtState::kPreOperational || s == cc::NmtState::kOperational)) {
        queue_.push_back(
            {false, r.node,
             cc::encode_emcy(
                 {node_id(r.node), r.event.fault_code, static_cast<std::uint8_t>(1), {}})});
      }
    }
    (void)i;
  }

  void pump(bool redelivery = false) {
    for (int guard = 0; !queue_.empty(); ++guard) {
      if (guard > 100000) {
        throw std::logic_error("bus loop: frames never settle");
      }
      const Pending p = queue_.front();
      queue_.pop_front();
      const WireFrame w{++order_, now, p.from_master, p.frame};
      wire.push_back(w);
      const Fate fate = (filter && !redelivery) ? filter(w) : Fate::kDeliver;
      if (fate == Fate::kDrop) {
        continue;
      }
      if (sdo_round_trip && !p.from_master && !redelivery && p.frame.cob_id > 0x580 &&
          p.frame.cob_id <= 0x5FF) {
        late_sdo_.push_back(p.frame);
        continue;
      }
      if (fate == Fate::kHold) {
        held.push_back(w);
        continue;
      }
      if (p.from_master) {
        for (std::size_t i = 0; i < kSessionNodes; ++i) {
          if (intercept[i]) {
            if (const std::optional<cc::Frame> reply = intercept[i](p.frame)) {
              queue_.push_back({false, i, *reply});
              continue;
            }
          }
          from_drive(i, drives[i]->on_frame(p.frame, now));
        }
      } else {
        from_master(session.on_frame(p.frame, now));
      }
      redelivery = false;  // Whatever a re-delivered frame causes is ordinary traffic.
    }
  }

  std::deque<Pending> queue_;
  std::vector<cc::Frame> late_sdo_;
  std::vector<DriveRecord> pending_events_;
  std::array<runtime_core::MonoTime, kSessionNodes> next_heartbeat_{};
  std::array<bool, kSessionNodes> powered_{};
  std::uint64_t order_{0};
  std::uint64_t next_op_{1};
};

}  // namespace canopen_master::testing
