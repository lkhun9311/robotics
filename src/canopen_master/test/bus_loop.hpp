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
#include "canopen_master/testing/drive_bus.hpp"
#include "canopen_master/vdrive_session.hpp"
#include "drive_core/drive.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/time.hpp"

// The test bus: the master core with its vdrive session on one side, six in-memory
// drive_core::Drive on the other, in logic time. Every frame either side sends is recorded and
// then offered to a filter that delivers it, drops it or holds it for later re-delivery. Frames
// are delivered at the instant they are sent (zero transport delay); the drives tick every W and
// before any frame of the same instant. The device side (the drives, their heartbeats and EMCYs)
// is the shared DriveBus (canopen_master/testing/drive_bus.hpp).
namespace canopen_master::testing {

namespace cc = canopen_codec;

inline constexpr runtime_core::MonoTime kT0{1'000'000'000'000};  // 1000 s
inline constexpr runtime_core::Duration kW = drive_core::kTickPeriod;

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

enum class Fate : std::uint8_t { kDeliver, kDrop, kHold };

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
      : master(std::move(master_config)), session(master, session_config) {}
  BusLoop(const BusLoop&) = delete;
  BusLoop& operator=(const BusLoop&) = delete;

  // Powers every drive on at `now`; the boot-ups reach the session.
  void power_on() {
    from_drives(bus.power_on(now));
    pump();
  }

  // One W: the drives tick, heartbeats fall due, then the session ticks.
  void step() { advance(kW); }

  // Time jumps by `d` in one call (a worker that slept): one drive tick, one session tick.
  void advance(runtime_core::Duration d) {
    now = now + d;
    from_drives(bus.tick(now));
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
    return bus.to_drive(i, f, now);
  }

  void inject_fault(std::size_t i, std::uint16_t code) {
    from_drives(bus.inject_fault(i, code, now));
    pump();
  }
  void inject_torque_loss(std::size_t i) {
    from_drives(bus.inject_torque_loss(i, now));
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

  runtime_core::MonoTime now{kT0};
  DriveBus bus{now};
  Master master;
  VdriveSession session;
  // The device side, by the names the tests use.
  std::array<std::unique_ptr<drive_core::Drive>, kSessionNodes>& drives{bus.drives};
  std::vector<DriveRecord>& drive_events{bus.drive_events};
  // Per node: answers a frame addressed to that drive instead of it (a scripted peer for one
  // object); std::nullopt lets the drive handle the frame.
  std::array<Intercept, kSessionNodes>& intercept{bus.intercept};
  // Periodic heartbeats of a muted node are not sent (boot-ups still are).
  std::array<bool, kSessionNodes>& heartbeat_muted{bus.heartbeat_muted};
  std::vector<WireFrame> wire;
  std::vector<WireFrame> held;
  // Decides each frame's fate; empty delivers everything.
  std::function<Fate(const WireFrame&)> filter;
  std::function<void()> after_step;
  bool sync_this_step{false};
  // When set, every SDO answer arrives at the next step instead of at once: an SDO round trip
  // takes one W, so a chain of requests spans SYNCs as on a real bus.
  bool sdo_round_trip{false};
  // Replaces a frame before it is recorded and delivered; std::nullopt keeps it.
  std::function<std::optional<cc::Frame>(bool from_master, const cc::Frame&)> rewrite;

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
  void from_drives(const std::vector<DriveFrame>& frames) {
    for (const DriveFrame& f : frames) {
      queue_.push_back({false, f.node, f.frame});
    }
  }

  void pump(bool redelivery = false) {
    for (int guard = 0; !queue_.empty(); ++guard) {
      if (guard > 100000) {
        throw std::logic_error("bus loop: frames never settle");
      }
      Pending p = queue_.front();
      if (rewrite) {
        if (const std::optional<cc::Frame> changed = rewrite(p.from_master, p.frame)) {
          p.frame = *changed;  // A scripted peer: the frame as that peer would send it.
        }
      }
      queue_.pop_front();
      const WireFrame w{bus.next_order(), now, p.from_master, p.frame};
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
        from_drives(bus.deliver(p.frame, now));
      } else {
        from_master(session.on_frame(p.frame, now));
      }
      redelivery = false;  // Whatever a re-delivered frame causes is ordinary traffic.
    }
  }

  std::deque<Pending> queue_;
  std::vector<cc::Frame> late_sdo_;
  std::uint64_t next_op_{1};
};

}  // namespace canopen_master::testing
