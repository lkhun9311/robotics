// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "canopen_backend/canopen_backend.hpp"
#include "canopen_backend/transport.hpp"
#include "canopen_codec/codec.hpp"
#include "canopen_master/testing/drive_bus.hpp"
#include "drive_core/cia402.hpp"
#include "drive_core/drive.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/time.hpp"

// The logic-time rig of the canopen backend tests: the backend over an in-memory transport to the
// shared DriveBus (six drive_core::Drive), stepped every W. Frames cross the transport at the
// instant they are sent, unless a fault hook says otherwise; the drives tick before the backend
// steps at the same instant.
// What the drives did is translated into the contract suite's backend-neutral device events.
namespace canopen_backend::testing {

namespace cc = canopen_codec;
namespace cmt = canopen_master::testing;

inline constexpr std::size_t kNodes = cmt::kBusNodes;
inline constexpr runtime_core::Duration kW = drive_core::kTickPeriod;
inline constexpr runtime_core::MonoTime kT0{1'000'000'000'000};  // 1000 s, a multiple of W

struct WireFrame {
  runtime_core::MonoTime at{};
  bool from_master{};
  cc::Frame frame{};
};

// What a drive did, as the drive saw it (the contract suite's DeviceEventKind, by name).
enum class RigEventKind : std::uint8_t {
  kActuated,
  kDisabled,
  kCommitted,
  kEnabled,
  kLocalStop,
  kQuickStop
};

struct RigEvent {
  runtime_core::MonoTime at{};
  std::size_t node{0};
  RigEventKind kind{RigEventKind::kActuated};
  double target{0.0};
  double position{0.0};
  double velocity{0.0};
  std::optional<std::uint64_t> source_seq{};
  hal_contract::AuthorityId id{};
};

inline bool is_tpdo(const cc::Frame& f) {
  for (unsigned base : {0x180U, 0x280U, 0x380U}) {
    if (f.cob_id > base && f.cob_id <= base + kNodes) return true;
  }
  return false;
}
inline bool is_command_pdo(const cc::Frame& f) {
  for (unsigned base : {0x200U, 0x300U}) {  // RPDO1 targets, RPDO2 deadline/seq/tag
    if (f.cob_id > base && f.cob_id <= base + kNodes) return true;
  }
  return false;
}

class Rig;

// The backend's side of the in-memory bus.
class RigTransport final : public Transport {
 public:
  explicit RigTransport(Rig& rig) : rig_(rig) {}
  void send(const cc::Frame& frame) override;
  std::vector<cc::Frame> poll() override;

 private:
  Rig& rig_;
};

class Rig {
 public:
  Rig() = default;
  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;

  static CanopenConfig default_config() {
    CanopenConfig c;
    c.session.max_feedback_age = runtime_core::milliseconds(20);
    return c;
  }

  // A backend over this rig's bus; the rig steps it (the caller may hand ownership to a Hal).
  std::unique_ptr<CanopenBackend> make_backend(CanopenConfig config = default_config()) {
    auto b =
        std::make_unique<CanopenBackend>(std::move(config), std::make_unique<RigTransport>(*this));
    backend = b.get();
    return b;
  }

  void power_on() { route(bus.power_on(now)); }

  // Moves time by `d` in W steps: the drives tick, then the backend steps.
  void advance(runtime_core::Duration d) {
    const runtime_core::MonoTime until = now + d;
    while (now + kW <= until) {
      now = now + kW;
      for (std::size_t i = 0; i < kNodes; ++i) {
        if (inject_[i]) route(bus.inject_fault(i, drive_core::kErrorFollowing, now));
        inject_[i] = false;
      }
      route(bus.tick(now));
      observe();
      if (backend != nullptr) backend->step(now);
    }
    now = until;
  }
  template <class Pred>
  bool advance_until(Pred done, runtime_core::Duration limit) {
    const runtime_core::MonoTime end = now + limit;
    while (!done()) {
      if (now >= end) return false;
      advance(kW);
    }
    return true;
  }

  hal_contract::OpTicket ticket(runtime_core::Duration timeout) {
    return hal_contract::OpTicket{next_op_++, now + timeout};
  }
  // Posts an op on the backend directly and steps until it ends; returns its final state.
  template <class Post>
  hal_contract::OpState run_op(runtime_core::Duration timeout, Post post) {
    const hal_contract::OpTicket t = ticket(timeout);
    post(t);
    advance_until([&] { return backend->op_state(t.op_id) != hal_contract::OpState::kPending; },
                  runtime_core::Duration{timeout.ns + 2 * kW.ns});
    return backend->op_state(t.op_id);
  }
  hal_contract::OpState configure() {
    return run_op(hal_contract::kConfigureTimeout,
                  [&](const hal_contract::OpTicket& t) { backend->configure(t, now); });
  }
  hal_contract::OpState activate() {
    return run_op(hal_contract::kActivateTimeout,
                  [&](const hal_contract::OpTicket& t) { backend->activate(t, now); });
  }
  hal_contract::OpState arm(const hal_contract::AuthorityId& id, runtime_core::Duration hold) {
    return run_op(hal_contract::kArmTimeout, [&](const hal_contract::OpTicket& t) {
      backend->arm(t, hal_contract::ArmRequest{id, now + hold, t.op_id}, now);
    });
  }
  hal_contract::OpState deactivate() {
    return run_op(hal_contract::kDeactivateTimeout,
                  [&](const hal_contract::OpTicket& t) { backend->deactivate(t, now); });
  }

  // Drives every joint to `target` (rad) through the backend, then leaves the drives at rest
  // there in a fresh boot, with nothing connected: what a bench looks like before a test.
  // Throws std::logic_error when the bring-up does not complete.
  void place(const hal_contract::Joints& target) {
    std::unique_ptr<CanopenBackend> b = make_backend();
    power_on();
    advance(kW);
    const hal_contract::AuthorityId id{{900}, {900}, {900}};
    const auto require = [&](hal_contract::OpState s, const char* what) {
      if (s != hal_contract::OpState::kSucceeded) {
        const std::optional<hal_contract::OpResult> r = backend->wait(next_op_ - 1, {});
        throw std::logic_error(std::string("rig: placement ") + what +
                               " failed: " + (r ? r->error : std::string("pending")));
      }
    };
    require(configure(), "configure");
    require(activate(), "activate");
    require(arm(id, runtime_core::milliseconds(1000)), "arm");
    std::uint64_t seq = 0;
    for (int k = 0; k < 100 && !at(target); ++k) {
      hal_contract::ApprovedCommand c{id, ++seq, target, now, now + runtime_core::milliseconds(40)};
      backend->submit(c, now);
      advance(runtime_core::milliseconds(10));
    }
    if (!at(target) || deactivate() != hal_contract::OpState::kSucceeded) {
      throw std::logic_error("rig: placement did not reach the target");
    }
    backend = nullptr;
    b.reset();
    route(bus.deliver(cc::encode_nmt({cc::NmtCommand::kResetNode, std::nullopt}), now));
    // Past the boot quarantine of the new boot (D3), with nothing listening.
    advance(runtime_core::milliseconds(200));
    rx_.clear();
    wire.clear();
    events.clear();
    bus.drive_events.clear();
    seen_events_ = 0;
    tags_ = {};
  }

  bool at(const hal_contract::Joints& target) const {
    for (std::size_t i = 0; i < kNodes; ++i) {
      if (bus.drives[i]->joint().position_nrad != std::llround(target[i] * 1e9) ||
          bus.drives[i]->joint().velocity_nrad_s != 0) {
        return false;
      }
    }
    return true;
  }

  hal_contract::Joints position() const {
    hal_contract::Joints out{};
    for (std::size_t i = 0; i < kNodes; ++i) {
      out[i] = static_cast<double>(bus.drives[i]->joint().position_nrad) / 1e9;
    }
    return out;
  }
  hal_contract::Joints velocity() const {
    hal_contract::Joints out{};
    for (std::size_t i = 0; i < kNodes; ++i) {
      out[i] = static_cast<double>(bus.drives[i]->joint().velocity_nrad_s) / 1e9;
    }
    return out;
  }

  // Axis `i` stops on its own at its next step: an injected following error (Fault Reaction,
  // then Fault), with its EMCY.
  void inject_stop(std::size_t i) { inject_[i] = true; }

  // ---- the transport, from the backend ----
  void from_master(const cc::Frame& sent) {
    cc::Frame f = sent;
    if (rewrite) {
      if (const std::optional<cc::Frame> changed = rewrite(true, f)) f = *changed;
    }
    wire.push_back(WireFrame{now, true, f});
    if (drop_commands && is_command_pdo(f)) return;
    const std::vector<cmt::DriveFrame> replies = bus.deliver(f, now);
    observe();
    route(replies);
  }
  std::vector<cc::Frame> take_received() {
    std::vector<cc::Frame> out;
    while (!rx_.empty() && rx_.front().first <= now) {
      out.push_back(rx_.front().second);
      rx_.pop_front();
    }
    return out;
  }

  runtime_core::MonoTime now{kT0};
  cmt::DriveBus bus{now};
  CanopenBackend* backend{nullptr};
  std::vector<WireFrame> wire;
  std::vector<RigEvent> events;

  // ---- fault hooks ----
  // Replaces a frame before it is recorded and delivered (a scripted peer); nullopt keeps it.
  std::function<std::optional<cc::Frame>(bool from_master, const cc::Frame&)> rewrite;
  bool stall_samples{false};       // TPDOs from the drives are lost
  runtime_core::Duration delay{};  // every frame from the drives arrives this much later
  bool drop_commands{false};       // RPDO1/RPDO2 never reach the drives

 private:
  void route(const std::vector<cmt::DriveFrame>& frames) {
    for (const cmt::DriveFrame& d : frames) {
      cc::Frame f = d.frame;
      if (rewrite) {
        if (const std::optional<cc::Frame> changed = rewrite(false, f)) f = *changed;
      }
      wire.push_back(WireFrame{now, false, f});
      note_commit(d.node, f);
      if (stall_samples && is_tpdo(f)) continue;
      rx_.emplace_back(now + delay, f);
    }
  }

  // A drive's successful answer to the 0x2011 commit write: the device committed a new identity.
  void note_commit(std::size_t i, const cc::Frame& f) {
    const std::optional<cc::Sdo> s = cc::decode_sdo(cmt::node_id(i), f);
    if (f.cob_id != 0x580U + cmt::node_id(i).value() || !s ||
        s->type != cc::SdoType::kDownloadResponse || s->address.index != 0x2011) {
      return;
    }
    const drive_core::ExecTagIssuer& issuer = bus.drives[i]->issuer();
    // The wire carries generation and session; the manager instance is the host's, joined from
    // the arm request the backend was running when the device committed.
    hal_contract::AuthorityId id{runtime_core::ManagerInstance{0},
                                 runtime_core::SessionId{issuer.session()},
                                 runtime_core::Generation{issuer.generation()}};
    const std::optional<hal_contract::ArmRequest> arm =
        backend != nullptr ? backend->last_arm() : std::nullopt;
    if (arm) {
      const hal_contract::AuthorityId host = arm->id;
      if (host.generation.value == issuer.generation() &&
          static_cast<std::uint32_t>(host.session.value) == issuer.session()) {
        id = host;
      }
    }
    tags_[i][issuer.current()] = id;
    push(i, RigEventKind::kCommitted, 0.0, std::nullopt, id);
  }

  void observe() {
    for (; seen_events_ < bus.drive_events.size(); ++seen_events_) {
      const cmt::DriveRecord& r = bus.drive_events[seen_events_];
      const drive_core::DriveEvent& e = r.event;
      const std::size_t i = r.node;
      switch (e.kind) {
        case drive_core::DriveEventKind::kActuated: {
          const std::optional<drive_core::Command>& c = bus.drives[i]->actuated();
          const double target = c ? static_cast<double>(c->target_position) / 1e6 : 0.0;
          std::optional<std::uint64_t> source;
          if (backend != nullptr) {
            const auto d = backend->session().dispatched(e.seq);
            if (!d) throw std::logic_error("rig: a drive actuated a seq the backend never sent");
            if (d->approved) source = d->source_seq;
          }
          push(i, RigEventKind::kActuated, target, source, tag_id(i, e.tag));
          break;
        }
        case drive_core::DriveEventKind::kQuickStop:
          // The drive's own stops (expiry, timeout, SYNC loss, integrity, comm state).
          // A stop by controlword emits no drive event; it is seen as the transition below.
          push(i, e.reason == drive_core::QuickStopReason::kControlword ? RigEventKind::kQuickStop
                                                                        : RigEventKind::kLocalStop);
          break;
        case drive_core::DriveEventKind::kFault:
          push(i, RigEventKind::kLocalStop);
          break;
        default:
          break;
      }
    }
    for (std::size_t i = 0; i < kNodes; ++i) {
      const drive_core::Cia402State s = bus.drives[i]->cia402().state();
      const bool enabled = s == drive_core::Cia402State::kOperationEnabled;
      const bool was = last_state_[i] == drive_core::Cia402State::kOperationEnabled;
      if (enabled && !was) {
        push(i, RigEventKind::kEnabled, 0.0, std::nullopt,
             tag_id(i, bus.drives[i]->issuer().current()));
      } else if (!enabled && was && s == drive_core::Cia402State::kSwitchedOn) {
        push(i, RigEventKind::kDisabled);
      } else if (was && s == drive_core::Cia402State::kQuickStopActive &&
                 bus.drives[i]->cia402().quick_stop_reason() ==
                     drive_core::QuickStopReason::kControlword) {
        push(i, RigEventKind::kQuickStop);  // The host's Quick Stop (controlword 0x02).
      }
      last_state_[i] = s;
    }
  }

  hal_contract::AuthorityId tag_id(std::size_t i, std::uint16_t tag) const {
    const auto it = tags_[i].find(tag);
    return it == tags_[i].end() ? hal_contract::AuthorityId{} : it->second;
  }

  void push(std::size_t i, RigEventKind kind, double target = 0.0,
            std::optional<std::uint64_t> source = std::nullopt, hal_contract::AuthorityId id = {}) {
    const drive_core::JointState& j = bus.drives[i]->joint();
    events.push_back(RigEvent{now, i, kind, target, static_cast<double>(j.position_nrad) / 1e9,
                              static_cast<double>(j.velocity_nrad_s) / 1e9, source, id});
  }

  std::deque<std::pair<runtime_core::MonoTime, cc::Frame>> rx_;
  std::size_t seen_events_{0};
  std::array<bool, kNodes> inject_{};
  std::array<drive_core::Cia402State, kNodes> last_state_{};
  std::array<std::map<std::uint16_t, hal_contract::AuthorityId>, kNodes> tags_{};
  std::uint64_t next_op_{1};
};

inline void RigTransport::send(const cc::Frame& frame) { rig_.from_master(frame); }
inline std::vector<cc::Frame> RigTransport::poll() { return rig_.take_received(); }

}  // namespace canopen_backend::testing
