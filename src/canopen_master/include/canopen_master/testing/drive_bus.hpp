// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "canopen_master/feedback_join.hpp"
#include "drive_core/drive.hpp"
#include "runtime_core/time.hpp"

// The device side of the logic-time test buses: six in-memory drive_core::Drive (node IDs 1-6)
// with what the vdrive_bank process adds around each drive — its heartbeat (100 ms, phase from
// its latest boot-up) and its EMCY for an accepted fault. Shared by the vdrive session's bus loop
// and the canopen backend's in-memory transport. Header-only test utility: a test that includes
// it links drive_core itself. Not a library header: the exported-targets check of run_checks.sh
// leaves canopen_master/testing/ out, since drive_core is only a test dependency of canopen_master.
//
// Also the scripted-peer helpers both test suites use: a node that answers one object
// differently (an intercept) or sends one PDO field differently (a rewrite).
namespace canopen_master::testing {

namespace cc = canopen_codec;

inline constexpr std::size_t kBusNodes = kSessionNodes;
inline constexpr runtime_core::Duration kHeartbeat{100'000'000};

inline cc::NodeId node_id(std::size_t index) {
  return cc::NodeId::make(static_cast<std::uint8_t>(index + 1)).value();
}

struct DriveRecord {
  std::uint64_t order{};  // One counter for frames and drive events: their order of happening.
  std::size_t node{};     // 0..5
  drive_core::DriveEvent event{};
};

// A frame one drive (or a scripted peer in its place) sends.
struct DriveFrame {
  std::size_t node{};
  cc::Frame frame{};
};

// Answers a frame addressed to one drive instead of it; std::nullopt lets the drive handle it.
using Intercept = std::function<std::optional<cc::Frame>(const cc::Frame&)>;

class DriveBus {
 public:
  explicit DriveBus(runtime_core::MonoTime now) {
    for (std::size_t i = 0; i < kBusNodes; ++i) {
      // Distinct boot ids: each drive starts its tags elsewhere (D3).
      drives[i] = std::make_unique<drive_core::Drive>(
          node_id(i), 0x5EED'0000U + static_cast<std::uint32_t>(i) * 7919U, now);
      drives[i]->set_event_sink([this, i](const drive_core::DriveEvent& e) {
        pending_events_.push_back(DriveRecord{++order_, i, e});
      });
    }
  }
  DriveBus(const DriveBus&) = delete;
  DriveBus& operator=(const DriveBus&) = delete;

  // Powers every drive on at `now`; returns the boot-ups.
  std::vector<DriveFrame> power_on(runtime_core::MonoTime now) {
    std::vector<DriveFrame> out;
    for (std::size_t i = 0; i < kBusNodes; ++i) {
      from_drive(i, drives[i]->power_on(now), now, out);
    }
    return out;
  }

  // One W at `now`: every drive ticks, then its EMCY and its heartbeat if one fell due.
  std::vector<DriveFrame> tick(runtime_core::MonoTime now) {
    std::vector<DriveFrame> out;
    for (std::size_t i = 0; i < kBusNodes; ++i) {
      drives[i]->tick(now);
      flush_events(out);
      if (now >= next_heartbeat_[i] && powered_[i]) {
        const std::int64_t periods = (now - next_heartbeat_[i]).ns / kHeartbeat.ns + 1;
        next_heartbeat_[i] = next_heartbeat_[i] + runtime_core::Duration{periods * kHeartbeat.ns};
        if (!heartbeat_muted[i]) {
          out.push_back({i, cc::encode_heartbeat({node_id(i), drives[i]->nmt().state()})});
        }
      }
    }
    return out;
  }

  // A frame the master sent, to every drive (or its intercept); returns what they send back.
  std::vector<DriveFrame> deliver(const cc::Frame& frame, runtime_core::MonoTime now) {
    std::vector<DriveFrame> out;
    for (std::size_t i = 0; i < kBusNodes; ++i) {
      if (intercept[i]) {
        if (const std::optional<cc::Frame> reply = intercept[i](frame)) {
          out.push_back({i, *reply});
          continue;
        }
      }
      from_drive(i, drives[i]->on_frame(frame, now), now, out);
    }
    return out;
  }

  // A frame straight to one drive, outside the bus: its replies go to the caller and its events
  // are recorded without an EMCY (a test setting the drive up before the master runs).
  std::vector<cc::Frame> to_drive(std::size_t i, const cc::Frame& frame,
                                  runtime_core::MonoTime now) {
    std::vector<cc::Frame> out = drives[i]->on_frame(frame, now);
    for (const DriveRecord& r : pending_events_) {
      drive_events.push_back(r);
    }
    pending_events_.clear();
    return out;
  }

  std::vector<DriveFrame> inject_fault(std::size_t i, std::uint16_t code,
                                       runtime_core::MonoTime now) {
    drives[i]->inject_fault(code, now);
    std::vector<DriveFrame> out;
    flush_events(out);
    return out;
  }
  std::vector<DriveFrame> inject_torque_loss(std::size_t i, runtime_core::MonoTime now) {
    drives[i]->inject_torque_loss(now);
    std::vector<DriveFrame> out;
    flush_events(out);
    return out;
  }

  // The shared order counter, for frames a bus records alongside the drive events.
  std::uint64_t next_order() { return ++order_; }

  std::array<std::unique_ptr<drive_core::Drive>, kBusNodes> drives;
  std::vector<DriveRecord> drive_events;
  std::array<Intercept, kBusNodes> intercept;
  // Periodic heartbeats of a muted node are not sent (boot-ups still are).
  std::array<bool, kBusNodes> heartbeat_muted{};

 private:
  void from_drive(std::size_t i, const std::vector<cc::Frame>& frames, runtime_core::MonoTime now,
                  std::vector<DriveFrame>& out) {
    for (const cc::Frame& f : frames) {
      if (f.cob_id == 0x700U + node_id(i).value() && f.dlc == 1 && f.data[0] == 0) {
        powered_[i] = true;
        next_heartbeat_[i] = now + kHeartbeat;  // A boot-up restarts the heartbeat phase.
      }
      out.push_back({i, f});
    }
    flush_events(out);
  }

  // Records the drives' events; an accepted fault sends its EMCY (Pre-op and Operational).
  void flush_events(std::vector<DriveFrame>& out) {
    std::vector<DriveRecord> events;
    events.swap(pending_events_);
    for (const DriveRecord& r : events) {
      drive_events.push_back(r);
      const cc::NmtState s = drives[r.node]->nmt().state();
      if (r.event.kind == drive_core::DriveEventKind::kFault &&
          (s == cc::NmtState::kPreOperational || s == cc::NmtState::kOperational)) {
        out.push_back(
            {r.node, cc::encode_emcy(
                         {node_id(r.node), r.event.fault_code, static_cast<std::uint8_t>(1), {}})});
      }
    }
  }

  std::vector<DriveRecord> pending_events_;
  std::array<runtime_core::MonoTime, kBusNodes> next_heartbeat_{};
  std::array<bool, kBusNodes> powered_{};
  std::uint64_t order_{0};
};

// ---- scripted peers ----

// Node `i` answers an SDO upload of index:sub (any sub when std::nullopt) with `answer`.
inline Intercept upload_answer(std::size_t i, std::uint16_t index, std::optional<std::uint8_t> sub,
                               std::function<cc::Sdo(const cc::Sdo& request)> answer) {
  return [i, index, sub,
          answer = std::move(answer)](const cc::Frame& f) -> std::optional<cc::Frame> {
    const std::optional<cc::Sdo> s = cc::decode_sdo(node_id(i), f);
    if (f.cob_id != 0x600U + node_id(i).value() || !s || s->type != cc::SdoType::kUploadRequest ||
        s->address.index != index || (sub && s->address.sub != *sub)) {
      return std::nullopt;
    }
    return cc::encode_sdo(node_id(i), answer(*s));
  };
}

// Node `i` has no object `index`: its uploads end in abort 0x06020000.
inline Intercept without_object(std::size_t i, std::uint16_t index) {
  return upload_answer(i, index, std::nullopt, [](const cc::Sdo& request) {
    cc::Sdo abort{};
    abort.type = cc::SdoType::kAbortToClient;
    abort.address = request.address;
    abort.abort_code = 0x0602'0000;
    return abort;
  });
}

// Node `i` reports `value` (`size` bytes) for index:sub.
inline Intercept reports(std::size_t i, std::uint16_t index, std::uint8_t sub, std::uint32_t value,
                         std::uint8_t size = 4) {
  return upload_answer(i, index, sub, [value, size](const cc::Sdo& request) {
    cc::Sdo answer{};
    answer.type = cc::SdoType::kUploadResponse;
    answer.address = request.address;
    answer.size = size;
    answer.value = value;
    return answer;
  });
}

// A rewrite for node `i`'s TPDO3 that never reports an applied tag (applied_tag = 0): a peer
// that commits like a vdrive but never shows the armed tag. std::nullopt keeps the frame.
inline std::optional<cc::Frame> without_applied_tag(std::size_t i, const cc::Frame& f) {
  if (f.cob_id != 0x380U + node_id(i).value()) {
    return std::nullopt;
  }
  std::optional<cc::od::Tpdo3> t = cc::unpack_tpdo3(node_id(i), f);
  if (!t) {
    return std::nullopt;
  }
  t->applied_tag = 0;
  return cc::pack_tpdo3(node_id(i), *t);
}

}  // namespace canopen_master::testing
