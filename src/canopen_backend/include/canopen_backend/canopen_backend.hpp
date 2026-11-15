// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "canopen_backend/transport.hpp"
#include "canopen_master/master.hpp"
#include "canopen_master/vdrive_session.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

// The CANopen backend (C1, C2, C9): the vdrive session of canopen_master behind the
// hal_contract Backend interface, over a frame Transport. Lifecycle ops are posted to the session
// and progress as step() runs; the control-path calls (submit, poll, inhibited, can_apply,
// op_state) only read or write the session's fixed state and never touch the transport.
//
// Capabilities are probed by configure (C2): six vdrive nodes give the virtual CANopen profile,
// provisional until the first arm confirms it by behaviour; anything else gives the generic
// CiA 402 profile. A configured profile limit (the backend_profile_limit parameter) can only
// lower the probed profile, per capability.
//
// configure is refused (the op fails at once, nothing is sent) while an arm is running, the group
// is armed or a deactivate is running, unless the inhibit is latched: any of those may have
// drives Operation Enabled, and the configure's Enter Pre-operational would stop them without the
// stop being tracked (a configure posted mid-arm would also supersede the arm). Deactivate first,
// or let an inhibit stop the group.
//
// A configuration outside the C20 timing profile (or one the master or session refuses: duplicate
// or unknown nodes, non-positive timeouts) does not throw: the backend is built without a session
// and every lifecycle op fails at once with the reason, which for C20 lists the allowed values.
//
// Not thread-safe; one caller serializes every call. In logic time the caller is the test, which
// calls step() every W; in E3c Task 6 it is the I/O worker.
namespace canopen_backend {

struct CanopenConfig {
  // Node IDs and the C20 timing profile; outside C20 every op fails with the allowed values.
  canopen_master::SessionConfig session{};
  runtime_core::Duration heartbeat_timeout{canopen_master::kDefaultHeartbeatTimeout};
  runtime_core::Duration boot_timeout{canopen_master::kDefaultBootTimeout};
  runtime_core::Duration sdo_timeout{canopen_master::kDefaultSdoTimeout};
  // C2 backend_profile_limit: the reported profile is the weaker, per capability, of the probed
  // one and this. std::nullopt sets no limit.
  std::optional<runtime_core::DeviceCapabilities> profile_limit{};
};

// The weaker of two supports: kUnsupported < kUnknown < kNotApplicable < kProvidedByHost <
// kSupported. kUnknown ranks below kNotApplicable: an unverified guarantee must not count for more
// than a stated "meaningless here", so a limit or a node that is unknown always lowers the result.
runtime_core::Support weaker(runtime_core::Support a, runtime_core::Support b);
runtime_core::DeviceCapabilities weaker(const runtime_core::DeviceCapabilities& a,
                                        const runtime_core::DeviceCapabilities& b);

class CanopenBackend final : public hal_contract::Backend {
 public:
  // Throws std::invalid_argument for a null transport only; a refused configuration is reported
  // by configure() (see above).
  CanopenBackend(CanopenConfig config, std::unique_ptr<Transport> transport);
  CanopenBackend(const CanopenBackend&) = delete;
  CanopenBackend& operator=(const CanopenBackend&) = delete;

  // Time passes to `now`: every received frame to the session, then the session's tick, then
  // what arrived in reply (the TPDOs answering this tick's SYNC included), until nothing more
  // arrives. `max_frames` (kMaxFramesPerStep by default; the I/O worker passes its per-wake
  // budget, B1) is a soft bound over both drains, checked between poll() batches: a step stops
  // polling once that many frames were handled, but a batch already taken is handled whole; the
  // tick runs either way. Returns the number of frames handled.
  std::size_t step(runtime_core::MonoTime now, std::size_t max_frames = kMaxFramesPerStep);

  // ---- hal_contract::Backend ----
  void configure(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  runtime_core::DeviceCapabilities capabilities() const override;
  void activate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  void arm(const hal_contract::OpTicket& ticket, const hal_contract::ArmRequest& request,
           runtime_core::MonoTime now) override;
  void submit(const hal_contract::ApprovedCommand& command, runtime_core::MonoTime now) override;
  hal_contract::Feedback poll(runtime_core::MonoTime now) override;
  void deactivate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  // Latches the inhibit at the time of the last step; the stop frames leave with the next step.
  void on_error_nonblocking() override;
  hal_contract::OpState op_state(std::uint64_t op_id) const override;
  // Logic time: returns at once (std::nullopt while the op is pending).
  std::optional<hal_contract::OpResult> wait(std::uint64_t op_id,
                                             runtime_core::Duration timeout) override;
  // Without a session (a refused configuration) the backend reports itself inhibited.
  bool inhibited() const override { return !session_ || session_->inhibited(); }
  bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const override {
    return session_ && session_->can_apply(valid_until, now);
  }

  // ---- for the I/O worker (the one thread that steps the backend) ----
  // C10: fails a pending op now (the caller's wait timed out); see VdriveSession::cancel.
  void cancel(std::uint64_t op_id, const char* why, runtime_core::MonoTime now);
  // Why the op failed: a refusal's reason or the session's (storage that lives as long as the
  // backend); nullptr unless it failed.
  const char* op_error(std::uint64_t op_id) const;
  // Every master event the session handles (VdriveSession::set_event_observer); nothing for a
  // backend built from a refused configuration.
  void set_event_observer(canopen_master::VdriveSession::EventObserver observer);
  // The master's last SYNC; std::nullopt before the first or without a session.
  std::optional<runtime_core::MonoTime> last_sync() const {
    return session_ ? session_->last_sync() : std::nullopt;
  }
  runtime_core::Duration bus_period() const { return config_.session.bus_period; }

  // ---- observation (tests, traces) ----
  // Throw std::logic_error for a backend built from a refused configuration.
  const canopen_master::VdriveSession& session() const;
  const canopen_master::Master& master() const;
  // Why the configuration was refused; empty when it was accepted.
  const std::string& config_error() const { return config_error_; }
  // The newest arm request posted, for observers that join device commits to an identity.
  std::optional<hal_contract::ArmRequest> last_arm() const { return last_arm_; }

  static constexpr std::size_t kMaxFramesPerStep = 4096;
  // Refused ops are remembered in a ring this long. An id pushed out by later refusals reads as
  // kUnknown ("unknown operation" from wait), not as its refusal; the Hal keeps fewer ops
  // (Hal::kOpSlots = 8), so it never asks for one.
  static constexpr std::size_t kRefusalRing = 8;

 private:
  struct Refusal {
    std::uint64_t op_id{0};
    const char* error{nullptr};
  };

  void send_all(const std::vector<canopen_codec::Frame>& frames);
  const Refusal* refusal(std::uint64_t op_id) const;
  void refuse(const hal_contract::OpTicket& ticket, const char* error);
  // A configure now could reach drives that are (or may become) Operation Enabled, untracked.
  bool configure_unsafe() const;

  CanopenConfig config_;
  std::unique_ptr<Transport> transport_;
  std::string config_error_;
  std::optional<canopen_master::Master> master_;
  std::optional<canopen_master::VdriveSession> session_;
  runtime_core::MonoTime now_{};
  std::optional<hal_contract::ArmRequest> last_arm_{};
  std::array<Refusal, kRefusalRing> refusals_{};
  std::size_t refusals_next_{0};
};

}  // namespace canopen_backend
