// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

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
// configure while armed and not inhibited is refused (the op fails at once, nothing is sent):
// its Enter Pre-operational would stop enabled drives without the stop being tracked. Deactivate
// first, or let an inhibit stop the group.
//
// Not thread-safe; one caller serializes every call. In logic time the caller is the test, which
// calls step() every W; in E3c Task 6 it is the I/O worker.
namespace canopen_backend {

struct CanopenConfig {
  // Node IDs and the C20 timing profile; outside C20 the constructor throws with the allowed
  // values in the message.
  canopen_master::SessionConfig session{};
  runtime_core::Duration heartbeat_timeout{canopen_master::kDefaultHeartbeatTimeout};
  runtime_core::Duration boot_timeout{canopen_master::kDefaultBootTimeout};
  runtime_core::Duration sdo_timeout{canopen_master::kDefaultSdoTimeout};
  // C2 backend_profile_limit: the reported profile is the weaker, per capability, of the probed
  // one and this. std::nullopt sets no limit.
  std::optional<runtime_core::DeviceCapabilities> profile_limit{};
};

// The weaker of two supports: kUnsupported < kUnknown < kNotApplicable < kProvidedByHost <
// kSupported.
runtime_core::Support weaker(runtime_core::Support a, runtime_core::Support b);
runtime_core::DeviceCapabilities weaker(const runtime_core::DeviceCapabilities& a,
                                        const runtime_core::DeviceCapabilities& b);

class CanopenBackend final : public hal_contract::Backend {
 public:
  // Throws std::invalid_argument for a null transport or a configuration the session refuses.
  CanopenBackend(CanopenConfig config, std::unique_ptr<Transport> transport);
  CanopenBackend(const CanopenBackend&) = delete;
  CanopenBackend& operator=(const CanopenBackend&) = delete;

  // Time passes to `now`: every received frame to the session, then the session's tick, then
  // what arrived in reply, until nothing more arrives (at most kMaxFramesPerStep frames).
  void step(runtime_core::MonoTime now);

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
  bool inhibited() const override { return session_.inhibited(); }
  bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const override {
    return session_.can_apply(valid_until, now);
  }

  // ---- observation (tests, traces) ----
  const canopen_master::VdriveSession& session() const { return session_; }
  const canopen_master::Master& master() const { return master_; }
  // The newest arm request posted, for observers that join device commits to an identity.
  std::optional<hal_contract::ArmRequest> last_arm() const { return last_arm_; }

  static constexpr std::size_t kMaxFramesPerStep = 4096;
  static constexpr std::size_t kRefusalRing = 8;

 private:
  struct Refusal {
    std::uint64_t op_id{0};
    const char* error{nullptr};
  };

  void send_all(const std::vector<canopen_codec::Frame>& frames);
  const Refusal* refusal(std::uint64_t op_id) const;

  CanopenConfig config_;
  std::unique_ptr<Transport> transport_;
  canopen_master::Master master_;
  canopen_master::VdriveSession session_;
  runtime_core::MonoTime now_{};
  std::optional<hal_contract::ArmRequest> last_arm_{};
  std::array<Refusal, kRefusalRing> refusals_{};
  std::size_t refusals_next_{0};
};

}  // namespace canopen_backend
