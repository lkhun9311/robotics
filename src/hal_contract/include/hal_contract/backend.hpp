// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

namespace hal_contract {

// What a device side must offer the Hal (C1).
// Two implementations follow: the in-process DirectBackend and the CANopen backend; both run the
// same contract suite (hal_contract/testing/contract_suite.hpp).
//
// Lifecycle ops (configure, activate, arm, deactivate) are posted, never waited for here.
// The Hal allocates each op's id and absolute deadline and passes them in an OpTicket, so an
// activation refused by the capability gate and a backend op share one id space; the backend
// reports progress through op_state() and must fail an op whose deadline has passed (C10).
//
// Control-path calls (submit, poll, inhibited, can_apply, op_state) never block and never
// allocate once active.
class Backend {
 public:
  virtual ~Backend() = default;

  // Brings the device to a state where capabilities() is valid.
  // Clears the group inhibit (C17: only a new configure clears it).
  virtual void configure(const OpTicket& ticket, runtime_core::MonoTime now) = 0;
  // Valid after configure succeeded.
  virtual runtime_core::DeviceCapabilities capabilities() const = 0;
  // To Switched On with no execution tag; nothing moves.
  // The Hal calls it only after the gate.
  virtual void activate(const OpTicket& ticket, runtime_core::MonoTime now) = 0;
  // Group-wide: disable if enabled, commit `request.id`, hold the actual position until
  // `request.hold_until`, enable (C9).
  // Commands submitted before it are discarded.
  virtual void arm(const OpTicket& ticket, const ArmRequest& request,
                   runtime_core::MonoTime now) = 0;
  // A command HalGuard approved.
  // The newest one wins; one overwritten before dispatch is counted as superseded, never
  // acknowledged.
  virtual void submit(const ApprovedCommand& command, runtime_core::MonoTime now) = 0;
  virtual Feedback poll(runtime_core::MonoTime now) = 0;
  // Stop every enabled axis and confirm each stop (C18), then leave the device Switched On.
  virtual void deactivate(const OpTicket& ticket, runtime_core::MonoTime now) = 0;
  // Latches the inhibit and starts the group stop without waiting for it.
  virtual void on_error_nonblocking() = 0;

  virtual OpState op_state(std::uint64_t op_id) const = 0;
  // Blocks up to `timeout` for a lifecycle op. nullopt means still pending after `timeout`; a
  // backend whose ops run in a worker fails the op when the wait times out (C10).
  // A backend stepped in logic time returns at once.
  virtual std::optional<OpResult> wait(std::uint64_t op_id, runtime_core::Duration timeout) = 0;

  // The group motion inhibit (C17), read on every write so a latch takes effect at once.
  virtual bool inhibited() const = 0;
  // False when a command closing at `valid_until` could no longer be applied by the device
  // (C6: on CANopen, strictly later than the next SYNC + W + 1 ms).
  virtual bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const = 0;
};

}  // namespace hal_contract
