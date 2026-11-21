// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "runtime_core/time.hpp"

// The heartbeat consumer of one node (C7).
// It only observes: whatever it detects is reported to the caller, and nothing here sends a frame.
// A window is open while now < deadline; at the deadline it has expired.
namespace canopen_master {

enum class HeartbeatStatus : std::uint8_t {
  kDisabled,      // Consumer time 0.
  kIdle,          // Enabled, not started yet.
  kAwaitingBoot,  // Started or restarted; waiting for the first heartbeat until the boot deadline.
  kBootTimedOut,  // The boot deadline passed without a heartbeat.
  kAlive,         // Heartbeats arrive within the consumer time.
  kLost,          // The consumer time passed after the last heartbeat.
};

enum class HeartbeatChange : std::uint8_t {
  kNone,
  kFirst,        // First heartbeat after a start: kAwaitingBoot -> kAlive.
  kResumed,      // kLost or kBootTimedOut -> kAlive.
  kBootTimeout,  // kAwaitingBoot -> kBootTimedOut.
  kLost,         // kAlive -> kLost.
};

class HeartbeatConsumer {
 public:
  // A consumer time of zero or less disables the consumer.
  HeartbeatConsumer(runtime_core::Duration consumer_time, runtime_core::Duration boot_timeout);

  // Starts (or restarts, after an NMT reset sent by the master) the wait for a first heartbeat:
  // the boot deadline is now + boot_timeout.
  // No effect while disabled.
  void start(runtime_core::MonoTime now);

  // Zero or less disables.
  // Enabling a disabled consumer starts it at `now`; changing the time of an enabled one moves the
  // running deadline to last heartbeat + the new time.
  void set_consumer_time(runtime_core::Duration consumer_time, runtime_core::MonoTime now);

  // A heartbeat (any state byte, boot-up included) received at `now`.
  // Call expire(now) first.
  HeartbeatChange on_heartbeat(runtime_core::MonoTime now);

  // Reports a deadline that has passed at `now`, once.
  HeartbeatChange expire(runtime_core::MonoTime now);

  HeartbeatStatus status() const { return status_; }
  runtime_core::Duration consumer_time() const { return consumer_time_; }

 private:
  runtime_core::Duration consumer_time_;
  runtime_core::Duration boot_timeout_;
  HeartbeatStatus status_;
  runtime_core::MonoTime deadline_{};
  runtime_core::MonoTime last_heartbeat_{};
};

}  // namespace canopen_master
