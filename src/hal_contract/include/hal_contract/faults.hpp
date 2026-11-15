// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace hal_contract {

// Faults a backend reports in Feedback. Each is one bit, so a set travels as one uint32 (the
// ros2_control `hal/faults` state interface). A policy refusal of a write is never a fault: it
// is a HalVerdict, and it changes nothing here (spec §8.2).
enum class Fault : std::uint32_t {
  kTransportLost = 1U << 0,    // the bus or socket is gone
  kNodeLost = 1U << 1,         // a node's heartbeat stopped
  kBootTimeout = 1U << 2,      // a node never booted
  kDriveFault = 1U << 3,       // a drive reports Fault, Fault Reaction Active or a non-zero EMCY
  kEventOverflow = 1U << 4,    // the worker's event queue was full
  kStopUnconfirmed = 1U << 5,  // at least one node's stop was not confirmed (C18)
  kPolicyStop = 1U << 6,       // a device stopped on its own: expiry, command timeout, hold end
  kInhibited = 1U << 7,        // the group motion inhibit is latched (C17)
  kArmFailed = 1U << 8,        // an arm op failed or timed out
};

const char* to_string(Fault fault);

class FaultSet {
 public:
  constexpr FaultSet() = default;
  constexpr explicit FaultSet(std::uint32_t bits) : bits_(bits) {}

  constexpr void set(Fault f) { bits_ |= static_cast<std::uint32_t>(f); }
  constexpr bool has(Fault f) const { return (bits_ & static_cast<std::uint32_t>(f)) != 0U; }
  constexpr bool any() const { return bits_ != 0U; }
  constexpr std::uint32_t bits() const { return bits_; }

  constexpr bool operator==(const FaultSet&) const = default;

 private:
  std::uint32_t bits_{0};
};

}  // namespace hal_contract
