// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "runtime_core/time.hpp"

// Wire validity of a command (contract D5). valid_until_us is microseconds of the shared
// CLOCK_MONOTONIC domain, modulo 2^32.
namespace drive_core {

// The lease bound (Global Constraints: lease <= 48 ms). Callers pass it explicitly; this is
// the default they use.
inline constexpr std::uint32_t kLeaseUs = 48'000;

// True when 0 < (until_us - now_us) mod 2^32 <= lease_us. At equality the window is expired;
// farther ahead than the lease is rejected. A lease_us of 2^31 or more would make "ahead" and
// "behind" ambiguous, so such a lease opens nothing.
bool validity_open(std::uint32_t until_us, std::uint32_t now_us, std::uint32_t lease_us);

// floor(t.ns / 1000) mod 2^32. Floor (also for negative times) keeps the wire time no later
// than the monotonic time it encodes.
std::uint32_t to_wire_us(runtime_core::MonoTime t);

}  // namespace drive_core
