// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "runtime_core/time.hpp"

namespace runtime_core {

// A validity window is open while now < valid_until. At now == valid_until it is closed.
constexpr bool is_expired(MonoTime now, MonoTime valid_until) { return now >= valid_until; }

// A command may never outlive the permit it was sent under.
constexpr MonoTime clamp_expiry(MonoTime requested, MonoTime permit_expires_at) {
  return requested < permit_expires_at ? requested : permit_expires_at;
}

}  // namespace runtime_core
