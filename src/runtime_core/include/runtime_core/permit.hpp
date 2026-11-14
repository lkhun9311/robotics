// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "runtime_core/ids.hpp"
#include "runtime_core/time.hpp"

namespace runtime_core {

// Project-specific upper motion authority (spec §6.1). Not an industry-standard lease.
struct MotionPermit {
  ManagerInstance manager{};
  SessionId session{};
  Generation generation{};
  MonoTime expires_at{};
};

}  // namespace runtime_core
