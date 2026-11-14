// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <compare>
#include <cstdint>

namespace runtime_core {

struct SessionId {
  std::uint64_t value{0};
  auto operator<=>(const SessionId&) const = default;
};

struct Generation {
  std::uint64_t value{0};
  auto operator<=>(const Generation&) const = default;
};

struct ManagerInstance {
  std::uint64_t value{0};
  auto operator<=>(const ManagerInstance&) const = default;
};

// Strictly increasing, never-zero identifiers. Generations are global across sessions,
// so an old execution can never share a generation with a new one.
class IdAllocator {
 public:
  SessionId next_session() { return SessionId{++session_}; }
  Generation next_generation() { return Generation{++generation_}; }

 private:
  std::uint64_t session_{0};
  std::uint64_t generation_{0};
};

}  // namespace runtime_core
