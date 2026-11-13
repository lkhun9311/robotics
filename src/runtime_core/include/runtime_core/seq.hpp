// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace runtime_core {

// Serial-number comparison for 16-bit wrapping sequence numbers (RFC 1982 style).
// True when `a` is strictly newer than `b`. Values exactly half the range apart are
// ambiguous; both directions answer "not newer" so an ambiguous frame is dropped.
constexpr bool seq16_newer(std::uint16_t a, std::uint16_t b) {
  const auto diff = static_cast<std::uint16_t>(a - b);
  return diff != 0 && diff < 0x8000;
}

}  // namespace runtime_core
