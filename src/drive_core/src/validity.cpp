// SPDX-License-Identifier: Apache-2.0
#include "drive_core/validity.hpp"

namespace drive_core {

bool validity_open(std::uint32_t until_us, std::uint32_t now_us, std::uint32_t lease_us) {
  if (lease_us >= 0x8000'0000U) {
    return false;
  }
  const auto ahead = static_cast<std::uint32_t>(until_us - now_us);
  return ahead != 0 && ahead <= lease_us;
}

std::uint32_t to_wire_us(runtime_core::MonoTime t) {
  // C++ division truncates toward zero; step down once for a negative remainder to get floor.
  std::int64_t us = t.ns / 1000;
  if (t.ns % 1000 < 0) {
    --us;
  }
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(us));
}

}  // namespace drive_core
