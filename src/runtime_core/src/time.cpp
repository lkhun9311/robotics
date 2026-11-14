// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/time.hpp"

#include <ctime>
#include <stdexcept>

namespace runtime_core {

MonoTime SystemMonotonicClock::now() const {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return MonoTime{static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 +
                  static_cast<std::int64_t>(ts.tv_nsec)};
}

void FakeClock::advance(Duration d) {
  if (d.ns < 0) {
    throw std::invalid_argument("FakeClock::advance: negative duration");
  }
  now_ = now_ + d;
}

}  // namespace runtime_core
