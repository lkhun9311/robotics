// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/time.hpp"

#include <cerrno>
#include <ctime>
#include <stdexcept>
#include <system_error>

namespace runtime_core {

MonoTime SystemMonotonicClock::now() const {
  timespec ts{};
  // CLOCK_MONOTONIC does not fail on Linux in practice, but a zeroed timespec would read as
  // time 0 and make every deadline look expired or far away. Fail loudly instead.
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    throw std::system_error(errno, std::generic_category(), "clock_gettime(CLOCK_MONOTONIC)");
  }
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
