// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/lease.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace runtime_core {

std::optional<Duration> max_lease(double v_max, double a_stop, double e_allow,
                                  Duration guard_period) {
  if (!std::isfinite(v_max) || !std::isfinite(a_stop) || !std::isfinite(e_allow) || v_max <= 0.0 ||
      a_stop <= 0.0 || e_allow <= 0.0 || guard_period.ns < 0) {
    return std::nullopt;
  }
  const double stop_distance = v_max * v_max / (2.0 * a_stop);
  const double budget_ns_real = (e_allow - stop_distance) / v_max * 1e9;
  // Intermediate results can overflow even with finite inputs; never cast an out-of-range
  // double to int64 (undefined behaviour).
  if (!std::isfinite(budget_ns_real) || budget_ns_real <= 0.0 ||
      budget_ns_real >= static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  const auto budget_ns = static_cast<std::int64_t>(std::floor(budget_ns_real));
  const std::int64_t lease_ns = budget_ns - guard_period.ns;
  if (lease_ns <= 0) return std::nullopt;
  return Duration{lease_ns};
}

Duration floor_to_period(Duration d, Duration period) {
  if (period.ns <= 0) throw std::invalid_argument("floor_to_period: period must be positive");
  return Duration{d.ns - d.ns % period.ns};
}

}  // namespace runtime_core
