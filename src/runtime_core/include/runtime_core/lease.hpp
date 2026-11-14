// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>

#include "runtime_core/time.hpp"

namespace runtime_core {

// Largest lease such that a joint moving at v_max, unseen for (lease + guard_period),
// then stopping at a_stop, stays within e_allow. nullopt when the stop alone uses the
// whole budget or an input is not positive and finite (spec §10.1).
std::optional<Duration> max_lease(double v_max, double a_stop, double e_allow,
                                  Duration guard_period);

// Rounds down to a whole number of periods so expiry lands on a tick boundary.
Duration floor_to_period(Duration d, Duration period);

}  // namespace runtime_core
