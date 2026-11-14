// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "runtime_core/lease.hpp"

namespace rc = runtime_core;

TEST(Lease, DerivesFromVelocityStopAndAllowedError) {
  // stop distance 1^2/(2*10) = 0.05 rad, budget (0.1-0.05)/1 = 50 ms, minus W = 2 ms.
  const auto lease = rc::max_lease(1.0, 10.0, 0.1, rc::milliseconds(2));
  ASSERT_TRUE(lease.has_value());
  EXPECT_GE(lease->ns, 47'999'000);
  EXPECT_LE(lease->ns, 48'000'000);
}

TEST(Lease, NoLeaseWhenStoppingAloneUsesTheBudget) {
  EXPECT_FALSE(rc::max_lease(1.0, 10.0, 0.05, rc::milliseconds(2)).has_value());
  EXPECT_FALSE(rc::max_lease(1.0, 10.0, 0.0519, rc::milliseconds(2)).has_value());
}

TEST(Lease, RejectsNonPositiveOrNonFiniteInputs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rc::max_lease(0.0, 10.0, 0.1, rc::milliseconds(2)).has_value());
  EXPECT_FALSE(rc::max_lease(1.0, -1.0, 0.1, rc::milliseconds(2)).has_value());
  EXPECT_FALSE(rc::max_lease(nan, 10.0, 0.1, rc::milliseconds(2)).has_value());
  EXPECT_FALSE(rc::max_lease(1.0, 10.0, 0.1, rc::Duration{-1}).has_value());
}

TEST(Lease, RejectsBudgetsThatDoNotFitInNanoseconds) {
  // Finite inputs whose budget overflows int64 nanoseconds.
  EXPECT_FALSE(rc::max_lease(1e-300, 1e300, 1e300, rc::milliseconds(2)).has_value());
}

TEST(Lease, FloorToPeriod) {
  EXPECT_EQ(rc::floor_to_period(rc::Duration{47'999'999}, rc::milliseconds(1)).ns, 47'000'000);
  EXPECT_EQ(rc::floor_to_period(rc::milliseconds(48), rc::milliseconds(1)).ns, 48'000'000);
  EXPECT_THROW(rc::floor_to_period(rc::milliseconds(48), rc::Duration{0}), std::invalid_argument);
}
