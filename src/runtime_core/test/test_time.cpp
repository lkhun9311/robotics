// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "runtime_core/time.hpp"

namespace rc = runtime_core;

TEST(Time, ArithmeticIsIntegerNanoseconds) {
  const rc::MonoTime t{1'000};
  EXPECT_EQ((t + rc::milliseconds(2)).ns, 2'001'000);
  EXPECT_EQ((rc::MonoTime{5'000} - rc::microseconds(2)).ns, 3'000);
  EXPECT_EQ((rc::MonoTime{5'000} - rc::MonoTime{2'000}).ns, 3'000);
  EXPECT_EQ(rc::microseconds(7).ns, 7'000);
}

TEST(FakeClock, AdvancesForwardOnly) {
  rc::FakeClock clock{rc::MonoTime{10}};
  clock.advance(rc::Duration{5});
  EXPECT_EQ(clock.now().ns, 15);
  EXPECT_THROW(clock.advance(rc::Duration{-1}), std::invalid_argument);
  EXPECT_EQ(clock.now().ns, 15);
}

TEST(SystemMonotonicClock, NeverGoesBackwards) {
  const rc::SystemMonotonicClock clock;
  const rc::MonoTime a = clock.now();
  const rc::MonoTime b = clock.now();
  EXPECT_LE(a, b);
}

TEST(Time, ArithmeticRejectsOverflowInsteadOfWrapping) {
  constexpr auto kMax = std::numeric_limits<std::int64_t>::max();
  constexpr auto kMin = std::numeric_limits<std::int64_t>::min();
  EXPECT_THROW(rc::MonoTime{kMax} + rc::Duration{1}, std::overflow_error);
  EXPECT_THROW(rc::MonoTime{kMin} - rc::Duration{1}, std::overflow_error);
  EXPECT_THROW(rc::MonoTime{kMax} - rc::MonoTime{-1}, std::overflow_error);
  EXPECT_THROW(rc::milliseconds(kMax / 1'000'000 + 1), std::overflow_error);
  EXPECT_THROW(rc::microseconds(kMin / 1'000 - 1), std::overflow_error);
  EXPECT_EQ((rc::MonoTime{kMax - 1} + rc::Duration{1}).ns, kMax);
}
