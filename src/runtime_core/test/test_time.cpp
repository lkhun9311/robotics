// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

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
