// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "runtime_core/restart_budget.hpp"

namespace rc = runtime_core;

namespace {
rc::MonoTime sec(std::int64_t s) { return rc::MonoTime{s * 1'000'000'000}; }
}  // namespace

TEST(RestartBudget, AllowsUpToMaxInsideWindow) {
  rc::RestartBudget b{3, rc::milliseconds(10'000)};
  EXPECT_TRUE(b.try_consume(sec(0)));
  EXPECT_TRUE(b.try_consume(sec(1)));
  EXPECT_TRUE(b.try_consume(sec(2)));
  EXPECT_FALSE(b.try_consume(sec(3)));
  EXPECT_EQ(b.used(sec(3)), 3U);
}

TEST(RestartBudget, WindowSlides) {
  rc::RestartBudget b{3, rc::milliseconds(10'000)};
  b.try_consume(sec(0));
  b.try_consume(sec(1));
  b.try_consume(sec(2));
  EXPECT_FALSE(b.try_consume(sec(9)));
  EXPECT_TRUE(b.try_consume(sec(10)));
  EXPECT_EQ(b.used(sec(10)), 3U);
}

TEST(RestartBudget, RefusedAttemptsAreNotRecorded) {
  rc::RestartBudget b{1, rc::milliseconds(10'000)};
  EXPECT_TRUE(b.try_consume(sec(0)));
  EXPECT_FALSE(b.try_consume(sec(5)));
  EXPECT_TRUE(b.try_consume(sec(10)));
}
