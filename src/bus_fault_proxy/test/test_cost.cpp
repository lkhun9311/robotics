// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "bus_fault_proxy/cost.hpp"

using namespace bus_fault_proxy;

TEST(CostDistribution, EmptySamplesGiveNoBins) { EXPECT_TRUE(run_lengths({}).empty()); }

TEST(CostDistribution, EachDistinctValueOnceWithItsCountAscending) {
  const std::vector<ValueCount> d = run_lengths({5, 5, 7, 9, 9, 9});
  ASSERT_EQ(d.size(), 3U);
  EXPECT_EQ(d[0].value, 5);
  EXPECT_EQ(d[0].count, 2U);
  EXPECT_EQ(d[1].value, 7);
  EXPECT_EQ(d[1].count, 1U);
  EXPECT_EQ(d[2].value, 9);
  EXPECT_EQ(d[2].count, 3U);
}

TEST(CostDistribution, CountsSumToTheSamples) {
  std::vector<std::int64_t> sorted;
  for (std::int64_t v = 1000; v < 1100; v += 10) {
    for (std::int64_t k = 0; k <= v / 100; ++k) {
      sorted.push_back(v);
    }
  }
  std::uint64_t total = 0;
  for (const ValueCount& b : run_lengths(sorted)) {
    total += b.count;
  }
  EXPECT_EQ(total, sorted.size());
}
