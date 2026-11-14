// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>

#include "drive_core/validity.hpp"

namespace {

using drive_core::kLeaseUs;
using drive_core::to_wire_us;
using drive_core::validity_open;
using runtime_core::MonoTime;

TEST(Validity, WrapsAround) {
  const std::uint32_t now = 0xFFFF'FF00U;
  const auto until = static_cast<std::uint32_t>(now + 1000U);
  ASSERT_LT(until, now);  // The deadline wrapped past 2^32.
  EXPECT_TRUE(validity_open(until, now, kLeaseUs));
  // A deadline just behind `now` across the wrap is expired, not 2^32 - 16 µs ahead.
  EXPECT_FALSE(validity_open(0xFFFF'FFF0U, 0x10U, kLeaseUs));
  EXPECT_TRUE(validity_open(0x10U, 0xFFFF'FFF0U, kLeaseUs));
}

TEST(Validity, ExpiredAtExactBoundary) {
  for (const std::uint32_t now : {0U, 1U, 123'456U, 0x7FFF'FFFFU, 0xFFFF'FFFFU}) {
    EXPECT_FALSE(validity_open(now, now, kLeaseUs)) << now;
    EXPECT_TRUE(validity_open(now + 1U, now, kLeaseUs)) << now;
    EXPECT_FALSE(validity_open(now - 1U, now, kLeaseUs)) << now;
  }
}

TEST(Validity, FartherThanLeaseIsRejected) {
  EXPECT_EQ(kLeaseUs, 48'000U);
  for (const std::uint32_t now : {0U, 5'000'000U, 0xFFFF'0000U, 0xFFFF'FFFFU}) {
    EXPECT_TRUE(validity_open(now + kLeaseUs, now, kLeaseUs)) << now;
    EXPECT_FALSE(validity_open(now + kLeaseUs + 1U, now, kLeaseUs)) << now;
    EXPECT_FALSE(validity_open(now + 1'000'000U, now, kLeaseUs)) << now;
  }
  // The lease is a parameter, not a hidden constant.
  EXPECT_TRUE(validity_open(1'000U + 10'000U, 1'000U, 10'000U));
  EXPECT_FALSE(validity_open(1'000U + 10'001U, 1'000U, 10'000U));
}

TEST(Validity, HalfRangeIsRejected) {
  const std::uint32_t now = 77'777U;
  EXPECT_FALSE(validity_open(now + 0x8000'0000U, now, kLeaseUs));
  EXPECT_FALSE(validity_open(now + 0x7FFF'FFFFU, now, kLeaseUs));
  EXPECT_FALSE(validity_open(now + 0x8000'0001U, now, kLeaseUs));
  // A lease of half the range or more would make ahead and behind ambiguous: nothing opens.
  EXPECT_FALSE(validity_open(now + 1U, now, 0x8000'0000U));
  EXPECT_FALSE(validity_open(now + 1U, now, 0xFFFF'FFFFU));
  EXPECT_TRUE(validity_open(now + 0x7FFF'FFFFU, now, 0x7FFF'FFFFU));
}

TEST(Validity, ToWireNeverLater) {
  constexpr std::int64_t kWrapNs = (std::int64_t{1} << 32) * 1000;  // 2^32 µs in ns.
  const auto check = [](std::int64_t ns) {
    const std::uint32_t wire = to_wire_us(MonoTime{ns});
    // Floor division, also for negative values.
    std::int64_t floor_us = ns / 1000;
    if (ns % 1000 < 0) {
      --floor_us;
    }
    ASSERT_EQ(wire, static_cast<std::uint32_t>(static_cast<std::uint64_t>(floor_us))) << ns;
    // Never later: the encoded microsecond starts at most 999 ns before `ns`, never after it.
    const std::int64_t behind = ns - floor_us * 1000;
    ASSERT_GE(behind, 0) << ns;
    ASSERT_LT(behind, 1000) << ns;
  };
  for (std::int64_t ns = -5'000; ns <= 5'000; ++ns) {
    check(ns);
  }
  for (std::int64_t ns = kWrapNs - 5'000; ns <= kWrapNs + 5'000; ++ns) {
    check(ns);
  }
  for (std::int64_t ns = 0; ns < 40 * kWrapNs; ns += 999'999'937) {
    check(ns);
  }
  check(std::int64_t{0x7FFF'FFFF'FFFF'FFFF});
  check(-std::int64_t{0x7FFF'FFFF'FFFF'FFFF} + 999);
  EXPECT_EQ(to_wire_us(MonoTime{1'999}), 1U);
  EXPECT_EQ(to_wire_us(MonoTime{-1}), 0xFFFF'FFFFU);
  EXPECT_EQ(to_wire_us(MonoTime{kWrapNs + 2'500}), 2U);
}

}  // namespace
