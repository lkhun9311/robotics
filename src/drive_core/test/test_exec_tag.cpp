// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <set>

#include "drive_core/exec_tag.hpp"

namespace {

using drive_core::CommitRefusal;
using drive_core::exec_tag_start;
using drive_core::ExecTagIssuer;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

constexpr MonoTime kBoot{1'000'000'000};
const MonoTime kReady = kBoot + milliseconds(100);

// A boot_id whose first tag is `start`, found by search so the test does not depend on the
// mixing function's internals.
std::optional<std::uint32_t> boot_id_with_start(std::uint16_t start) {
  for (std::uint32_t id = 0; id < (1U << 24); ++id) {
    if (exec_tag_start(id) == start) {
      return id;
    }
  }
  return std::nullopt;
}

TEST(ExecTag, NeverZeroAndStrictlyIncreasing) {
  for (std::uint32_t i = 0; i < 64; ++i) {
    const std::uint32_t boot_id = i * 0x9E37'79B9U;
    ExecTagIssuer issuer(boot_id, kBoot);
    const std::uint16_t start = exec_tag_start(boot_id);
    ASSERT_NE(start, 0);
    std::uint32_t expected = start;
    for (int n = 0; n < 2000; ++n) {
      const auto tag = issuer.commit(1, 1, kReady);
      if (!tag) {
        EXPECT_EQ(expected, 0x10000U) << "refused before the counter reached 0xFFFF";
        break;
      }
      ASSERT_NE(*tag, 0);
      ASSERT_EQ(*tag, expected);
      ASSERT_EQ(issuer.current(), *tag);
      ++expected;
    }
  }
  // One boot exhausted completely: every tag from start to 0xFFFF exactly once, none zero.
  const auto low = boot_id_with_start(3);
  ASSERT_TRUE(low.has_value());
  ExecTagIssuer issuer(*low, kBoot);
  std::uint32_t issued = 0;
  std::uint16_t previous = 0;
  while (const auto tag = issuer.commit(1, 1, kReady)) {
    ASSERT_GT(*tag, previous);
    previous = *tag;
    ++issued;
  }
  EXPECT_EQ(previous, 0xFFFF);
  EXPECT_EQ(issued, 0x10000U - 3U);
}

TEST(ExecTag, PreviousTagDeadAfterCommit) {
  ExecTagIssuer issuer(7, kBoot);
  const auto first = issuer.commit(10, 20, kReady);
  ASSERT_TRUE(first.has_value());
  const auto epoch = issuer.epoch();
  const auto second = issuer.commit(11, 21, kReady);
  ASSERT_TRUE(second.has_value());
  EXPECT_NE(*first, *second);
  EXPECT_EQ(issuer.current(), *second);
  EXPECT_NE(issuer.epoch(), epoch);
  EXPECT_EQ(issuer.generation(), 11U);
  EXPECT_EQ(issuer.session(), 21U);
  issuer.revoke();
  EXPECT_EQ(issuer.current(), 0);
}

TEST(ExecTag, WrapRefusesCommit) {
  const auto boot_id = boot_id_with_start(0xFFFE);
  ASSERT_TRUE(boot_id.has_value());
  ExecTagIssuer issuer(*boot_id, kBoot);
  EXPECT_EQ(issuer.commit(1, 1, kReady), std::optional<std::uint16_t>(0xFFFE));
  EXPECT_EQ(issuer.commit(1, 1, kReady), std::optional<std::uint16_t>(0xFFFF));
  const auto refused = issuer.commit_ex(1, 1, kReady);
  EXPECT_FALSE(refused.tag.has_value());
  EXPECT_EQ(refused.refusal, std::optional<CommitRefusal>(CommitRefusal::kExhausted));
  EXPECT_EQ(issuer.current(), 0) << "a refused commit must not leave the last tag current";
  EXPECT_FALSE(issuer.commit(1, 1, kReady + milliseconds(1)).has_value());
  issuer.reset_communication();
  EXPECT_FALSE(issuer.commit(1, 1, kReady + milliseconds(2)).has_value())
      << "reset communication keeps the allocator, so it stays exhausted";
  // Only a new boot (reset node) issues tags again.
  const MonoTime reboot = kReady + milliseconds(10);
  issuer.boot(*boot_id + 1, reboot);
  EXPECT_TRUE(issuer.commit(1, 1, reboot + milliseconds(100)).has_value());
}

TEST(ExecTag, StartDependsOnBootId) {
  std::set<std::uint16_t> starts;
  for (std::uint32_t id = 0; id < 1000; ++id) {
    const std::uint16_t start = exec_tag_start(id);
    ASSERT_NE(start, 0);
    starts.insert(start);
  }
  EXPECT_GE(starts.size(), 990U) << "consecutive boot ids must spread over the tag range";
  for (const std::uint32_t id : {0U, 1U, 0x2013U, 0xFFFF'FFFFU}) {
    ExecTagIssuer issuer(id, kBoot);
    EXPECT_EQ(issuer.commit(1, 1, kReady), std::optional<std::uint16_t>(exec_tag_start(id)));
    EXPECT_EQ(issuer.boot_id(), id);
  }
  EXPECT_NE(exec_tag_start(1), exec_tag_start(2));
}

TEST(ExecTag, SurvivesResetCommunication) {
  ExecTagIssuer issuer(42, kBoot);
  const auto first = issuer.commit(5, 6, kReady);
  ASSERT_TRUE(first.has_value());
  const auto epoch = issuer.epoch();
  issuer.reset_communication();
  EXPECT_EQ(issuer.current(), 0);
  EXPECT_EQ(issuer.generation(), 0U);
  EXPECT_EQ(issuer.session(), 0U);
  EXPECT_EQ(issuer.boot_id(), 42U);
  EXPECT_NE(issuer.epoch(), epoch);
  // No new quarantine: reset communication is not a new boot, and the allocator continues.
  const auto second = issuer.commit(5, 6, kReady + milliseconds(1));
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(static_cast<std::uint32_t>(*second), static_cast<std::uint32_t>(*first) + 1U);
}

TEST(ExecTag, ExpiresAfterLifetime) {
  EXPECT_EQ(drive_core::kTagLifetime, milliseconds(5 * 60 * 1000));
  ExecTagIssuer issuer(9, kBoot);
  const MonoTime committed = kReady + milliseconds(3);
  const auto tag = issuer.commit(1, 1, committed);
  ASSERT_TRUE(tag.has_value());
  const auto epoch = issuer.epoch();
  issuer.tick(committed + drive_core::kTagLifetime - runtime_core::Duration{1});
  EXPECT_EQ(issuer.current(), *tag);
  EXPECT_EQ(issuer.epoch(), epoch);
  issuer.tick(committed + drive_core::kTagLifetime);
  EXPECT_EQ(issuer.current(), 0) << "expired at exact equality";
  EXPECT_NE(issuer.epoch(), epoch);
  // A re-commit issues a fresh tag with a fresh lifetime.
  const MonoTime later = committed + drive_core::kTagLifetime + milliseconds(1);
  const auto next = issuer.commit(1, 1, later);
  ASSERT_TRUE(next.has_value());
  EXPECT_GT(*next, *tag);
  issuer.tick(later + drive_core::kTagLifetime - runtime_core::Duration{1});
  EXPECT_EQ(issuer.current(), *next);
}

TEST(ExecTag, BootQuarantineRefusesCommit) {
  EXPECT_EQ(drive_core::kBootQuarantine, milliseconds(100));
  ExecTagIssuer issuer(3, kBoot);
  const auto at_boot = issuer.commit_ex(1, 1, kBoot);
  EXPECT_FALSE(at_boot.tag.has_value());
  EXPECT_EQ(at_boot.refusal, std::optional<CommitRefusal>(CommitRefusal::kBootQuarantine));
  EXPECT_FALSE(issuer.commit(1, 1, kReady - runtime_core::Duration{1}).has_value());
  EXPECT_EQ(issuer.current(), 0);
  // The quarantine refusal does not consume a tag.
  EXPECT_EQ(issuer.commit(1, 1, kReady), std::optional<std::uint16_t>(exec_tag_start(3)));
  // A new boot starts a new quarantine and revokes the current tag.
  const MonoTime reboot = kReady + milliseconds(500);
  issuer.boot(4, reboot);
  EXPECT_EQ(issuer.current(), 0);
  EXPECT_FALSE(issuer.commit(1, 1, reboot + milliseconds(50)).has_value());
  EXPECT_EQ(issuer.commit(1, 1, reboot + milliseconds(100)),
            std::optional<std::uint16_t>(exec_tag_start(4)));
}

}  // namespace
