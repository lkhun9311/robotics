// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "drive_core/assembly.hpp"

namespace {

using canopen_codec::od::Rpdo1;
using canopen_codec::od::Rpdo2;
using drive_core::AssemblyResult;
using drive_core::Command;
using drive_core::CommandAssembler;
using drive_core::Counters;
using drive_core::ExecTagIssuer;
using drive_core::OfferResult;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

constexpr MonoTime kBoot{1'000'000'000};
const MonoTime kReady = kBoot + milliseconds(100);
constexpr std::uint32_t kNow = 5'000'000;  // Wire µs at the SYNC under test.

Rpdo1 r1(std::uint16_t tag, std::uint16_t seq, std::int32_t target) {
  Rpdo1 f;
  f.target_position = target;
  f.command_seq = seq;
  f.exec_tag = tag;
  return f;
}

Rpdo2 r2(std::uint16_t tag, std::uint16_t seq, std::uint32_t until) {
  Rpdo2 f;
  f.valid_until_us = until;
  f.command_seq = seq;
  f.exec_tag = tag;
  return f;
}

// The drive's side of a SYNC in these tests: every candidate is actuated and marked.
AssemblyResult actuate(CommandAssembler& assembler, std::uint32_t now) {
  AssemblyResult result = assembler.on_sync(now);
  if (result.command) {
    assembler.mark_actuated(*result.command);
  }
  return result;
}

// A committed tag and an assembler following it.
struct Drive {
  ExecTagIssuer issuer{0x2013, kBoot};
  CommandAssembler assembler{issuer};
  std::uint16_t tag{};

  Drive() { tag = commit(); }
  std::uint16_t commit() {
    const auto t = issuer.commit(1, 1, kReady);
    EXPECT_TRUE(t.has_value());
    return t.value_or(0);
  }
  // Offers both halves of one command and runs the SYNC that evaluates them.
  AssemblyResult send(std::uint16_t seq, std::int32_t target, std::uint32_t now = kNow) {
    EXPECT_EQ(assembler.offer(r1(tag, seq, target)), OfferResult::kAccepted);
    EXPECT_EQ(assembler.offer(r2(tag, seq, now + 20'000)), OfferResult::kAccepted);
    return actuate(assembler, now);
  }
};

TEST(Assembly, BothHalvesSameTagSameSeqAssemble) {
  Drive d;
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, -123'456)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 30'000)), OfferResult::kAccepted);
  const auto result = actuate(d.assembler, kNow);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(*result.command, (Command{-123'456, kNow + 30'000, 5, d.tag}));
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped, Counters{});
  // RPDO2 first works the same way; the window cleared after the SYNC.
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 6, kNow + 40'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 6, 7)), OfferResult::kAccepted);
  const auto second = actuate(d.assembler, kNow + 10'000);
  ASSERT_TRUE(second.command.has_value());
  EXPECT_EQ(second.command->command_seq, 6);
  EXPECT_EQ(second.command->target_position, 7);
  // A pair whose deadline is reached at this SYNC is dropped, never actuated.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 7, 8)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 7, kNow + 20'000)), OfferResult::kAccepted);
  const auto expired = actuate(d.assembler, kNow + 20'000);
  EXPECT_FALSE(expired.command.has_value());
  EXPECT_EQ(expired.dropped.not_valid, 1U);
  EXPECT_FALSE(actuate(d.assembler, kNow + 30'000).command.has_value());
}

TEST(Assembly, OldTagFragmentIsDropped) {
  // No tag committed yet: every fragment is of another tag.
  ExecTagIssuer issuer(0x2013, kBoot);
  CommandAssembler assembler(issuer);
  EXPECT_EQ(assembler.offer(r1(0, 1, 1)), OfferResult::kDroppedOtherTag);
  EXPECT_EQ(assembler.offer(r2(drive_core::exec_tag_start(0x2013), 1, kNow + 1'000)),
            OfferResult::kDroppedOtherTag);
  EXPECT_EQ(actuate(assembler, kNow).dropped.other_tag, 2U);

  Drive d;
  const std::uint16_t old_tag = d.tag;
  d.tag = d.commit();
  ASSERT_NE(d.tag, old_tag);
  EXPECT_EQ(d.assembler.offer(r1(old_tag, 1, 1)), OfferResult::kDroppedOtherTag);
  EXPECT_EQ(d.assembler.offer(r2(old_tag, 1, kNow + 1'000)), OfferResult::kDroppedOtherTag);
  const auto result = actuate(d.assembler, kNow);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.other_tag, 2U);
  EXPECT_EQ(result.dropped.missing_half, 0U);
}

TEST(Assembly, FragmentsFromTwoCommitsNeverPair) {
  Drive d;
  const std::uint16_t a = d.tag;
  const std::uint16_t b = d.commit();
  d.tag = b;
  // A late RPDO1 of the earlier commit, same seq as a fresh RPDO2 of the new one.
  d.assembler.offer(r1(a, 5, 111));
  d.assembler.offer(r2(b, 5, kNow + 10'000));
  auto result = actuate(d.assembler, kNow);
  EXPECT_FALSE(result.command.has_value()) << "target of tag A paired with deadline of tag B";
  EXPECT_FALSE(result.integrity_violation);
  // The other way round.
  d.assembler.offer(r1(b, 6, 222));
  d.assembler.offer(r2(a, 6, kNow + 20'000));
  result = actuate(d.assembler, kNow + 10'000);
  EXPECT_FALSE(result.command.has_value()) << "target of tag B paired with deadline of tag A";
  // A half held when the new commit happens is discarded with the old tag.
  EXPECT_EQ(d.assembler.offer(r1(b, 7, 333)), OfferResult::kAccepted);
  const std::uint16_t c = d.commit();
  d.tag = c;
  EXPECT_EQ(d.assembler.offer(r2(c, 7, kNow + 30'000)), OfferResult::kAccepted);
  result = actuate(d.assembler, kNow + 20'000);
  EXPECT_FALSE(result.command.has_value());
}

TEST(Assembly, SeqWrapWithinTagIsSafe) {
  Drive d;
  ASSERT_TRUE(d.send(0xFFFE, 1).command.has_value());
  ASSERT_TRUE(d.send(0xFFFF, 2, kNow + 10'000).command.has_value());
  const auto wrapped = d.send(0x0000, 3, kNow + 20'000);
  ASSERT_TRUE(wrapped.command.has_value());
  EXPECT_EQ(wrapped.command->command_seq, 0);
  // Copies of the commands before the wrap are older, not 65535 commands ahead.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 0xFFFF, 2)), OfferResult::kDroppedStaleSeq);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 0xFFFF, kNow + 40'000)), OfferResult::kDroppedStaleSeq);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 0xFFFE, 9)), OfferResult::kDroppedStaleSeq);
  const auto result = actuate(d.assembler, kNow + 30'000);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.stale_seq, 3U);
  ASSERT_TRUE(d.send(0x0001, 4, kNow + 40'000).command.has_value());
}

TEST(Assembly, OlderSeqIsDropped) {
  Drive d;
  ASSERT_TRUE(d.send(10, 100).command.has_value());
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 9, 90)), OfferResult::kDroppedStaleSeq);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 9, kNow + 30'000)), OfferResult::kDroppedStaleSeq);
  // An identical repeat of the actuated command is stale too, not a conflict.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 10, 100)), OfferResult::kDroppedStaleSeq);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 10, kNow + 20'000)), OfferResult::kDroppedStaleSeq);
  const auto result = actuate(d.assembler, kNow + 10'000);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.stale_seq, 4U);
}

TEST(Assembly, LostSyncPairsAreNotAViolationAndNewestWins) {
  Drive d;
  // A SYNC was lost: commands k and k+1 both arrive in one window, interleaved.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 500)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 6, 600)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 6, kNow + 20'000)), OfferResult::kAccepted);
  EXPECT_FALSE(d.assembler.integrity_violation());
  auto result = actuate(d.assembler, kNow);
  EXPECT_FALSE(result.integrity_violation);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(*result.command, (Command{600, kNow + 20'000, 6, d.tag}));
  EXPECT_EQ(result.dropped.superseded, 1U);
  // Newest wins across the seq wrap too.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 0xFFFF, 1)), OfferResult::kDroppedStaleSeq);
  Drive w;
  w.assembler.offer(r1(w.tag, 0x0000, 2));
  w.assembler.offer(r2(w.tag, 0x0000, kNow + 20'000));
  w.assembler.offer(r1(w.tag, 0xFFFF, 1));
  w.assembler.offer(r2(w.tag, 0xFFFF, kNow + 10'000));
  result = actuate(w.assembler, kNow);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->command_seq, 0x0000);
  EXPECT_EQ(result.dropped.superseded, 1U);
  // Halves of different seqs never pair, and are not a violation: two ordinary gaps.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 7, 700)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 8, kNow + 30'000)), OfferResult::kAccepted);
  result = actuate(d.assembler, kNow + 10'000);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.missing_half, 2U);
  // Two RPDO1 of different seqs in one window are not a violation either.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 9, 900)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 10, 1'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 10, kNow + 40'000)), OfferResult::kAccepted);
  result = actuate(d.assembler, kNow + 20'000);
  EXPECT_FALSE(result.integrity_violation);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->command_seq, 10);
  EXPECT_EQ(result.dropped.missing_half, 1U);
  // No fallback to an older pair: the newest complete pair has a closed deadline, the older one
  // an open deadline, and nothing is actuated.
  {
    Drive f;
    f.assembler.offer(r1(f.tag, 5, 500));
    f.assembler.offer(r2(f.tag, 5, kNow + 10'000));
    f.assembler.offer(r1(f.tag, 6, 600));
    f.assembler.offer(r2(f.tag, 6, kNow));  // Expired at exact equality.
    const auto closed = actuate(f.assembler, kNow);
    EXPECT_FALSE(closed.command.has_value());
    EXPECT_FALSE(closed.integrity_violation);
    EXPECT_EQ(closed.dropped.not_valid, 1U);
    EXPECT_EQ(closed.dropped.superseded, 1U);
  }
  // The window is bounded: a ninth seq evicts the oldest, and an arriving seq older than every
  // held one is itself the one dropped.
  static_assert(CommandAssembler::kWindowSlots == 8);
  for (std::uint16_t seq = 20; seq <= 28; ++seq) {
    ASSERT_EQ(d.assembler.offer(r1(d.tag, seq, seq)), OfferResult::kAccepted) << seq;
  }
  // Seq 20 was evicted by seq 28; its late RPDO2 is older than every held seq.
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 20, kNow + 40'000)), OfferResult::kDroppedSuperseded);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 19, 19)), OfferResult::kDroppedSuperseded);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 28, kNow + 40'000)), OfferResult::kAccepted);
  result = actuate(d.assembler, kNow + 30'000);
  EXPECT_FALSE(result.integrity_violation);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->command_seq, 28);
  EXPECT_EQ(result.dropped.superseded, 3U);  // Seq 20 evicted, its late RPDO2, seq 19.
  EXPECT_EQ(result.dropped.missing_half, 7U);
}

TEST(Assembly, SameSeqDifferentPayloadIsIntegrityViolation) {
  Drive d;
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 1'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 1'000)), OfferResult::kDuplicate);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 2'000)), OfferResult::kIntegrityViolation);
  EXPECT_TRUE(actuate(d.assembler, kNow).integrity_violation);

  d.assembler.reset();
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'001)), OfferResult::kIntegrityViolation);
  EXPECT_TRUE(actuate(d.assembler, kNow).integrity_violation);

  // A second payload for the seq that was already actuated, in a later window.
  d.assembler.reset();
  ASSERT_TRUE(d.send(40, 4'000).command.has_value());
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 40, 4'001)), OfferResult::kIntegrityViolation);
  EXPECT_TRUE(actuate(d.assembler, kNow + 10'000).integrity_violation);
}

TEST(Assembly, ViolationDetectedAtIngestionWithoutSync) {
  Drive d;
  EXPECT_FALSE(d.assembler.integrity_violation());
  d.assembler.offer(r2(d.tag, 5, kNow + 10'000));
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'500)), OfferResult::kIntegrityViolation);
  EXPECT_TRUE(d.assembler.integrity_violation()) << "known before any SYNC";
  // Normal actuation is blocked at once: a complete, valid pair after the violation is refused.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 7, 1)), OfferResult::kBlocked);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 7, kNow + 10'000)), OfferResult::kBlocked);
  auto result = actuate(d.assembler, kNow);
  EXPECT_TRUE(result.integrity_violation);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_EQ(result.dropped.blocked, 2U);
  // Latched until the tag changes (the drive revokes it on the policy stop).
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 8, 1)), OfferResult::kBlocked);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 8, kNow + 20'000)), OfferResult::kBlocked);
  result = actuate(d.assembler, kNow + 10'000);
  EXPECT_TRUE(result.integrity_violation);
  EXPECT_FALSE(result.command.has_value());
  d.issuer.revoke();
  EXPECT_FALSE(actuate(d.assembler, kNow + 20'000).integrity_violation);
  EXPECT_FALSE(d.assembler.integrity_violation());
}

TEST(Assembly, ViolationStaysPendingAcrossTagChangeUntilReported) {
  // The tag changes (revoke, or lifetime expiry at the W tick) between detection and the next
  // SYNC. The violation must still be reported, exactly once.
  for (const bool by_expiry : {false, true}) {
    SCOPED_TRACE(by_expiry ? "lifetime expiry" : "revoke");
    Drive d;
    EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'000)), OfferResult::kAccepted);
    EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'500)), OfferResult::kIntegrityViolation);
    if (by_expiry) {
      d.issuer.tick(kReady + drive_core::kTagLifetime);
    } else {
      d.issuer.revoke();
    }
    ASSERT_EQ(d.issuer.current(), 0);
    EXPECT_TRUE(d.assembler.integrity_violation()) << "erased before it was reported";
    // An offer that follows the tag change must not erase it either.
    EXPECT_EQ(d.assembler.offer(r1(d.tag, 6, 1)), OfferResult::kDroppedOtherTag);
    EXPECT_TRUE(d.assembler.integrity_violation());
    EXPECT_TRUE(actuate(d.assembler, kNow).integrity_violation);
    // Reported once; after that the new tag starts clean.
    EXPECT_FALSE(d.assembler.integrity_violation());
    EXPECT_FALSE(actuate(d.assembler, kNow + 10'000).integrity_violation);
    d.tag = d.issuer.commit(1, 1, kReady + drive_core::kTagLifetime + milliseconds(1)).value_or(0);
    ASSERT_NE(d.tag, 0);
    const auto result = d.send(1, 1, kNow + 20'000);
    EXPECT_FALSE(result.integrity_violation);
    EXPECT_TRUE(result.command.has_value());
  }
}

TEST(Assembly, MissingHalfIsAnOrdinaryGap) {
  Drive d;
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 1)), OfferResult::kAccepted);
  auto result = actuate(d.assembler, kNow);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.missing_half, 1U);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 6, kNow + 30'000)), OfferResult::kAccepted);
  result = actuate(d.assembler, kNow + 10'000);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  EXPECT_EQ(result.dropped.missing_half, 1U);
  // The next complete command is actuated; the lone halves never pair across windows.
  result = d.send(7, 70, kNow + 20'000);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->command_seq, 7);
  EXPECT_FALSE(d.assembler.integrity_violation());
}

TEST(Assembly, LegitimateSeqWrapIsNotAConflict) {
  Drive d;
  // 100 Hz for 5.12 s, crossing both the 16-bit seq wrap and the 2^32 µs wire-time wrap.
  std::uint32_t now = 0xFFFF'0000U;
  std::uint16_t seq = 0xFF00;
  int actuated = 0;
  for (int i = 0; i < 512; ++i) {
    const auto target = static_cast<std::int32_t>(i * 1'000 - 200'000);
    ASSERT_EQ(d.assembler.offer(r1(d.tag, seq, target)), OfferResult::kAccepted) << i;
    ASSERT_EQ(d.assembler.offer(r2(d.tag, seq, now + 10'000U + 20'000U)), OfferResult::kAccepted)
        << i;
    now += 10'000U;  // The next SYNC.
    const auto result = actuate(d.assembler, now);
    ASSERT_FALSE(result.integrity_violation) << i;
    ASSERT_EQ(result.dropped, Counters{}) << i;
    ASSERT_TRUE(result.command.has_value()) << i;
    EXPECT_EQ(result.command->command_seq, seq);
    EXPECT_EQ(result.command->target_position, target);
    ++actuated;
    ++seq;
  }
  EXPECT_EQ(actuated, 512);
  EXPECT_LT(seq, 0xFF00) << "the stream did not cross the seq wrap";
  EXPECT_LT(now, 0xFFFF'0000U) << "the stream did not cross the wire-time wrap";
}

TEST(Replay, OldCommandWithOpenDeadlineAfterResetCommunicationIsDropped) {
  Drive d;
  const std::uint16_t old_tag = d.tag;
  const Rpdo1 old1 = r1(old_tag, 10, 5'000);
  const Rpdo2 old2 = r2(old_tag, 10, kNow + 40'000);
  d.assembler.offer(old1);
  d.assembler.offer(old2);
  ASSERT_TRUE(actuate(d.assembler, kNow).command.has_value());
  // NMT reset communication: tag revoked, per-command state cleared, allocator kept.
  d.issuer.reset_communication();
  d.assembler.reset();
  d.tag = d.commit();
  EXPECT_EQ(static_cast<std::uint32_t>(d.tag), static_cast<std::uint32_t>(old_tag) + 1U);
  // The old command is replayed while its deadline is still open, and with a seq that is new
  // for the cleared state. It carries the revoked tag, so it is dropped.
  ASSERT_TRUE(drive_core::validity_open(old2.valid_until_us, kNow + 1'000, drive_core::kLeaseUs));
  EXPECT_EQ(d.assembler.offer(old1), OfferResult::kDroppedOtherTag);
  EXPECT_EQ(d.assembler.offer(old2), OfferResult::kDroppedOtherTag);
  const auto result = actuate(d.assembler, kNow + 1'000);
  EXPECT_FALSE(result.command.has_value());
  EXPECT_EQ(result.dropped.other_tag, 2U);
  // The fresh command of the new tag works.
  ASSERT_TRUE(d.send(1, 1, kNow + 2'000).command.has_value());
}

TEST(Assembly, OnlyAnActuationMarksTheSeq) {
  Drive d;
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 1)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 20'000)), OfferResult::kAccepted);
  auto result = d.assembler.on_sync(kNow);  // A candidate the owner did not actuate.
  ASSERT_TRUE(result.command.has_value());
  // The same seq with another deadline is a new candidate, not a second payload.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 1)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 30'000)), OfferResult::kAccepted);
  result = d.assembler.on_sync(kNow + 10'000);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_FALSE(result.integrity_violation);
  d.assembler.mark_actuated(*result.command);
  // Once actuated, another payload for that seq is a violation and an older seq is stale.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 4, 1)), OfferResult::kDroppedStaleSeq);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 40'000)), OfferResult::kIntegrityViolation);
}

TEST(Assembly, MarkActuatedIgnoresAnotherTag) {
  Drive d;
  const Command old{1, kNow + 20'000, 5, d.tag};
  d.tag = d.commit();
  d.assembler.mark_actuated(old);
  // Seq 5 of the new tag is not stale: the old tag's command was never this tag's.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 5, 2)), OfferResult::kAccepted);
}

TEST(Assembly, AcknowledgedViolationIsNotReportedAgain) {
  Drive d;
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 5, kNow + 10'500)), OfferResult::kIntegrityViolation);
  d.assembler.acknowledge_violation();
  // Still latched for this tag: still blocking.
  EXPECT_TRUE(d.assembler.integrity_violation());
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 6, 1)), OfferResult::kBlocked);
  // The owner acted on it (its stop revokes the tag) before any SYNC: the first SYNC after the
  // change does not report it again. Unacknowledged, it would
  // (ViolationStaysPendingAcrossTagChangeUntilReported).
  d.issuer.revoke();
  EXPECT_FALSE(d.assembler.on_sync(kNow).integrity_violation);
  EXPECT_FALSE(d.assembler.integrity_violation());
}

TEST(Assembly, DroppedTargetsStayBoundAndEvictedSeqsTurnStale) {
  Drive d;
  // Nine candidates, each dropped by the owner: seqs 1..9, target = 10 * seq.
  for (std::uint16_t seq = 1; seq <= 9; ++seq) {
    EXPECT_EQ(d.assembler.offer(r1(d.tag, seq, 10 * seq)), OfferResult::kAccepted);
    EXPECT_EQ(d.assembler.offer(r2(d.tag, seq, kNow + 20'000)), OfferResult::kAccepted);
    const auto result = d.assembler.on_sync(kNow);
    ASSERT_TRUE(result.command.has_value());
    d.assembler.mark_dropped(*result.command);
  }
  // Seq 1 was evicted (eight remembered): it and anything older are stale now.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 1, 10)), OfferResult::kDroppedStaleSeq);
  // A remembered seq takes its own target again with a new deadline, and nothing else.
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 2, 20)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r2(d.tag, 2, kNow + 30'000)), OfferResult::kAccepted);
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 9, 91)), OfferResult::kIntegrityViolation);
  // A new tag forgets all of it.
  d.tag = d.commit();
  EXPECT_EQ(d.assembler.offer(r1(d.tag, 1, 11)), OfferResult::kAccepted);
}

}  // namespace
