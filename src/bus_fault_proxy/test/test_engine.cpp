// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "bus_fault_proxy/engine.hpp"

using namespace bus_fault_proxy;
using canopen_codec::Frame;
using runtime_core::MonoTime;

namespace {

constexpr std::int64_t kMs = 1'000'000;
MonoTime ms(std::int64_t v) { return MonoTime{v * kMs}; }
MonoTime us(std::int64_t v) { return MonoTime{v * 1'000}; }

Frame frame(std::uint16_t id, std::uint8_t tag = 0) {
  Frame f;
  f.cob_id = id;
  f.dlc = 4;
  f.data[0] = tag;
  f.data[1] = 0x10;
  return f;
}

Engine make(const char* text, MonoTime epoch = MonoTime{}) {
  const auto r = parse_rules(text);
  EXPECT_EQ(r.error, "");
  return Engine(r.rules, epoch);
}

}  // namespace

TEST(Engine, NoRuleForwardsAtOnce) {
  Engine e = make("");
  const auto out = e.on_frame(Dir::kH2D, frame(0x200), ms(5));
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].dir, Dir::kH2D);
  EXPECT_EQ(out[0].frame, frame(0x200));
  EXPECT_TRUE(e.counts().empty());
}

TEST(Engine, FirstMatchingRuleWins) {
  Engine e = make("h2d 0x200/0x7FF drop\nh2d 0x200/0x7FF dup\nh2d 0x300/0x7FF dup\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), ms(1)).empty());
  EXPECT_EQ(e.counts()[0].dropped, 1U);
  EXPECT_EQ(e.counts()[1].matched, 0U);
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x300), ms(2)).size(), 2U);
  // A non-matching identifier is forwarded untouched.
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x400), ms(3)).size(), 1U);
}

TEST(Engine, MaskSelectsIdentifierRange) {
  Engine e = make("d2h 0x580/0x780 drop\n");
  EXPECT_TRUE(e.on_frame(Dir::kD2H, frame(0x585), ms(1)).empty());
  EXPECT_EQ(e.on_frame(Dir::kD2H, frame(0x605), ms(1)).size(), 1U);
}

TEST(Engine, DelayReleasesAtDueTimeInOrder) {
  Engine e = make("h2d 0x200/0x7FF delay 30\nh2d 0x201/0x7FF delay 10\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 1), ms(0)).empty());
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x201, 2), ms(5)).empty());
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 3), ms(6)).empty());
  ASSERT_TRUE(e.next_due().has_value());
  EXPECT_EQ(*e.next_due(), ms(15));
  EXPECT_TRUE(e.poll(us(14'999)).empty());
  auto out = e.poll(ms(15));
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].frame.data[0], 2);
  EXPECT_EQ(*e.next_due(), ms(30));
  out = e.poll(ms(100));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].frame.data[0], 1);
  EXPECT_EQ(out[1].frame.data[0], 3);
  EXPECT_FALSE(e.next_due().has_value());
  EXPECT_EQ(e.counts()[0].delayed, 2U);
  EXPECT_EQ(e.counts()[1].delayed, 1U);
}

TEST(Engine, DueFramesPrecedeTheNewFrame) {
  Engine e = make("h2d 0x200/0x7FF delay 10\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 1), ms(0)).empty());
  const auto out = e.on_frame(Dir::kH2D, frame(0x300, 2), ms(10));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].frame.data[0], 1);
  EXPECT_EQ(out[1].frame.data[0], 2);
}

TEST(Engine, EveryPredicateSelectsNthMatch) {
  for (const char* action : {"drop", "dup"}) {
    Engine e = make((std::string("h2d 0x200/0x7FF every 3 offset 1 ") + action).c_str());
    std::vector<int> hit;
    for (int n = 0; n < 9; ++n) {
      const auto out = e.on_frame(Dir::kH2D, frame(0x200, static_cast<std::uint8_t>(n)), ms(n));
      const std::size_t expected_normal = 1;
      if (out.size() != expected_normal) hit.push_back(n);
    }
    EXPECT_EQ(hit, (std::vector<int>{1, 4, 7})) << action;
    EXPECT_EQ(e.counts()[0].matched, 3U);
  }
}

TEST(Engine, OccurrenceCountIgnoresOtherIdentifiers) {
  Engine e = make("h2d 0x200/0x7FF every 2 drop\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), ms(0)).empty());
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x300), ms(1)).size(), 1U);
  EXPECT_EQ(e.on_frame(Dir::kD2H, frame(0x200), ms(2)).size(), 1U);
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x200), ms(3)).size(), 1U);
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), ms(4)).empty());
}

TEST(Engine, DropWindowBoundaries) {
  Engine e = make("h2d 0x200/0x7FF window 100 200 drop\n", MonoTime{0});
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x200), us(99'999)).size(), 1U);
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), ms(100)).empty());
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), us(199'999)).empty());
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x200), ms(200)).size(), 1U);
  EXPECT_EQ(e.counts()[0].dropped, 2U);
}

TEST(Engine, WindowIsRelativeToEpoch) {
  Engine e = make("h2d 0x200/0x7FF window 0 10 drop\n", ms(1000));
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x200), ms(999)).size(), 1U);
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200), ms(1000)).empty());
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x200), ms(1010)).size(), 1U);
}

TEST(Engine, DupSendsTwice) {
  Engine e = make("h2d 0x200/0x7FF dup\n");
  const auto out = e.on_frame(Dir::kH2D, frame(0x200, 9), ms(1));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].frame, out[1].frame);
  EXPECT_EQ(e.counts()[0].duplicated, 1U);
}

TEST(Engine, DupCorruptSendsOriginalThenCorruptedCopy) {
  Engine e = make("d2h 0x180/0x7FF dup_corrupt 1 0xFF\n");
  const auto out = e.on_frame(Dir::kD2H, frame(0x180, 5), ms(1));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].frame, frame(0x180, 5));
  Frame expect = frame(0x180, 5);
  expect.data[1] = 0x10 ^ 0xFF;
  EXPECT_EQ(out[1].frame, expect);
  EXPECT_EQ(e.counts()[0].duplicated, 1U);
  EXPECT_EQ(e.counts()[0].corrupted, 1U);
}

TEST(Engine, CorruptXorsOneByte) {
  Engine e = make("h2d 0x200/0x7FF corrupt 0 0x0F\n");
  const auto out = e.on_frame(Dir::kH2D, frame(0x200, 0xA5), ms(1));
  ASSERT_EQ(out.size(), 1U);
  Frame expect = frame(0x200, 0xA5);
  expect.data[0] = 0xA5 ^ 0x0F;
  EXPECT_EQ(out[0].frame, expect);
  EXPECT_EQ(e.counts()[0].corrupted, 1U);
}

TEST(Engine, CorruptBeyondDlcLeavesFrameUnchanged) {
  Engine e = make("h2d 0x200/0x7FF corrupt 6 0xFF\n");
  const auto out = e.on_frame(Dir::kH2D, frame(0x200), ms(1));
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].frame, frame(0x200));
  EXPECT_EQ(e.counts()[0].matched, 1U);
  EXPECT_EQ(e.counts()[0].corrupted, 0U);
}

TEST(Engine, ReorderSwapsWithNextMatching) {
  Engine e = make("h2d 0x200/0x7FF reorder\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 1), ms(0)).empty());
  EXPECT_EQ(*e.next_due(), ms(20));
  // A non-matching frame passes through while one is held.
  EXPECT_EQ(e.on_frame(Dir::kH2D, frame(0x300, 7), ms(2)).size(), 1U);
  const auto out = e.on_frame(Dir::kH2D, frame(0x200, 2), ms(4));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].frame.data[0], 2);
  EXPECT_EQ(out[1].frame.data[0], 1);
  EXPECT_FALSE(e.next_due().has_value());
  EXPECT_EQ(e.counts()[0].reordered, 1U);
  EXPECT_EQ(e.counts()[0].flushed, 0U);
  // The next matching frame starts a new hold.
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 3), ms(5)).empty());
}

TEST(Engine, ReorderFlushesAfterHoldLimit) {
  Engine e = make("h2d 0x200/0x7FF reorder\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 1), ms(0)).empty());
  EXPECT_TRUE(e.poll(us(19'999)).empty());
  const auto out = e.poll(ms(20));
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].frame.data[0], 1);
  EXPECT_EQ(e.counts()[0].flushed, 1U);
  EXPECT_EQ(e.counts()[0].reordered, 0U);
  EXPECT_FALSE(e.next_due().has_value());
}

TEST(Engine, ReorderHoldExpiresBeforeANewMatchingFrame) {
  Engine e = make("h2d 0x200/0x7FF reorder\n");
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 1), ms(0)).empty());
  // The second match arrives after the hold limit: the first is flushed, then the second is held.
  const auto out = e.on_frame(Dir::kH2D, frame(0x200, 2), ms(25));
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].frame.data[0], 1);
  EXPECT_EQ(*e.next_due(), ms(45));
}

TEST(Engine, DirectionsAreIndependent) {
  Engine e = make("h2d 0x200/0x7FF drop\nd2h 0x200/0x7FF reorder\n");
  EXPECT_TRUE(e.on_frame(Dir::kD2H, frame(0x200, 1), ms(0)).empty());
  // An h2d frame neither releases nor swaps the d2h held frame.
  EXPECT_TRUE(e.on_frame(Dir::kH2D, frame(0x200, 2), ms(1)).empty());
  EXPECT_TRUE(e.next_due().has_value());
  const auto out = e.on_frame(Dir::kD2H, frame(0x200, 3), ms(2));
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].dir, Dir::kD2H);
  EXPECT_EQ(out[0].frame.data[0], 3);
  EXPECT_EQ(e.counts()[0].dropped, 1U);
  EXPECT_EQ(e.counts()[1].reordered, 1U);
}

TEST(Engine, NextDueEmptyWhenNothingHeld) {
  Engine e = make("h2d 0x200/0x7FF drop\nh2d 0x300/0x7FF dup\n");
  EXPECT_FALSE(e.next_due().has_value());
  e.on_frame(Dir::kH2D, frame(0x200), ms(0));
  e.on_frame(Dir::kH2D, frame(0x300), ms(0));
  EXPECT_FALSE(e.next_due().has_value());
  EXPECT_TRUE(e.poll(ms(1000)).empty());
}
