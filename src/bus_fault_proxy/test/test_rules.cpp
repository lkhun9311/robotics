// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "bus_fault_proxy/rules.hpp"

using namespace bus_fault_proxy;

TEST(Rules, ParsesEveryAction) {
  const auto r = parse_rules(
      "h2d 0x200/0x7FF delay 80\n"
      "d2h 0x180/0x780 drop\n"
      "h2d 0x600/0x780 dup\n"
      "d2h 0x580/0x780 dup_corrupt 3 0xFF\n"
      "h2d 0x300/0x7FF corrupt 7 1\n"
      "d2h 0x700/0x780 reorder\n"
      "h2d 0x201/0x7FF every 3 offset 1 window 100 200 drop\n");
  ASSERT_EQ(r.error, "");
  ASSERT_EQ(r.rules.size(), 7U);
  EXPECT_EQ(r.rules[0].action, Action::kDelay);
  EXPECT_EQ(r.rules[0].delay_ms, 80U);
  EXPECT_EQ(r.rules[0].id, 0x200);
  EXPECT_EQ(r.rules[1].dir, Dir::kD2H);
  EXPECT_EQ(r.rules[1].mask, 0x780);
  EXPECT_EQ(r.rules[2].action, Action::kDup);
  EXPECT_EQ(r.rules[3].action, Action::kDupCorrupt);
  EXPECT_EQ(r.rules[3].byte, 3);
  EXPECT_EQ(r.rules[3].xor_mask, 0xFF);
  EXPECT_EQ(r.rules[4].action, Action::kCorrupt);
  EXPECT_EQ(r.rules[5].action, Action::kReorder);
  EXPECT_EQ(r.rules[6].every, 3U);
  EXPECT_EQ(r.rules[6].offset, 1U);
  EXPECT_TRUE(r.rules[6].has_window);
  EXPECT_EQ(r.rules[6].window_from_ms, 100U);
  EXPECT_EQ(r.rules[6].window_to_ms, 200U);
  EXPECT_EQ(r.rules[0].every, 1U);
  EXPECT_FALSE(r.rules[0].has_window);
}

TEST(Rules, PredicatesApplyToEveryAction) {
  for (const char* action :
       {"delay 5", "drop", "dup", "dup_corrupt 0 1", "corrupt 0 1", "reorder"}) {
    const auto r = parse_rules(std::string("h2d 1/0x7FF every 4 offset 2 window 1 2 ") + action);
    ASSERT_EQ(r.error, "") << action;
    EXPECT_EQ(r.rules[0].every, 4U);
    EXPECT_EQ(r.rules[0].offset, 2U);
    EXPECT_TRUE(r.rules[0].has_window);
  }
}

TEST(Rules, RejectsDelayAbove80ms) {
  const auto r = parse_rules("h2d 0x200/0x7FF delay 81\n");
  EXPECT_NE(r.error.find("line 1"), std::string::npos) << r.error;
  EXPECT_NE(r.error.find("81"), std::string::npos) << r.error;
  EXPECT_TRUE(r.rules.empty());
}

TEST(Rules, RejectsUnknownWordBadByteAndEmptyWindow) {
  auto r = parse_rules("h2d 0x200/0x7FF drop\nh2d 0x200/0x7FF explode\n");
  EXPECT_TRUE(r.rules.empty());
  EXPECT_NE(r.error.find("line 2"), std::string::npos) << r.error;
  EXPECT_NE(r.error.find("explode"), std::string::npos) << r.error;

  r = parse_rules("sideways 1/1 drop");
  EXPECT_NE(r.error.find("sideways"), std::string::npos) << r.error;

  r = parse_rules("h2d 1/0x7FF corrupt 8 1");
  EXPECT_NE(r.error.find("'8'"), std::string::npos) << r.error;
  r = parse_rules("h2d 1/0x7FF dup_corrupt 0 256");
  EXPECT_NE(r.error.find("'256'"), std::string::npos) << r.error;

  r = parse_rules("h2d 1/0x7FF window 200 200 drop");
  EXPECT_NE(r.error.find("window"), std::string::npos) << r.error;
  r = parse_rules("h2d 1/0x7FF window 200 100 drop");
  EXPECT_NE(r.error.find("window"), std::string::npos) << r.error;

  EXPECT_NE(parse_rules("h2d 1/0x7FF every 0 drop").error, "");
  EXPECT_NE(parse_rules("h2d 1/0x7FF every 3 offset 3 drop").error, "");
  EXPECT_EQ(parse_rules("h2d 1/0x7FF every 3 offset 2 drop").error, "");
  EXPECT_NE(parse_rules("h2d 1/0x7FF drop extra").error, "");
  EXPECT_NE(parse_rules("h2d 0x800/0x7FF drop").error, "");
  EXPECT_NE(parse_rules("h2d 0x100 drop").error, "");
  EXPECT_NE(parse_rules("h2d 1/0x7FF").error, "");
  EXPECT_NE(parse_rules("h2d 1/0x7FF delay").error, "");
  EXPECT_NE(parse_rules("h2d 1/0x7FF window 1 drop").error, "");
}

TEST(Rules, CommentsAndBlankLinesIgnored) {
  const auto r = parse_rules(
      "# header\n"
      "\n"
      "   \t \n"
      "h2d 0x200/0x7FF drop   # trailing\r\n"
      "  # indented comment\n"
      "d2h 0x180/0x7FF dup");
  ASSERT_EQ(r.error, "");
  EXPECT_EQ(r.rules.size(), 2U);
  EXPECT_TRUE(parse_rules("").rules.empty());
  EXPECT_EQ(parse_rules("").error, "");
}
