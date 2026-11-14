// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <vector>

#include "lely_peer/args.hpp"

namespace {

using lely_peer::json_escape;
using lely_peer::parse_nodes;
using lely_peer::parse_number;
using lely_peer::parse_options;

TEST(ParseNodes, RangeListAndSingle) {
  EXPECT_EQ(parse_nodes("1-6").nodes, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(parse_nodes("3").nodes, (std::vector<std::uint8_t>{3}));
  EXPECT_EQ(parse_nodes("1,3-6").nodes, (std::vector<std::uint8_t>{1, 3, 4, 5, 6}));
  EXPECT_EQ(parse_nodes("127").nodes, (std::vector<std::uint8_t>{127}));
}

TEST(ParseNodes, RejectsBadLists) {
  for (const std::string_view bad :
       {"", "0", "128", "6-1", "1,1", "1-3,2", "a", "1-", ",1", "1,"}) {
    const auto result = parse_nodes(bad);
    EXPECT_FALSE(result.ok) << bad;
    EXPECT_FALSE(result.error.empty()) << bad;
    EXPECT_TRUE(result.nodes.empty()) << bad;
  }
}

TEST(ParseNumber, BoundsAndSyntax) {
  EXPECT_EQ(parse_number("3000", 0, 3600000).value, 3000U);
  EXPECT_TRUE(parse_number("0", 0, 1).ok);
  EXPECT_FALSE(parse_number("2", 0, 1).ok);
  EXPECT_FALSE(parse_number("-1", 0, 10).ok);
  EXPECT_FALSE(parse_number("1x", 0, 10).ok);
  EXPECT_FALSE(parse_number("", 0, 10).ok);
}

TEST(ParseOptions, AcceptsKnownPairs) {
  const auto result = parse_options({"--if", "vcan_host", "--scenario", "idle"},
                                    {"if", "scenario", "node"}, {"if"});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.values.at("if"), "vcan_host");
  EXPECT_EQ(result.values.at("scenario"), "idle");
  EXPECT_EQ(result.values.count("node"), 0U);
}

TEST(ParseOptions, RejectsUnknownMissingRepeatedAndDangling) {
  EXPECT_FALSE(parse_options({"--bogus", "1"}, {"if"}, {}).ok);
  EXPECT_FALSE(parse_options({}, {"if"}, {"if"}).ok);
  EXPECT_FALSE(parse_options({"--if", "a", "--if", "b"}, {"if"}, {}).ok);
  EXPECT_FALSE(parse_options({"--if"}, {"if"}, {}).ok);
  EXPECT_FALSE(parse_options({"if", "a"}, {"if"}, {}).ok);
}

TEST(JsonEscape, EscapesQuotesBackslashAndControls) {
  EXPECT_EQ(json_escape("plain"), "plain");
  EXPECT_EQ(json_escape("a\"b\\c"), "a\\\"b\\\\c");
  EXPECT_EQ(json_escape("x\ny\tz"), "x\\ny\\tz");
  EXPECT_EQ(json_escape(std::string_view("\x01", 1)), "\\u0001");
}

TEST(Crc, IsoHdlcCheckValueAndCommitLayout) {
  // The catalogue check value of CRC-32/ISO-HDLC: the CRC of the ASCII bytes "123456789".
  const std::string_view check = "123456789";
  std::vector<std::uint8_t> bytes(check.begin(), check.end());
  EXPECT_EQ(lely_peer::crc32_iso_hdlc(bytes.data(), bytes.size()), 0xCBF4'3926U);
  // The commit value is the same CRC over the four words, each little-endian.
  const std::vector<std::uint8_t> words{0x01, 0x00, 0x00, 0x00, 0xEF, 0xCD, 0xAB, 0x89,
                                        0x67, 0x45, 0x23, 0x01, 0xEF, 0xBE, 0xAD, 0xDE};
  EXPECT_EQ(lely_peer::commit_crc(1, 0x89AB'CDEF, 0x0123'4567, 0xDEAD'BEEF),
            lely_peer::crc32_iso_hdlc(words.data(), words.size()));
  EXPECT_NE(lely_peer::commit_crc(1, 0x89AB'CDEF, 0x0123'4567, 0xDEAD'BEEF),
            lely_peer::commit_crc(2, 0x89AB'CDEF, 0x0123'4567, 0xDEAD'BEEF));
}

}  // namespace
