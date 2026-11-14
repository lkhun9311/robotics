// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <vector>

#include "lely_peer/args.hpp"

namespace {

using lely_peer::ConsoleCommand;
using lely_peer::json_escape;
using lely_peer::parse_console;
using lely_peer::parse_nodes;
using lely_peer::parse_number;
using lely_peer::parse_options;
using lely_peer::parse_statusword_source;
using lely_peer::StatuswordSource;

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

TEST(StatuswordSource, TpdoAndSdo) {
  const auto tpdo = parse_statusword_source("tpdo");
  EXPECT_TRUE(tpdo.ok);
  EXPECT_EQ(tpdo.source, StatuswordSource::kTpdo);
  const auto sdo = parse_statusword_source("sdo");
  EXPECT_TRUE(sdo.ok);
  EXPECT_EQ(sdo.source, StatuswordSource::kSdo);
}

TEST(StatuswordSource, RejectsOtherWords) {
  for (const std::string_view word : {"", "SDO", "tpdo1", "rpdo"}) {
    const auto parsed = parse_statusword_source(word);
    EXPECT_FALSE(parsed.ok) << word;
    EXPECT_FALSE(parsed.error.empty()) << word;
  }
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

TEST(Console, ParsesEachVerb) {
  const ConsoleCommand nmt = parse_console("nmt 3 129");
  ASSERT_TRUE(nmt.ok) << nmt.error;
  EXPECT_EQ(nmt.verb, ConsoleCommand::Verb::kNmt);
  EXPECT_EQ(nmt.node, 3);
  EXPECT_EQ(nmt.value, 129);

  const ConsoleCommand cw = parse_console("cw 3 0x80");
  ASSERT_TRUE(cw.ok) << cw.error;
  EXPECT_EQ(cw.verb, ConsoleCommand::Verb::kControlword);
  EXPECT_EQ(cw.value, 0x80);

  const ConsoleCommand read = parse_console("read 2 0x1018 1 u32");
  ASSERT_TRUE(read.ok) << read.error;
  EXPECT_EQ(read.verb, ConsoleCommand::Verb::kRead);
  EXPECT_EQ(read.node, 2);
  EXPECT_EQ(read.index, 0x1018);
  EXPECT_EQ(read.sub, 1);
  EXPECT_EQ(read.type, ConsoleCommand::Type::kU32);

  const ConsoleCommand write = parse_console("write 3 0x6060 0 i8 -2");
  ASSERT_TRUE(write.ok) << write.error;
  EXPECT_EQ(write.verb, ConsoleCommand::Verb::kWrite);
  EXPECT_EQ(write.type, ConsoleCommand::Type::kI8);
  EXPECT_EQ(write.value, -2);

  const ConsoleCommand quit = parse_console("quit");
  ASSERT_TRUE(quit.ok) << quit.error;
  EXPECT_EQ(quit.verb, ConsoleCommand::Verb::kQuit);
}

TEST(Console, RejectsBadCommands) {
  for (const std::string_view bad :
       {"", "jump 3", "nmt 3", "nmt 0 1", "nmt 128 1", "nmt 3 256", "cw 3 0x10000",
        "read 3 0x1000 0", "read 3 0x1000 0 u64", "read 3 0x10000 0 u8", "read 3 0x1000 256 u8",
        "write 3 0x6060 0 i8 128", "write 3 0x6060 0 u8 -1", "write 3 0x6060 0 u16 70000",
        "write 3 0x6060 0 i32 2147483648", "read 3 0x1000 0 u8 5", "quit now", "nmt 3 1 "}) {
    const ConsoleCommand c = parse_console(bad);
    EXPECT_FALSE(c.ok) << "'" << bad << "'";
    EXPECT_FALSE(c.error.empty()) << "'" << bad << "'";
  }
}

}  // namespace
