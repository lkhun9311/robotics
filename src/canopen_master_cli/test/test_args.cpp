// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <string_view>
#include <vector>

#include "args.hpp"

namespace {

namespace cli = canopen_master_cli;
using Verb = canopen_backend::RawCommand::Verb;

}  // namespace

TEST(CliArgs, OptionsNodesAndScenarios) {
  const std::vector<std::string_view> args{"--if",  "vcan_host",  "--nodes",
                                           "1-3,5", "--scenario", "stream"};
  const cli::Options o =
      cli::parse_options(args, {"if", "nodes", "scenario", "trace"}, {"if", "nodes", "scenario"});
  ASSERT_TRUE(o.ok) << o.error;
  EXPECT_FALSE(cli::parse_options({"--if", "a", "--if", "b"}, {"if"}, {}).ok);
  EXPECT_FALSE(cli::parse_options({"--bogus", "a"}, {"if"}, {}).ok);
  EXPECT_FALSE(cli::parse_options({"--if"}, {"if"}, {}).ok);
  EXPECT_FALSE(cli::parse_options({}, {"if"}, {"if"}).ok);

  const cli::NodeList n = cli::parse_nodes(o.values.at("nodes"));
  ASSERT_TRUE(n.ok) << n.error;
  EXPECT_EQ(n.nodes, (std::vector<std::uint8_t>{1, 2, 3, 5}));
  EXPECT_FALSE(cli::parse_nodes("0").ok);
  EXPECT_FALSE(cli::parse_nodes("3-1").ok);
  EXPECT_FALSE(cli::parse_nodes("1,1").ok);
  EXPECT_FALSE(cli::parse_nodes("128").ok);

  const cli::ScenarioOption s = cli::parse_scenario("stream");
  ASSERT_TRUE(s.ok);
  EXPECT_EQ(s.scenario, cli::Scenario::kStream);
  EXPECT_STREQ(cli::to_string(cli::Scenario::kActivateHal), "activate_hal");
  EXPECT_FALSE(cli::parse_scenario("idle").ok);
  EXPECT_FALSE(cli::parse_number("12x", 0, 100).ok);
  EXPECT_FALSE(cli::parse_number("101", 0, 100).ok);
  EXPECT_EQ(cli::parse_number("100", 0, 100).value, 100U);
}

TEST(CliArgs, RequiredCapabilities) {
  EXPECT_FALSE(cli::parse_required("none").required.device_side_expiry);
  const cli::RequiredOption all = cli::parse_required("all");
  ASSERT_TRUE(all.ok);
  EXPECT_TRUE(all.required.device_side_expiry && all.required.execution_generation &&
              all.required.local_stop && all.required.apply_ack && all.required.source_timestamp);
  const cli::RequiredOption two = cli::parse_required("local_stop,apply_ack");
  ASSERT_TRUE(two.ok) << two.error;
  EXPECT_TRUE(two.required.local_stop && two.required.apply_ack);
  EXPECT_FALSE(two.required.device_side_expiry || two.required.execution_generation ||
               two.required.source_timestamp);
  EXPECT_FALSE(cli::parse_required("local_stop,local_stop").ok);
  EXPECT_FALSE(cli::parse_required("warp_drive").ok);
  EXPECT_FALSE(cli::parse_required("").ok);
}

TEST(CliArgs, ConsoleCommands) {
  cli::ConsoleLine c = cli::parse_console("nmt 2 0x81");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kNmt);
  EXPECT_EQ(c.command.node, 2);
  EXPECT_EQ(c.command.nmt, canopen_codec::NmtCommand::kResetNode);

  c = cli::parse_console("read 2 0x1018 1 u32");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kSdoUpload);
  EXPECT_EQ(c.command.index, 0x1018);
  EXPECT_EQ(c.command.sub, 1);

  c = cli::parse_console("write 2 0x1017 0 u16 400");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kSdoDownload);
  EXPECT_EQ(c.command.size, 2);
  EXPECT_EQ(c.command.bytes[0], 0x90);
  EXPECT_EQ(c.command.bytes[1], 0x01);

  c = cli::parse_console("write 2 0x2000 0 i16 -2");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.bytes[0], 0xFE);
  EXPECT_EQ(c.command.bytes[1], 0xFF);

  c = cli::parse_console("sync step");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kSync);
  EXPECT_EQ(c.command.sync, canopen_master::SyncControl::kStep);

  c = cli::parse_console("hb 2 0");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kHeartbeatConsumer);
  EXPECT_EQ(c.command.consumer.ns, 0);

  c = cli::parse_console("rxpdo 2 0x182 0x6041:0:16 0x6064:0:32:s");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kRegisterPdo);
  EXPECT_EQ(c.command.cob, 0x182U);
  ASSERT_EQ(c.command.field_count, 2);
  EXPECT_EQ(c.command.fields[1].index, 0x6064);
  EXPECT_EQ(c.command.fields[1].bit_length, 32);
  EXPECT_TRUE(c.command.fields[1].is_signed);
  EXPECT_FALSE(c.command.fields[0].is_signed);

  c = cli::parse_console("send 0x202 01ff");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.verb, Verb::kSend);
  EXPECT_EQ(c.command.frame.cob_id, 0x202);
  EXPECT_EQ(c.command.frame.dlc, 2);
  EXPECT_EQ(c.command.frame.data[1], 0xFF);
  c = cli::parse_console("send 0x080 -");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.command.frame.dlc, 0);

  c = cli::parse_console("quit");
  ASSERT_TRUE(c.ok);
  EXPECT_TRUE(c.quit);
}

TEST(CliArgs, BadConsoleLinesAreRefused) {
  for (const char* line : {"",
                           "bogus",
                           "quit now",
                           "nmt 2",
                           "nmt 2 3",
                           "nmt 0 1",
                           "read 2 0x1000 0 f32",
                           "write 2 0x1017 0 u8 256",
                           "write 2 0x1017 0 i8 -129",
                           "sync later",
                           "hb 2 70000",
                           "rxpdo 2 0x182",
                           "rxpdo 2 0x182 0x6041:0:0",
                           "rxpdo 2 0x182 0x6041:0:40 0x6064:0:32",
                           "rxpdo 2 0x182 0x6041:0:16:u",
                           "send 0x800 00",
                           "send 0x202 0",
                           "send 0x202 zz",
                           "send 0x202 000102030405060708",
                           "read  2 0x1000 0 u32"}) {
    EXPECT_FALSE(cli::parse_console(line).ok) << "'" << line << "'";
  }
  EXPECT_NE(cli::json_escape("a\"b\\c\n").find("\\\""), std::string::npos);
  EXPECT_EQ(cli::json_escape(std::string_view("\x01", 1)), "\\u0001");
}
