// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string_view>

#include "runtime_core/trace.hpp"

namespace rc = runtime_core;

TEST(JsonlTraceSink, WritesOneObjectPerLine) {
  std::ostringstream out;
  rc::JsonlTraceSink sink{out, "fake_host_mono"};
  sink.emit(rc::MonoTime{42}, "command_sent",
            {{"seq", std::int64_t{7}}, {"verdict", std::string_view{"SEND"}}});
  sink.emit(rc::MonoTime{43}, "scenario_end", {});
  EXPECT_EQ(out.str(),
            "{\"t_ns\":42,\"clock\":\"fake_host_mono\",\"kind\":\"command_sent\","
            "\"seq\":7,\"verdict\":\"SEND\"}\n"
            "{\"t_ns\":43,\"clock\":\"fake_host_mono\",\"kind\":\"scenario_end\"}\n");
}

TEST(JsonlTraceSink, EscapesQuotesBackslashesAndControlCharacters) {
  EXPECT_EQ(rc::json_escape("a\"b\\c\nd"), "a\\\"b\\\\c\\u000ad");
}

TEST(JsonlTraceSink, WritesNegativeIntegers) {
  std::ostringstream out;
  rc::JsonlTraceSink sink{out, "fake_host_mono"};
  sink.emit(rc::MonoTime{0}, "x", {{"delta", std::int64_t{-5}}});
  EXPECT_EQ(out.str(), "{\"t_ns\":0,\"clock\":\"fake_host_mono\",\"kind\":\"x\",\"delta\":-5}\n");
}
