// SPDX-License-Identifier: Apache-2.0
#include "args.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace canopen_master_cli {

namespace cb = canopen_backend;
namespace cc = canopen_codec;
namespace cm = canopen_master;

Options parse_options(const std::vector<std::string_view>& args,
                      const std::set<std::string, std::less<>>& allowed,
                      const std::set<std::string, std::less<>>& required) {
  Options out;
  for (std::size_t i = 0; i < args.size(); i += 2) {
    const std::string_view key = args[i];
    if (key.size() < 3 || key.substr(0, 2) != "--") {
      out.error = "expected --option, got '" + std::string(key) + "'";
      return out;
    }
    const std::string name(key.substr(2));
    if (allowed.find(name) == allowed.end()) {
      out.error = "unknown option --" + name;
      return out;
    }
    if (i + 1 >= args.size()) {
      out.error = "--" + name + " needs a value";
      return out;
    }
    if (!out.values.emplace(name, std::string(args[i + 1])).second) {
      out.error = "--" + name + " given twice";
      return out;
    }
  }
  for (const auto& name : required) {
    if (out.values.find(name) == out.values.end()) {
      out.error = "missing --" + name;
      return out;
    }
  }
  out.ok = true;
  return out;
}

Number parse_number(std::string_view text, std::uint64_t low, std::uint64_t high) {
  Number out;
  std::uint64_t value = 0;
  const char* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (text.empty() || ec != std::errc() || ptr != end) {
    out.error = "not a decimal number: '" + std::string(text) + "'";
    return out;
  }
  if (value < low || value > high) {
    out.error = std::string(text) + " is outside [" + std::to_string(low) + ", " +
                std::to_string(high) + "]";
    return out;
  }
  out.ok = true;
  out.value = value;
  return out;
}

NodeList parse_nodes(std::string_view text) {
  NodeList out;
  std::set<std::uint8_t> nodes;
  while (true) {
    const std::size_t comma = text.find(',');
    const std::string_view item = text.substr(0, comma);
    const std::size_t dash = item.find('-');
    const Number first = parse_number(item.substr(0, dash), 1, 127);
    const Number last =
        dash == std::string_view::npos ? first : parse_number(item.substr(dash + 1), 1, 127);
    if (!first.ok || !last.ok) {
      out.error = "bad node list item '" + std::string(item) +
                  "': " + (first.ok ? last.error : first.error);
      return out;
    }
    if (last.value < first.value) {
      out.error = "descending range '" + std::string(item) + "'";
      return out;
    }
    for (std::uint64_t n = first.value; n <= last.value; ++n) {
      if (!nodes.insert(static_cast<std::uint8_t>(n)).second) {
        out.error = "node " + std::to_string(n) + " listed twice";
        return out;
      }
    }
    if (comma == std::string_view::npos) {
      break;
    }
    text = text.substr(comma + 1);
  }
  out.nodes.assign(nodes.begin(), nodes.end());
  out.ok = true;
  return out;
}

namespace {

constexpr std::array<std::pair<std::string_view, Scenario>, 4> kScenarios{{
    {"console", Scenario::kConsole},
    {"activate_hal", Scenario::kActivateHal},
    {"arm", Scenario::kArm},
    {"stream", Scenario::kStream},
}};

}  // namespace

ScenarioOption parse_scenario(std::string_view text) {
  ScenarioOption out;
  for (const auto& [name, scenario] : kScenarios) {
    if (name == text) {
      out.ok = true;
      out.scenario = scenario;
      return out;
    }
  }
  out.error =
      "unknown scenario '" + std::string(text) + "' (known: console, activate_hal, arm, stream)";
  return out;
}

const char* to_string(Scenario scenario) {
  for (const auto& [name, s] : kScenarios) {
    if (s == scenario) {
      return name.data();
    }
  }
  return "unknown";
}

RequiredOption parse_required(std::string_view text) {
  RequiredOption out;
  runtime_core::Requirements& r = out.required;
  if (text == "none") {
    out.ok = true;
    return out;
  }
  if (text == "all") {
    r = runtime_core::Requirements{true, true, true, true, true};
    out.ok = true;
    return out;
  }
  while (true) {
    const std::size_t comma = text.find(',');
    const std::string_view item = text.substr(0, comma);
    bool* flag = nullptr;
    if (item == "device_side_expiry") {
      flag = &r.device_side_expiry;
    } else if (item == "execution_generation") {
      flag = &r.execution_generation;
    } else if (item == "local_stop") {
      flag = &r.local_stop;
    } else if (item == "apply_ack") {
      flag = &r.apply_ack;
    } else if (item == "source_timestamp") {
      flag = &r.source_timestamp;
    }
    if (flag == nullptr) {
      out.error = "unknown capability '" + std::string(item) +
                  "' (known: device_side_expiry, execution_generation, local_stop, apply_ack, "
                  "source_timestamp; or none, all)";
      return out;
    }
    if (*flag) {
      out.error = "capability '" + std::string(item) + "' listed twice";
      return out;
    }
    *flag = true;
    if (comma == std::string_view::npos) {
      break;
    }
    text = text.substr(comma + 1);
  }
  out.ok = true;
  return out;
}

TargetOption parse_target(std::string_view text) {
  TargetOption out;
  std::size_t n = 0;
  while (true) {
    const std::size_t comma = text.find(',');
    const std::string item(text.substr(0, comma));
    char* end = nullptr;
    const double v = item.empty() ? 0.0 : std::strtod(item.c_str(), &end);
    if (item.empty() || end != item.c_str() + item.size() || !std::isfinite(v) || v < -6.0 ||
        v > 6.0 || n >= out.position.size()) {
      out.error = "--target: expected six positions in [-6, 6] rad, comma separated";
      return out;
    }
    out.position.at(n++) = v;
    if (comma == std::string_view::npos) {
      break;
    }
    text = text.substr(comma + 1);
  }
  if (n != out.position.size()) {
    out.error = "--target: expected six positions in [-6, 6] rad, comma separated";
    return out;
  }
  out.ok = true;
  return out;
}

namespace {

// Decimal or 0x hex, optionally negative, in [low, high].
bool console_number(std::string_view text, std::int64_t low, std::int64_t high, std::int64_t& out) {
  bool negative = false;
  if (!text.empty() && text.front() == '-') {
    negative = true;
    text.remove_prefix(1);
  }
  int base = 10;
  if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
    base = 16;
    text.remove_prefix(2);
  }
  std::uint64_t magnitude = 0;
  const char* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, magnitude, base);
  if (text.empty() || ec != std::errc() || ptr != end || magnitude > (1ULL << 32U)) {
    return false;
  }
  const auto value =
      negative ? -static_cast<std::int64_t>(magnitude) : static_cast<std::int64_t>(magnitude);
  if (value < low || value > high) {
    return false;
  }
  out = value;
  return true;
}

// Space-separated tokens; an empty token (a double, leading or trailing space) is kept so the
// line is rejected rather than read loosely.
std::vector<std::string_view> tokens(std::string_view line, char sep) {
  std::vector<std::string_view> out;
  if (line.empty()) {
    return out;
  }
  while (true) {
    const std::size_t at = line.find(sep);
    out.push_back(line.substr(0, at));
    if (at == std::string_view::npos) {
      return out;
    }
    line.remove_prefix(at + 1);
  }
}

ConsoleLine fail(std::string error) {
  ConsoleLine c;
  c.error = std::move(error);
  return c;
}

bool hex_bytes(std::string_view text, cc::Frame& frame) {
  if (text == "-") {
    frame.dlc = 0;
    return true;
  }
  if (text.empty() || text.size() % 2 != 0 || text.size() > 16) {
    return false;
  }
  for (std::size_t i = 0; i < text.size(); i += 2) {
    unsigned value = 0;
    const char* first = text.data() + i;
    const auto [ptr, ec] = std::from_chars(first, first + 2, value, 16);
    if (ec != std::errc() || ptr != first + 2) {
      return false;
    }
    frame.data[i / 2] = static_cast<std::uint8_t>(value);
  }
  frame.dlc = static_cast<std::uint8_t>(text.size() / 2);
  return true;
}

}  // namespace

ConsoleLine parse_console(std::string_view line) {
  using Verb = cb::RawCommand::Verb;
  const std::vector<std::string_view> tok = tokens(line, ' ');
  if (tok.empty()) {
    return fail("empty command");
  }
  ConsoleLine c;
  cb::RawCommand& r = c.command;
  const std::string verb(tok[0]);
  const auto arity = [&](std::size_t n) { return tok.size() == n; };
  const auto node_at = [&](std::size_t i) {
    std::int64_t node = 0;
    if (!console_number(tok[i], 1, 127, node)) {
      return false;
    }
    r.node = static_cast<std::uint8_t>(node);
    return true;
  };
  if (verb == "quit") {
    if (!arity(1)) return fail("wrong argument count for 'quit'");
    c.quit = true;
  } else if (verb == "nmt") {
    if (!arity(3)) return fail("wrong argument count for 'nmt'");
    if (!node_at(1)) return fail("bad node '" + std::string(tok[1]) + "'");
    std::int64_t cs = 0;
    if (!console_number(tok[2], 0, 0xFF, cs) ||
        (cs != 0x01 && cs != 0x02 && cs != 0x80 && cs != 0x81 && cs != 0x82)) {
      return fail("bad NMT command '" + std::string(tok[2]) + "' (1, 2, 0x80, 0x81, 0x82)");
    }
    r.verb = Verb::kNmt;
    r.nmt = static_cast<cc::NmtCommand>(cs);
  } else if (verb == "read" || verb == "write") {
    const bool write = verb == "write";
    if (!arity(write ? 6 : 5)) return fail("wrong argument count for '" + verb + "'");
    if (!node_at(1)) return fail("bad node '" + std::string(tok[1]) + "'");
    std::int64_t index = 0;
    std::int64_t sub = 0;
    if (!console_number(tok[2], 0, 0xFFFF, index) || !console_number(tok[3], 0, 0xFF, sub)) {
      return fail("bad index or sub-index");
    }
    r.index = static_cast<std::uint16_t>(index);
    r.sub = static_cast<std::uint8_t>(sub);
    struct Range {
      std::string_view name;
      std::uint8_t size;
      std::int64_t low;
      std::int64_t high;
    };
    constexpr std::array<Range, 6> kTypes{{{"u8", 1, 0, 0xFF},
                                           {"u16", 2, 0, 0xFFFF},
                                           {"u32", 4, 0, 0xFFFF'FFFF},
                                           {"i8", 1, -0x80, 0x7F},
                                           {"i16", 2, -0x8000, 0x7FFF},
                                           {"i32", 4, -0x8000'0000LL, 0x7FFF'FFFF}}};
    const Range* range = nullptr;
    for (const Range& t : kTypes) {
      if (t.name == tok[4]) range = &t;
    }
    if (range == nullptr) return fail("bad type '" + std::string(tok[4]) + "'");
    r.verb = write ? Verb::kSdoDownload : Verb::kSdoUpload;
    if (write) {
      std::int64_t value = 0;
      if (!console_number(tok[5], range->low, range->high, value)) {
        return fail("bad value '" + std::string(tok[5]) + "' for " + std::string(range->name));
      }
      const auto bits = static_cast<std::uint64_t>(value);
      r.size = range->size;
      for (std::size_t b = 0; b < range->size; ++b) {
        r.bytes.at(b) = static_cast<std::uint8_t>(bits >> (8U * b));
      }
    }
  } else if (verb == "sync") {
    if (!arity(2)) return fail("wrong argument count for 'sync'");
    r.verb = Verb::kSync;
    if (tok[1] == "pause") {
      r.sync = cm::SyncControl::kPause;
    } else if (tok[1] == "resume") {
      r.sync = cm::SyncControl::kResume;
    } else if (tok[1] == "step") {
      r.sync = cm::SyncControl::kStep;
    } else {
      return fail("bad sync control '" + std::string(tok[1]) + "' (pause, resume, step)");
    }
  } else if (verb == "hb") {
    if (!arity(3)) return fail("wrong argument count for 'hb'");
    if (!node_at(1)) return fail("bad node '" + std::string(tok[1]) + "'");
    std::int64_t ms = 0;
    if (!console_number(tok[2], 0, 65535, ms)) {
      return fail("bad consumer time '" + std::string(tok[2]) + "' (0..65535 ms)");
    }
    r.verb = Verb::kHeartbeatConsumer;
    r.consumer = runtime_core::milliseconds(ms);
  } else if (verb == "rxpdo") {
    if (tok.size() < 4 || tok.size() > 3 + cb::kMaxEventPdoFields) {
      return fail("wrong argument count for 'rxpdo'");
    }
    if (!node_at(1)) return fail("bad node '" + std::string(tok[1]) + "'");
    std::int64_t cob = 0;
    if (!console_number(tok[2], 1, 0x7FF, cob)) {
      return fail("bad COB-ID '" + std::string(tok[2]) + "'");
    }
    r.verb = Verb::kRegisterPdo;
    r.cob = static_cast<std::uint32_t>(cob);
    unsigned total = 0;
    for (std::size_t i = 3; i < tok.size(); ++i) {
      const std::vector<std::string_view> part = tokens(tok[i], ':');
      std::int64_t index = 0;
      std::int64_t sub = 0;
      std::int64_t bits = 0;
      if ((part.size() != 3 && part.size() != 4) || !console_number(part[0], 0, 0xFFFF, index) ||
          !console_number(part[1], 0, 0xFF, sub) || !console_number(part[2], 1, 64, bits) ||
          (part.size() == 4 && part[3] != "s")) {
        return fail("bad PDO field '" + std::string(tok[i]) + "' (index:sub:bits[:s])");
      }
      total += static_cast<unsigned>(bits);
      r.fields.at(r.field_count++) =
          cm::PdoField{static_cast<std::uint16_t>(index), static_cast<std::uint8_t>(sub),
                       static_cast<std::uint8_t>(bits), part.size() == 4};
    }
    if (total > 64) return fail("PDO fields hold " + std::to_string(total) + " bits, at most 64");
  } else if (verb == "send") {
    if (!arity(3)) return fail("wrong argument count for 'send'");
    std::int64_t cob = 0;
    if (!console_number(tok[1], 0, 0x7FF, cob)) {
      return fail("bad COB-ID '" + std::string(tok[1]) + "'");
    }
    r.verb = Verb::kSend;
    r.frame.cob_id = static_cast<std::uint16_t>(cob);
    if (!hex_bytes(tok[2], r.frame)) {
      return fail("bad frame data '" + std::string(tok[2]) + "' (0-8 hex byte pairs, or -)");
    }
  } else {
    return fail("unknown command '" + verb + "'");
  }
  c.ok = true;
  return c;
}

std::string json_escape(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::array<char, 7> buf{};
          std::snprintf(buf.data(), buf.size(), "\\u%04x", static_cast<unsigned>(c));
          out += buf.data();
        } else {
          out += c;
        }
    }
  }
  return out;
}

}  // namespace canopen_master_cli
