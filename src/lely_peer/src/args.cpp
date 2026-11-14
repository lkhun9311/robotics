// SPDX-License-Identifier: Apache-2.0
#include "lely_peer/args.hpp"

#include <array>
#include <charconv>
#include <cstdio>

namespace lely_peer {

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

std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFF'FFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB8'8320U : crc >> 1U;
    }
  }
  return ~crc;
}

std::uint32_t commit_crc(std::uint32_t attempt_id, std::uint32_t generation_lo,
                         std::uint32_t generation_hi, std::uint32_t session_lo) {
  const std::array<std::uint32_t, 4> words{attempt_id, generation_lo, generation_hi, session_lo};
  std::array<std::uint8_t, 16> bytes{};
  for (std::size_t w = 0; w < words.size(); ++w) {
    for (std::size_t b = 0; b < 4; ++b) {
      bytes.at((w * 4) + b) = static_cast<std::uint8_t>(words.at(w) >> (8U * b));
    }
  }
  return crc32_iso_hdlc(bytes.data(), bytes.size());
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
std::vector<std::string_view> console_tokens(std::string_view line) {
  std::vector<std::string_view> out;
  if (line.empty()) {
    return out;
  }
  while (true) {
    const std::size_t space = line.find(' ');
    out.push_back(line.substr(0, space));
    if (space == std::string_view::npos) {
      return out;
    }
    line.remove_prefix(space + 1);
  }
}

}  // namespace

ConsoleCommand parse_console(std::string_view line) {
  using Verb = ConsoleCommand::Verb;
  using Type = ConsoleCommand::Type;
  ConsoleCommand c;
  const std::vector<std::string_view> tok = console_tokens(line);
  if (tok.empty()) {
    c.error = "empty command";
    return c;
  }
  std::size_t arity = 0;
  if (tok[0] == "quit") {
    c.verb = Verb::kQuit;
    arity = 1;
  } else if (tok[0] == "nmt") {
    c.verb = Verb::kNmt;
    arity = 3;
  } else if (tok[0] == "cw") {
    c.verb = Verb::kControlword;
    arity = 3;
  } else if (tok[0] == "read") {
    c.verb = Verb::kRead;
    arity = 5;
  } else if (tok[0] == "write") {
    c.verb = Verb::kWrite;
    arity = 6;
  } else {
    c.error = "unknown command '" + std::string(tok[0]) + "'";
    return c;
  }
  if (tok.size() != arity) {
    c.error = "wrong argument count for '" + std::string(tok[0]) + "'";
    return c;
  }
  if (c.verb == Verb::kQuit) {
    c.ok = true;
    return c;
  }
  std::int64_t node = 0;
  if (!console_number(tok[1], 1, 127, node)) {
    c.error = "bad node '" + std::string(tok[1]) + "'";
    return c;
  }
  c.node = static_cast<std::uint8_t>(node);
  if (c.verb == Verb::kNmt || c.verb == Verb::kControlword) {
    const std::int64_t high = c.verb == Verb::kNmt ? 0xFF : 0xFFFF;
    if (!console_number(tok[2], 0, high, c.value)) {
      c.error = "bad value '" + std::string(tok[2]) + "'";
      return c;
    }
    c.ok = true;
    return c;
  }
  std::int64_t index = 0;
  std::int64_t sub = 0;
  if (!console_number(tok[2], 0, 0xFFFF, index) || !console_number(tok[3], 0, 0xFF, sub)) {
    c.error = "bad index or sub-index";
    return c;
  }
  c.index = static_cast<std::uint16_t>(index);
  c.sub = static_cast<std::uint8_t>(sub);
  struct Range {
    std::string_view name;
    Type type;
    std::int64_t low;
    std::int64_t high;
  };
  constexpr std::array<Range, 6> kTypes{{{"u8", Type::kU8, 0, 0xFF},
                                         {"u16", Type::kU16, 0, 0xFFFF},
                                         {"u32", Type::kU32, 0, 0xFFFF'FFFF},
                                         {"i8", Type::kI8, -0x80, 0x7F},
                                         {"i16", Type::kI16, -0x8000, 0x7FFF},
                                         {"i32", Type::kI32, -0x8000'0000LL, 0x7FFF'FFFF}}};
  const Range* range = nullptr;
  for (const Range& r : kTypes) {
    if (r.name == tok[4]) {
      range = &r;
    }
  }
  if (range == nullptr) {
    c.error = "bad type '" + std::string(tok[4]) + "'";
    return c;
  }
  c.type = range->type;
  if (c.verb == Verb::kWrite && !console_number(tok[5], range->low, range->high, c.value)) {
    c.error = "bad value '" + std::string(tok[5]) + "' for " + std::string(range->name);
    return c;
  }
  c.ok = true;
  return c;
}

}  // namespace lely_peer
