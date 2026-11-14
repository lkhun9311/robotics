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

}  // namespace lely_peer
