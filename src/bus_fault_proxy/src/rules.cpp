// SPDX-License-Identifier: Apache-2.0
#include "bus_fault_proxy/rules.hpp"

#include <charconv>
#include <cstddef>
#include <optional>

namespace bus_fault_proxy {
namespace {

std::vector<std::string_view> split_words(std::string_view line) {
  std::vector<std::string_view> words;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) ++i;
    const std::size_t start = i;
    while (i < line.size() && line[i] != ' ' && line[i] != '\t' && line[i] != '\r') ++i;
    if (i > start) words.push_back(line.substr(start, i - start));
  }
  return words;
}

// Decimal, or hex with a 0x prefix; the whole word must be consumed.
std::optional<std::uint32_t> parse_number(std::string_view word) {
  int base = 10;
  if (word.size() > 2 && word[0] == '0' && (word[1] == 'x' || word[1] == 'X')) {
    base = 16;
    word.remove_prefix(2);
  }
  std::uint32_t value = 0;
  const auto* end = word.data() + word.size();
  const auto result = std::from_chars(word.data(), end, value, base);
  if (word.empty() || result.ec != std::errc{} || result.ptr != end) return std::nullopt;
  return value;
}

class LineParser {
 public:
  explicit LineParser(std::vector<std::string_view> words) : words_(std::move(words)) {}

  // Returns an error message (without the line number) or an empty string.
  std::string parse(Rule& rule) {
    if (words_.empty()) return "empty rule";
    if (words_[0] == "h2d") {
      rule.dir = Dir::kH2D;
    } else if (words_[0] == "d2h") {
      rule.dir = Dir::kD2H;
    } else {
      return unknown(words_[0], "direction (h2d or d2h)");
    }
    pos_ = 1;
    if (pos_ >= words_.size()) return "missing identifier after the direction";
    if (std::string e = parse_id(rule); !e.empty()) return e;
    if (peek() == "every") {
      ++pos_;
      if (std::string e = number("every", 1, 0xFFFFFF, rule.every); !e.empty()) return e;
      if (peek() == "offset") {
        ++pos_;
        if (std::string e = number("offset", 0, rule.every - 1, rule.offset); !e.empty()) return e;
      }
    }
    if (peek() == "window") {
      ++pos_;
      if (std::string e = number("window from", 0, 0xFFFFFFF, rule.window_from_ms); !e.empty()) {
        return e;
      }
      if (std::string e = number("window to", 0, 0xFFFFFFF, rule.window_to_ms); !e.empty()) {
        return e;
      }
      if (rule.window_to_ms <= rule.window_from_ms) {
        return "word 'window': the window is empty (to must be greater than from)";
      }
      rule.has_window = true;
    }
    if (std::string e = parse_action(rule); !e.empty()) return e;
    if (pos_ < words_.size()) {
      return unknown(words_[pos_], "end of the rule (trailing word)");
    }
    return {};
  }

 private:
  std::string_view peek() const { return pos_ < words_.size() ? words_[pos_] : std::string_view{}; }

  static std::string unknown(std::string_view word, const char* expected) {
    return "unknown word '" + std::string(word) + "', expected " + expected;
  }

  std::string number(const char* what, std::uint32_t lo, std::uint32_t hi, std::uint32_t& out) {
    if (pos_ >= words_.size()) return std::string("word '") + what + "': a number is missing";
    const auto value = parse_number(words_[pos_]);
    if (!value) {
      return "word '" + std::string(words_[pos_]) + "': not a number (" + what + ")";
    }
    if (*value < lo || *value > hi) {
      return "word '" + std::string(words_[pos_]) + "': " + what + " out of range " +
             std::to_string(lo) + ".." + std::to_string(hi);
    }
    out = *value;
    ++pos_;
    return {};
  }

  std::string parse_id(Rule& rule) {
    const std::string_view word = words_[pos_];
    const std::size_t slash = word.find('/');
    if (slash == std::string_view::npos) {
      return "word '" + std::string(word) + "': expected <id>/<mask>";
    }
    const auto id = parse_number(word.substr(0, slash));
    const auto mask = parse_number(word.substr(slash + 1));
    if (!id || !mask || *id > 0x7FF || *mask > 0x7FF) {
      return "word '" + std::string(word) + "': id and mask must be numbers up to 0x7FF";
    }
    rule.id = static_cast<std::uint16_t>(*id);
    rule.mask = static_cast<std::uint16_t>(*mask);
    ++pos_;
    return {};
  }

  std::string parse_action(Rule& rule) {
    if (pos_ >= words_.size()) return "missing action";
    const std::string_view word = words_[pos_++];
    std::uint32_t value = 0;
    if (word == "drop") {
      rule.action = Action::kDrop;
    } else if (word == "dup") {
      rule.action = Action::kDup;
    } else if (word == "reorder") {
      rule.action = Action::kReorder;
    } else if (word == "delay") {
      rule.action = Action::kDelay;
      return number("delay ms", 0, kMaxDelayMs, rule.delay_ms);
    } else if (word == "corrupt" || word == "dup_corrupt") {
      rule.action = word == "corrupt" ? Action::kCorrupt : Action::kDupCorrupt;
      if (std::string e = number("byte index", 0, kMaxCorruptByte, value); !e.empty()) return e;
      rule.byte = static_cast<std::uint8_t>(value);
      if (std::string e = number("xor mask", 0, 255, value); !e.empty()) return e;
      rule.xor_mask = static_cast<std::uint8_t>(value);
    } else {
      return unknown(word, "an action (delay, drop, dup, dup_corrupt, corrupt, reorder)");
    }
    return {};
  }

  std::vector<std::string_view> words_;
  std::size_t pos_{0};
};

}  // namespace

ParseResult parse_rules(std::string_view text) {
  ParseResult result;
  std::size_t line_no = 0;
  std::size_t start = 0;
  while (start <= text.size()) {
    ++line_no;
    std::size_t end = text.find('\n', start);
    if (end == std::string_view::npos) end = text.size();
    std::string_view line = text.substr(start, end - start);
    start = end + 1;
    if (const std::size_t hash = line.find('#'); hash != std::string_view::npos) {
      line = line.substr(0, hash);
    }
    auto words = split_words(line);
    if (words.empty()) continue;
    Rule rule;
    if (std::string e = LineParser(std::move(words)).parse(rule); !e.empty()) {
      result.rules.clear();
      result.error = "line " + std::to_string(line_no) + ": " + e;
      return result;
    }
    result.rules.push_back(rule);
  }
  return result;
}

}  // namespace bus_fault_proxy
