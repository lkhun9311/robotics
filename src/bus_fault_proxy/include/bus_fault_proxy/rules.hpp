// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// Fault rules of the bus fault proxy, one per line:
//   <dir> <id>/<mask> [every <N> [offset <K>]] [window <from_ms> <to_ms>] <action> [args]
// dir is h2d or d2h; id and mask are decimal or 0x-hex, at most 0x7FF; a frame matches when
// (cob_id & mask) == (id & mask).
// `every` keeps the matching frames whose 0-based occurrence number n (per rule, counted over the
// frames of its direction and identifier) has n mod N == K.
// `window` keeps frames received in [from, to) milliseconds after the proxy's start.
// Both predicates apply to every action.
// Actions: delay <ms> (0..80), drop, dup, dup_corrupt <byte> <xor>, corrupt <byte> <xor>, reorder.
// '#' starts a comment.
namespace bus_fault_proxy {

enum class Dir : std::uint8_t { kH2D, kD2H };

enum class Action : std::uint8_t { kDelay, kDrop, kDup, kDupCorrupt, kCorrupt, kReorder };

constexpr unsigned kMaxDelayMs = 80;
constexpr unsigned kMaxCorruptByte = 7;

struct Rule {
  Dir dir{Dir::kH2D};
  std::uint16_t id{};
  std::uint16_t mask{};
  std::uint32_t every{1};
  std::uint32_t offset{0};
  bool has_window{false};
  std::uint32_t window_from_ms{0};
  std::uint32_t window_to_ms{0};
  Action action{Action::kDrop};
  std::uint32_t delay_ms{0};
  std::uint8_t byte{0};
  std::uint8_t xor_mask{0};
};

struct ParseResult {
  std::vector<Rule> rules;
  std::string error;  // Empty when ok; otherwise names the line and the word.
};

// All-or-nothing: on an error `rules` is empty.
ParseResult parse_rules(std::string_view text);

}  // namespace bus_fault_proxy
