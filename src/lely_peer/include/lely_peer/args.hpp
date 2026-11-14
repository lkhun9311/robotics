// SPDX-License-Identifier: Apache-2.0
// Command-line and output helpers shared by lely_peer and probe_slave. No Lely here, so the
// helpers are unit-tested without a bus.
#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace lely_peer {

struct Options {
  bool ok = false;
  std::string error;
  std::map<std::string, std::string, std::less<>> values;
};

// Parses `--key value` pairs. Every key must be in `allowed`; every key in `required` must be
// present; a key may appear once.
Options parse_options(const std::vector<std::string_view>& args,
                      const std::set<std::string, std::less<>>& allowed,
                      const std::set<std::string, std::less<>>& required);

struct NodeList {
  bool ok = false;
  std::string error;
  std::vector<std::uint8_t> nodes;  // ascending, no duplicates
};

// Parses "1-6", "3", "1,3-6": CANopen node IDs 1..127.
NodeList parse_nodes(std::string_view text);

struct Number {
  bool ok = false;
  std::string error;
  std::uint64_t value = 0;
};

// Parses a decimal number in [low, high].
Number parse_number(std::string_view text, std::uint64_t low, std::uint64_t high);

// Escapes a string for a JSON string literal (quotes not included).
std::string json_escape(std::string_view text);

}  // namespace lely_peer
