// SPDX-License-Identifier: Apache-2.0
// Command-line and output helpers shared by lely_peer and probe_slave. No Lely here, so the
// helpers are unit-tested without a bus.
#pragma once

#include <cstddef>
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

// One line of the `console` scenario (stdin), parsed in full before anything is done:
//   nmt <node> <cs>                              NMT command specifier cs (0..255) to node
//   cw <node> <value>                            controlword (0..0xFFFF) by RPDO3
//   read <node> <index> <sub> <type>             SDO upload
//   write <node> <index> <sub> <type> <value>    SDO download (value in the type's range)
//   quit                                         end the scenario (scenario_done)
// node 1..127; numbers decimal or 0x hex (values may be negative); type u8 u16 u32 i8 i16 i32.
struct ConsoleCommand {
  enum class Verb : std::uint8_t { kNmt, kControlword, kRead, kWrite, kQuit };
  enum class Type : std::uint8_t { kU8, kU16, kU32, kI8, kI16, kI32 };
  bool ok = false;
  std::string error;
  Verb verb = Verb::kQuit;
  std::uint8_t node = 0;
  std::uint16_t index = 0;
  std::uint8_t sub = 0;
  Type type = Type::kU8;
  std::int64_t value = 0;  // nmt: cs; cw: controlword; write: the value
};

ConsoleCommand parse_console(std::string_view line);

// Escapes a string for a JSON string literal (quotes not included).
std::string json_escape(std::string_view text);

// CRC-32/ISO-HDLC (reflected polynomial 0x04C11DB7, init and final XOR 0xFFFFFFFF), written here
// from the algorithm's parameters so the peer does not share the drive's implementation.
std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size);

// The drive extension's commit value (0x2011): the CRC over the little-endian bytes of
// attempt_id, generation_lo, generation_hi and session_lo, in that order.
std::uint32_t commit_crc(std::uint32_t attempt_id, std::uint32_t generation_lo,
                         std::uint32_t generation_hi, std::uint32_t session_lo);

}  // namespace lely_peer
