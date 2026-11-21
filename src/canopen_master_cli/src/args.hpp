// SPDX-License-Identifier: Apache-2.0
// Command line and console parsing of canopen_master_cli.
// No bus here, so these are unit-tested on their own.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "canopen_backend/worker.hpp"
#include "runtime_core/capability.hpp"

namespace canopen_master_cli {

struct Options {
  bool ok = false;
  std::string error;
  std::map<std::string, std::string, std::less<>> values;
};

// `--key value` pairs.
// Every key must be in `allowed`; every key in `required` must be present;
// a key may appear once.
Options parse_options(const std::vector<std::string_view>& args,
                      const std::set<std::string, std::less<>>& allowed,
                      const std::set<std::string, std::less<>>& required);

struct Number {
  bool ok = false;
  std::string error;
  std::uint64_t value = 0;
};

// A decimal number in [low, high].
Number parse_number(std::string_view text, std::uint64_t low, std::uint64_t high);

struct NodeList {
  bool ok = false;
  std::string error;
  std::vector<std::uint8_t> nodes;  // Ascending, no duplicates.
};

// "1-6", "2", "1,3-6": CANopen node IDs 1..127.
NodeList parse_nodes(std::string_view text);

enum class Scenario : std::uint8_t { kConsole, kActivateHal, kArm, kStream };

struct ScenarioOption {
  bool ok = false;
  std::string error;
  Scenario scenario = Scenario::kConsole;
};

ScenarioOption parse_scenario(std::string_view text);
const char* to_string(Scenario scenario);

struct RequiredOption {
  bool ok = false;
  std::string error;
  runtime_core::Requirements required{};
};

// The capabilities the activation gate requires: "none", "all", or a comma list of
// device_side_expiry, execution_generation, local_stop, apply_ack, source_timestamp.
RequiredOption parse_required(std::string_view text);

struct TargetOption {
  bool ok = false;
  std::string error;
  std::array<double, 6> position{};
};

// stream --target: six joint positions (rad), comma separated, each finite and in [-6, 6].
TargetOption parse_target(std::string_view text);

// One console line (stdin), parsed in full before anything is done:
//   nmt <node> <cs>                            NMT command cs (1, 2, 0x80, 0x81, 0x82) to node
//   read <node> <index> <sub> <type>           SDO upload
//   write <node> <index> <sub> <type> <value>  SDO download (value in the type's range)
//   sync pause|resume|step                     SYNC production (C5)
//   hb <node> <ms>                             heartbeat consumer time of node; 0 disables (C7)
//   rxpdo <node> <cob> <field>...              decode PDOs on cob as these fields (C5), each
//                                              index:sub:bits or index:sub:bits:s (signed),
//                                              1 to 8 fields, 64 bits at most in total
//   send <cob> <hex>                           one frame as given; <hex> is 0-8 bytes as hex
//                                              digit pairs, or - for none
//   quit                                       end once every command before it is done
// node 1..127; numbers decimal or 0x hex (values may be negative); type u8 u16 u32 i8 i16 i32.
struct ConsoleLine {
  bool ok = false;
  std::string error;
  bool quit = false;
  canopen_backend::RawCommand command{};  // Its id is set by the caller.
};

ConsoleLine parse_console(std::string_view line);

// Escapes a string for a JSON string literal (quotes not included).
std::string json_escape(std::string_view text);

}  // namespace canopen_master_cli
