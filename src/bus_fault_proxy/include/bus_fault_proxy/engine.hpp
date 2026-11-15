// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "bus_fault_proxy/rules.hpp"
#include "canopen_codec/codec.hpp"
#include "runtime_core/time.hpp"

namespace bus_fault_proxy {

// A frame to write onto the interface of the direction's destination.
struct Out {
  Dir dir{Dir::kH2D};
  canopen_codec::Frame frame;
};

// Per-rule counters. `matched` counts the frames the rule acted on (direction, identifier,
// `every` and `window` all held); the others count what the action then did.
struct RuleCounts {
  std::uint64_t matched{0};
  std::uint64_t dropped{0};
  std::uint64_t delayed{0};
  std::uint64_t duplicated{0};
  std::uint64_t corrupted{0};
  std::uint64_t reordered{0};  // A held frame released behind its successor.
  std::uint64_t flushed{0};    // A held reorder frame released by the 20 ms hold limit.
};

// Pure logic: the caller supplies every instant and performs every read and write. Time must
// not decrease between calls. The first rule whose predicates all hold wins; no rule means
// the frame is forwarded at once. Frames that fall due are returned before the new frame's own.
class Engine {
 public:
  static constexpr std::int64_t kReorderHoldMs = 20;

  Engine(std::vector<Rule> rules, runtime_core::MonoTime epoch);

  std::vector<Out> on_frame(Dir dir, const canopen_codec::Frame& frame, runtime_core::MonoTime now);
  // Releases every held frame whose due time has come, in due-time order.
  std::vector<Out> poll(runtime_core::MonoTime now);
  std::optional<runtime_core::MonoTime> next_due() const;
  const std::vector<RuleCounts>& counts() const { return counts_; }

 private:
  struct Held {
    runtime_core::MonoTime due;
    std::uint64_t seq;
    Out out;
    std::size_t rule;
    bool reorder;
  };

  void release_due(runtime_core::MonoTime now, std::vector<Out>& out);
  bool applies(std::size_t index, Dir dir, const canopen_codec::Frame& frame,
               runtime_core::MonoTime now);

  std::vector<Rule> rules_;
  runtime_core::MonoTime epoch_;
  std::vector<RuleCounts> counts_;
  std::vector<std::uint64_t> seen_;  // Per rule: frames of its direction and identifier so far.
  std::vector<Held> held_;
  std::uint64_t next_seq_{0};
};

}  // namespace bus_fault_proxy
