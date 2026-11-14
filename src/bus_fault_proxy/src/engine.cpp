// SPDX-License-Identifier: Apache-2.0
#include "bus_fault_proxy/engine.hpp"

#include <algorithm>
#include <utility>

namespace bus_fault_proxy {

using canopen_codec::Frame;
using runtime_core::MonoTime;

Engine::Engine(std::vector<Rule> rules, MonoTime epoch)
    : rules_(std::move(rules)), epoch_(epoch), counts_(rules_.size()), seen_(rules_.size(), 0) {}

bool Engine::applies(std::size_t index, Dir dir, const Frame& frame, MonoTime now) {
  const Rule& rule = rules_[index];
  if (rule.dir != dir || (frame.cob_id & rule.mask) != (rule.id & rule.mask)) return false;
  const std::uint64_t occurrence = seen_[index]++;
  if (occurrence % rule.every != rule.offset) return false;
  if (rule.has_window) {
    const std::int64_t since = (now - epoch_).ns;
    const std::int64_t from = static_cast<std::int64_t>(rule.window_from_ms) * 1'000'000;
    const std::int64_t to = static_cast<std::int64_t>(rule.window_to_ms) * 1'000'000;
    if (since < from || since >= to) return false;
  }
  return true;
}

void Engine::release_due(MonoTime now, std::vector<Out>& out) {
  std::vector<Held> due;
  for (auto it = held_.begin(); it != held_.end();) {
    if (it->due <= now) {
      due.push_back(*it);
      it = held_.erase(it);
    } else {
      ++it;
    }
  }
  std::sort(due.begin(), due.end(), [](const Held& a, const Held& b) {
    return a.due != b.due ? a.due < b.due : a.seq < b.seq;
  });
  for (Held& h : due) {
    if (h.reorder) ++counts_[h.rule].flushed;
    out.push_back(h.out);
  }
}

std::vector<Out> Engine::poll(MonoTime now) {
  std::vector<Out> out;
  release_due(now, out);
  return out;
}

std::optional<MonoTime> Engine::next_due() const {
  std::optional<MonoTime> best;
  for (const Held& h : held_) {
    if (!best || h.due < *best) best = h.due;
  }
  return best;
}

std::vector<Out> Engine::on_frame(Dir dir, const Frame& frame, MonoTime now) {
  std::vector<Out> out;
  release_due(now, out);
  for (std::size_t i = 0; i < rules_.size(); ++i) {
    if (!applies(i, dir, frame, now)) continue;
    const Rule& rule = rules_[i];
    RuleCounts& c = counts_[i];
    ++c.matched;
    switch (rule.action) {
      case Action::kDrop:
        ++c.dropped;
        break;
      case Action::kDelay:
        ++c.delayed;
        held_.push_back(Held{now + runtime_core::milliseconds(rule.delay_ms), next_seq_++,
                             Out{dir, frame}, i, false});
        break;
      case Action::kDup:
        ++c.duplicated;
        out.push_back(Out{dir, frame});
        out.push_back(Out{dir, frame});
        break;
      case Action::kDupCorrupt: {
        Frame copy = frame;
        out.push_back(Out{dir, frame});
        if (rule.byte < copy.dlc) {
          copy.data[rule.byte] = static_cast<std::uint8_t>(copy.data[rule.byte] ^ rule.xor_mask);
          ++c.corrupted;
        }
        ++c.duplicated;
        out.push_back(Out{dir, copy});
        break;
      }
      case Action::kCorrupt: {
        // A byte at or beyond the DLC is not on the wire; flipping it would break the
        // codec's zero-padding invariant, so such a frame is forwarded unchanged.
        Frame copy = frame;
        if (rule.byte < copy.dlc) {
          copy.data[rule.byte] = static_cast<std::uint8_t>(copy.data[rule.byte] ^ rule.xor_mask);
          ++c.corrupted;
        }
        out.push_back(Out{dir, copy});
        break;
      }
      case Action::kReorder: {
        const auto held = std::find_if(held_.begin(), held_.end(),
                                       [i](const Held& h) { return h.reorder && h.rule == i; });
        if (held == held_.end()) {
          held_.push_back(Held{now + runtime_core::milliseconds(kReorderHoldMs), next_seq_++,
                               Out{dir, frame}, i, true});
        } else {
          ++c.reordered;
          out.push_back(Out{dir, frame});
          out.push_back(held->out);
          held_.erase(held);
        }
        break;
      }
    }
    return out;
  }
  out.push_back(Out{dir, frame});
  return out;
}

}  // namespace bus_fault_proxy
