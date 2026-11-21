// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "runtime_core/time.hpp"
#include "vdrive_bank/bank.hpp"

// A bank in logic time with a record log.
// Expected frames in the tests are written from CiA 301 byte by byte; the codec builds only the
// frames the bank receives.
namespace vdrive_bank::test {

namespace cc = canopen_codec;
using runtime_core::Duration;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

inline constexpr std::array<std::uint32_t, kNodeCount> kBootIds{11, 22, 33, 44, 55, 66};
inline constexpr MonoTime kStart{5'000'000'000};

inline cc::NodeId id(std::uint8_t node) { return cc::NodeId::make(node).value(); }

// A CiA 301 frame from its identifier and data bytes; the DLC is the number of bytes.
inline cc::Frame raw(std::uint16_t cob_id, std::initializer_list<std::uint8_t> bytes) {
  cc::Frame f{cob_id, static_cast<std::uint8_t>(bytes.size()), {}};
  std::copy(bytes.begin(), bytes.end(), f.data.begin());
  return f;
}

struct Rig {
  MonoTime now{kStart};
  Bank bank{kBootIds, kStart};
  std::vector<BankRecord> records;

  Rig() {
    bank.set_sink([this](const BankRecord& r) { records.push_back(r); });
  }

  std::vector<cc::Frame> power_on() { return bank.power_on(now); }
  std::vector<cc::Frame> at(Duration since_start) {
    now = kStart + since_start;
    return bank.tick(now);
  }
  std::vector<cc::Frame> step(Duration d) {
    now = now + d;
    return bank.tick(now);
  }
  std::vector<cc::Frame> frame(const cc::Frame& f) { return bank.on_frame(f, now); }
  Bank::InjectResult inject(std::string_view command) { return bank.inject(command, now); }
  // NMT command (CiA 301: 0x000, DLC 2, command specifier, node-ID; 0 addresses all nodes).
  std::vector<cc::Frame> nmt(std::uint8_t command, std::uint8_t node) {
    return frame(raw(0x000, {command, node}));
  }
  // Powered on, then past the boot quarantine.
  void ready() {
    power_on();
    at(milliseconds(1));
  }

  std::size_t count(BankRecord::Kind kind) const {
    return static_cast<std::size_t>(std::count_if(
        records.begin(), records.end(), [kind](const BankRecord& r) { return r.kind == kind; }));
  }
  std::optional<BankRecord> last(BankRecord::Kind kind) const {
    for (auto it = records.rbegin(); it != records.rend(); ++it) {
      if (it->kind == kind) {
        return *it;
      }
    }
    return std::nullopt;
  }
};

// The heartbeat frames among `frames` (0x701..0x77F).
inline std::vector<cc::Frame> heartbeats(const std::vector<cc::Frame>& frames) {
  std::vector<cc::Frame> out;
  std::copy_if(frames.begin(), frames.end(), std::back_inserter(out),
               [](const cc::Frame& f) { return f.cob_id > 0x700 && f.cob_id <= 0x77F; });
  return out;
}

}  // namespace vdrive_bank::test
