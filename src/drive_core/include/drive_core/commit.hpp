// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "runtime_core/time.hpp"

// The staging half of the execution commit transaction (contract D17). The master writes
// 0x2010:04 attempt_id (opens a staging window and clears the mask), then 0x2010:01-03, then
// 0x2011 = CRC-32 over the staged values. This class holds the staging, the window and the
// highest committed attempt_id; tag allocation, the CiA 402 state rules and the policy latch are
// the drive's (drive.cpp). Every time value is drive-local monotonic time from the caller.
namespace drive_core {

// The staging window after the attempt_id write (D17). Open while now < opened + window.
inline constexpr runtime_core::Duration kStagingWindow{1'000'000'000};  // 1 s

// CRC-32/ISO-HDLC (the zlib/Ethernet CRC: reflected polynomial 0x04C11DB7, init and final XOR
// 0xFFFFFFFF). Check value over the ASCII bytes "123456789" is 0xCBF43926.
std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size);

// One staged (or committed) request: the four 32-bit values the CRC covers.
struct CommitRequest {
  std::uint32_t attempt_id{};
  std::uint32_t generation_lo{};
  std::uint32_t generation_hi{};
  std::uint32_t session_lo{};

  std::uint64_t generation() const {
    return (static_cast<std::uint64_t>(generation_hi) << 32U) | generation_lo;
  }
  // D17: CRC-32/ISO-HDLC over the little-endian bytes of attempt_id, generation_lo,
  // generation_hi, session_lo, in that order (16 bytes).
  std::uint32_t crc() const;

  friend bool operator==(const CommitRequest&, const CommitRequest&) = default;
};

class CommitStaging {
 public:
  // Sub-indices of 0x2010 (D14).
  static constexpr std::uint8_t kSubGenerationLo = 1;
  static constexpr std::uint8_t kSubGenerationHi = 2;
  static constexpr std::uint8_t kSubSessionLo = 3;
  static constexpr std::uint8_t kSubAttemptId = 4;

  // 0x2010:04. Accepted only if attempt_id is greater than every attempt_id committed in this
  // boot (0 never is); then the staging restarts with only the attempt bit set and the window
  // opens. A refused write changes nothing.
  bool begin(std::uint32_t attempt_id, runtime_core::MonoTime now);
  // 0x2010:01-03. Accepted only while a staging window is open; sets the sub's mask bit.
  bool stage(std::uint8_t sub, std::uint32_t value, runtime_core::MonoTime now);
  // The staged value of 0x2010:sub (1..4); 0 when nothing is staged there.
  std::uint32_t staged(std::uint8_t sub) const;

  // 0x2011 checks: the mask is complete, the window is open and `crc` matches the staged values.
  // On success returns the staged request (not yet committed: the caller allocates the tag and
  // then calls committed()). On failure returns nullopt and clears the staging.
  std::optional<CommitRequest> check(std::uint32_t crc, runtime_core::MonoTime now);
  // Records a successful commit of `request`: it becomes the highest committed attempt_id and
  // the staging clears.
  void committed(const CommitRequest& request);
  // Drops the staging (a refused commit, reset communication).
  void clear();

  // attempt_id of the last successful commit in this boot; 0 before the first (0x2016).
  std::uint32_t committed_attempt() const { return committed_attempt_; }
  // NMT reset communication (D12): the staging clears, the highest committed attempt_id stays.
  void reset_communication() { clear(); }
  // A new boot (reset node, D3): everything starts over.
  void new_boot();

 private:
  static constexpr std::uint8_t kMaskAll = 0x0F;
  static std::uint8_t bit(std::uint8_t sub) { return static_cast<std::uint8_t>(1U << (sub - 1U)); }
  bool window_open(runtime_core::MonoTime now) const;

  CommitRequest staged_{};
  std::uint8_t mask_{0};
  runtime_core::MonoTime window_end_{};
  std::uint32_t committed_attempt_{0};
};

}  // namespace drive_core
