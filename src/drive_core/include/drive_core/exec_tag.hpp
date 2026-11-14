// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

#include "runtime_core/time.hpp"

// Drive-issued execution identity (contract D3, D12). The drive, not the host, allocates the
// 16-bit exec_tag, so the side that checks a tag is the side that made it. Every time value is
// drive-local monotonic time passed in by the caller; nothing here reads a clock.
namespace drive_core {

// A tag is valid for at most this long (D3). It stays below half a 16-bit command_seq wrap at
// 100 Hz (327 s), so seq16_newer is unambiguous within one tag.
inline constexpr runtime_core::Duration kTagLifetime{300'000'000'000};  // 5 min
// After boot-up the drive refuses commits this long (D3). It exceeds the 48 ms lease, so no
// command of a previous boot can still have an open deadline once the first tag exists.
inline constexpr runtime_core::Duration kBootQuarantine{100'000'000};  // 100 ms

// The first tag of a boot, derived from boot_id (0x2013). Never 0. Tags are then issued in
// strictly increasing order from this value up to 0xFFFF; a start near the top leaves few tags
// for that boot, which is the price of an unpredictable start (D3's residual-risk argument).
std::uint16_t exec_tag_start(std::uint32_t boot_id);

enum class CommitRefusal : std::uint8_t {
  kBootQuarantine,  // Within kBootQuarantine of boot-up.
  kExhausted,       // The next tag would wrap past 0xFFFF; only a new boot (reset node) helps.
};

// Owns the tag allocator of one drive boot.
//
// - commit() issues the next tag and revokes the previous one at once.
// - reset_communication() (NMT reset communication, D12) revokes the current tag but keeps the
//   allocator and boot_id, so a tag value is never issued twice within one boot.
// - boot() (reset node / process restart) starts a new boot: new start value, new quarantine.
// - tick() revokes the current tag once kTagLifetime has elapsed since its commit.
//
// epoch() changes on every event that changes which tag is current (commit, revoke, expiry,
// reset, boot), including the case where a new boot happens to issue a tag value equal to an
// earlier boot's; consumers compare epochs, never tag values, to notice the change.
//
// Times use checked runtime_core arithmetic, which throws std::overflow_error only for values
// near +/-292 years; this class is on the commit (SDO) path, not on the frame path.
class ExecTagIssuer {
 public:
  ExecTagIssuer(std::uint32_t boot_id, runtime_core::MonoTime boot_time,
                runtime_core::Duration lifetime = kTagLifetime,
                runtime_core::Duration quarantine = kBootQuarantine);

  // Issues the next tag, or refuses (abort 0x08000022 at the SDO layer, D3). A refused
  // commit also revokes the current tag: the host asked to leave it, and an exhausted drive must
  // not keep running on its last tag. generation and session are recorded with the tag.
  struct CommitOutcome {
    std::optional<std::uint16_t> tag;
    std::optional<CommitRefusal> refusal;
  };
  CommitOutcome commit_ex(std::uint64_t generation, std::uint32_t session,
                          runtime_core::MonoTime now);
  std::optional<std::uint16_t> commit(std::uint64_t generation, std::uint32_t session,
                                      runtime_core::MonoTime now) {
    return commit_ex(generation, session, now).tag;
  }

  // The current tag as of the last commit/revoke/tick, or 0 when none is current.
  std::uint16_t current() const { return current_; }
  std::uint64_t generation() const { return generation_; }
  std::uint32_t session() const { return session_; }
  std::uint32_t boot_id() const { return boot_id_; }
  std::uint64_t epoch() const { return epoch_; }

  // Revokes the current tag if its lifetime has ended (expired at exact equality).
  void tick(runtime_core::MonoTime now);
  // Policy stop, normal disable (D11): the current tag dies; the allocator is untouched.
  void revoke();
  // NMT reset communication (D12): revokes the tag, clears generation and session, keeps the
  // allocator and boot_id.
  void reset_communication();
  // Reset node / process restart: a new boot with a new boot_id.
  void boot(std::uint32_t boot_id, runtime_core::MonoTime now);

 private:
  runtime_core::Duration lifetime_;
  runtime_core::Duration quarantine_;
  std::uint32_t boot_id_{};
  runtime_core::MonoTime commits_allowed_from_{};
  std::uint32_t next_{};  // Next tag to issue; 0x10000 means exhausted for this boot.
  std::uint16_t current_{0};
  runtime_core::MonoTime expires_at_{};
  std::uint64_t generation_{0};
  std::uint32_t session_{0};
  std::uint64_t epoch_{0};
};

}  // namespace drive_core
