// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "canopen_codec/od.hpp"
#include "drive_core/exec_tag.hpp"
#include "drive_core/validity.hpp"

// Two-fragment command assembly (contract D4, D6, D7, D8). Works on decoded RPDO1/RPDO2; the
// dispatcher decodes frames. Holds no CiA 402 state: it reports an integrity violation and the
// drive logic decides what to do (quick stop, reason `integrity`).
namespace drive_core {

// One actuatable command: an RPDO1 and an RPDO2 of the same tag and seq.
struct Command {
  std::int32_t target_position{};  // µrad
  std::uint32_t valid_until_us{};
  std::uint16_t command_seq{};
  std::uint16_t exec_tag{};

  friend bool operator==(const Command&, const Command&) = default;
};

// Fragments dropped, by reason.
struct Counters {
  std::uint32_t other_tag{};     // Tag is not the current one (or no tag is current).
  std::uint32_t stale_seq{};     // Seq not newer than the last actuated seq.
  std::uint32_t not_valid{};     // Pair whose valid_until_us is expired or beyond the lease.
  std::uint32_t missing_half{};  // A half without its partner at SYNC (an ordinary gap).
  std::uint32_t blocked{};       // Current-tag fragment after an integrity violation.
  std::uint32_t superseded{};    // Complete pair older than the newest complete pair at SYNC.

  friend bool operator==(const Counters&, const Counters&) = default;
};

enum class OfferResult : std::uint8_t {
  kAccepted,
  kDuplicate,  // Identical to a fragment already held; harmless, nothing changes.
  kDroppedOtherTag,
  kDroppedStaleSeq,
  kDroppedSuperseded,  // The window is full and this seq is the oldest in it.
  kBlocked,
  kIntegrityViolation,  // Detected now, at ingestion (D8).
};

struct AssemblyResult {
  std::optional<Command> command;
  bool integrity_violation{false};
  Counters dropped;  // Drops since the previous on_sync, including those made at this SYNC.
};

// Rules, for the tag current at the issuer (D8):
// - Fragments of another tag are dropped and counted.
// - A fragment whose seq is not newer than the last actuated seq is dropped (seq16_newer), except
//   that a fragment with the last actuated seq and a payload different from the actuated one is
//   an integrity violation (two payloads with the same seq), while an identical one is stale.
// - Fragments pair only on equal tag and seq. A window may hold halves of several seqs (after a
//   lost SYNC, D7); that is not a violation. Two different payloads of the same half and seq in
//   one window are an integrity violation.
// - The window holds at most kWindowSlots seqs. When a new seq arrives and every slot is used,
//   the oldest seq (the new one included) is dropped and counted as superseded.
// - An integrity violation latches: no command is produced and current-tag fragments are
//   blocked until the tag changes (issuer epoch) or reset() is called.
// - At SYNC (on_sync) the newest complete pair (seq16_newer) is actuated if validity_open(until,
//   now, lease), otherwise dropped as not_valid; older complete pairs are dropped as superseded.
//   A lone half is an ordinary gap. The window then clears.
// Any change of the issuer's epoch (commit, revoke, expiry, reset communication, boot) clears
// the window and the last actuated seq. The issuer must outlive the assembler.
class CommandAssembler {
 public:
  // Three lost SYNC periods (D7's watchdog bound) hold about three commands; the rest is margin.
  static constexpr std::size_t kWindowSlots = 8;

  explicit CommandAssembler(const ExecTagIssuer& issuer, std::uint32_t lease_us = kLeaseUs);

  OfferResult offer(const canopen_codec::od::Rpdo1& fragment);
  OfferResult offer(const canopen_codec::od::Rpdo2& fragment);
  AssemblyResult on_sync(std::uint32_t now_us);

  // True while an integrity violation is latched for the current tag.
  bool integrity_violation() const { return violated_; }
  // NMT reset communication (D12): clears the window, last actuated seq and the latch.
  void reset();

 private:
  void follow_issuer();
  OfferResult admit(std::uint16_t seq, bool matches_actuated);
  OfferResult violate();
  // One seq of the window; a half is present when its has_* flag is set.
  struct Slot {
    bool used{false};
    bool has_rpdo1{false};
    bool has_rpdo2{false};
    std::uint16_t seq{};
    canopen_codec::od::Rpdo1 rpdo1{};
    canopen_codec::od::Rpdo2 rpdo2{};
  };
  // The window slot for `seq`, making one if needed; nullptr when `seq` was dropped as the oldest.
  Slot* slot_for(std::uint16_t seq);
  void clear_window();

  const ExecTagIssuer& issuer_;
  std::uint32_t lease_us_;
  std::uint64_t epoch_;
  std::array<Slot, kWindowSlots> window_{};
  std::optional<Command> last_actuated_;
  bool violated_{false};
  Counters dropped_;
};

}  // namespace drive_core
