// SPDX-License-Identifier: Apache-2.0
#pragma once

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

  friend bool operator==(const Counters&, const Counters&) = default;
};

enum class OfferResult : std::uint8_t {
  kAccepted,
  kDuplicate,  // Identical to a fragment already held; harmless, nothing changes.
  kDroppedOtherTag,
  kDroppedStaleSeq,
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
// - Within one SYNC window every current-tag fragment must carry one seq; a second seq (in
//   either half) is an integrity violation. Two different payloads of the same half and seq in
//   one window are an integrity violation too.
// - An integrity violation latches: no command is produced and current-tag fragments are
//   blocked until the tag changes (issuer epoch) or reset() is called.
// - At SYNC (on_sync) a complete pair whose seq is newer is actuated if validity_open(until,
//   now, lease); otherwise it is dropped. A lone half is an ordinary gap. The window then clears.
// Any change of the issuer's epoch (commit, revoke, expiry, reset communication, boot) clears
// the window and the last actuated seq. The issuer must outlive the assembler.
class CommandAssembler {
 public:
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

  const ExecTagIssuer& issuer_;
  std::uint32_t lease_us_;
  std::uint64_t epoch_;
  std::optional<canopen_codec::od::Rpdo1> rpdo1_;
  std::optional<canopen_codec::od::Rpdo2> rpdo2_;
  std::optional<Command> last_actuated_;
  bool violated_{false};
  Counters dropped_;
};

}  // namespace drive_core
