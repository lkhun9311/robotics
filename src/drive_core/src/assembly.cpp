// SPDX-License-Identifier: Apache-2.0
#include "drive_core/assembly.hpp"

#include "runtime_core/seq.hpp"

namespace drive_core {

namespace {

using canopen_codec::od::Rpdo1;
using canopen_codec::od::Rpdo2;

bool same(const Rpdo1& a, const Rpdo1& b) {
  return a.target_position == b.target_position && a.command_seq == b.command_seq &&
         a.exec_tag == b.exec_tag;
}

bool same(const Rpdo2& a, const Rpdo2& b) {
  return a.valid_until_us == b.valid_until_us && a.command_seq == b.command_seq &&
         a.exec_tag == b.exec_tag;
}

}  // namespace

CommandAssembler::CommandAssembler(const ExecTagIssuer& issuer, std::uint32_t lease_us)
    : issuer_(issuer), lease_us_(lease_us), epoch_(issuer.epoch()) {}

void CommandAssembler::follow_issuer() {
  if (issuer_.epoch() != epoch_) {
    reset();
  }
}

void CommandAssembler::reset() {
  epoch_ = issuer_.epoch();
  rpdo1_.reset();
  rpdo2_.reset();
  last_actuated_.reset();
  violated_ = false;
}

OfferResult CommandAssembler::violate() {
  violated_ = true;
  rpdo1_.reset();
  rpdo2_.reset();
  return OfferResult::kIntegrityViolation;
}

// Checks shared by both halves, after the tag matched. kAccepted means "go on with the
// half-specific checks"; anything else is final.
OfferResult CommandAssembler::admit(std::uint16_t seq, bool matches_actuated) {
  if (violated_) {
    ++dropped_.blocked;
    return OfferResult::kBlocked;
  }
  if (last_actuated_) {
    if (seq == last_actuated_->command_seq) {
      if (!matches_actuated) {
        return violate();  // A second payload for a seq that was already actuated.
      }
      ++dropped_.stale_seq;
      return OfferResult::kDroppedStaleSeq;
    }
    if (!runtime_core::seq16_newer(seq, last_actuated_->command_seq)) {
      ++dropped_.stale_seq;
      return OfferResult::kDroppedStaleSeq;
    }
  }
  std::optional<std::uint16_t> held;
  if (rpdo1_) {
    held = rpdo1_->command_seq;
  } else if (rpdo2_) {
    held = rpdo2_->command_seq;
  }
  if (held && *held != seq) {
    return violate();  // The window would name two commands.
  }
  return OfferResult::kAccepted;
}

OfferResult CommandAssembler::offer(const Rpdo1& fragment) {
  follow_issuer();
  const std::uint16_t tag = issuer_.current();
  if (tag == 0 || fragment.exec_tag != tag) {
    ++dropped_.other_tag;
    return OfferResult::kDroppedOtherTag;
  }
  const bool matches_actuated =
      last_actuated_ && last_actuated_->target_position == fragment.target_position;
  if (const auto r = admit(fragment.command_seq, matches_actuated); r != OfferResult::kAccepted) {
    return r;
  }
  if (rpdo1_) {
    return same(*rpdo1_, fragment) ? OfferResult::kDuplicate : violate();
  }
  rpdo1_ = fragment;
  return OfferResult::kAccepted;
}

OfferResult CommandAssembler::offer(const Rpdo2& fragment) {
  follow_issuer();
  const std::uint16_t tag = issuer_.current();
  if (tag == 0 || fragment.exec_tag != tag) {
    ++dropped_.other_tag;
    return OfferResult::kDroppedOtherTag;
  }
  const bool matches_actuated =
      last_actuated_ && last_actuated_->valid_until_us == fragment.valid_until_us;
  if (const auto r = admit(fragment.command_seq, matches_actuated); r != OfferResult::kAccepted) {
    return r;
  }
  if (rpdo2_) {
    return same(*rpdo2_, fragment) ? OfferResult::kDuplicate : violate();
  }
  rpdo2_ = fragment;
  return OfferResult::kAccepted;
}

AssemblyResult CommandAssembler::on_sync(std::uint32_t now_us) {
  follow_issuer();
  AssemblyResult result;
  result.integrity_violation = violated_;
  if (!violated_) {
    // Both halves held means same tag (the current one) and same seq (admit enforces it).
    if (rpdo1_ && rpdo2_) {
      if (validity_open(rpdo2_->valid_until_us, now_us, lease_us_)) {
        result.command = Command{rpdo1_->target_position, rpdo2_->valid_until_us,
                                 rpdo2_->command_seq, rpdo2_->exec_tag};
        last_actuated_ = result.command;
      } else {
        ++dropped_.not_valid;
      }
    } else if (rpdo1_ || rpdo2_) {
      ++dropped_.missing_half;
    }
  }
  rpdo1_.reset();
  rpdo2_.reset();
  result.dropped = dropped_;
  dropped_ = Counters{};
  return result;
}

}  // namespace drive_core
