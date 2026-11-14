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

void CommandAssembler::clear_window() { window_.fill(Slot{}); }

std::size_t CommandAssembler::held_fragments() const {
  std::size_t held = 0;
  for (const Slot& slot : window_) {
    held += (slot.has_rpdo1 ? 1U : 0U) + (slot.has_rpdo2 ? 1U : 0U);
  }
  return held;
}

void CommandAssembler::reset() {
  epoch_ = issuer_.epoch();
  clear_window();
  last_actuated_.reset();
  violated_ = false;
}

OfferResult CommandAssembler::violate() {
  violated_ = true;
  violation_pending_ = true;
  clear_window();
  return OfferResult::kIntegrityViolation;
}

CommandAssembler::Slot* CommandAssembler::slot_for(std::uint16_t seq) {
  Slot* free = nullptr;
  Slot* oldest = &window_.front();
  for (Slot& slot : window_) {
    if (!slot.used) {
      if (free == nullptr) {
        free = &slot;
      }
      continue;
    }
    if (slot.seq == seq) {
      return &slot;
    }
    if (!oldest->used || runtime_core::seq16_newer(oldest->seq, slot.seq)) {
      oldest = &slot;
    }
  }
  if (free == nullptr) {
    // Full (so `oldest` is a used slot): the oldest seq gives way, possibly the arriving one.
    ++dropped_.superseded;
    if (!runtime_core::seq16_newer(seq, oldest->seq)) {
      return nullptr;
    }
    free = oldest;
  }
  *free = Slot{};
  free->used = true;
  free->seq = seq;
  return free;
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
  Slot* slot = slot_for(fragment.command_seq);
  if (slot == nullptr) {
    return OfferResult::kDroppedSuperseded;
  }
  if (slot->has_rpdo1) {
    return same(slot->rpdo1, fragment) ? OfferResult::kDuplicate : violate();
  }
  slot->has_rpdo1 = true;
  slot->rpdo1 = fragment;
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
  Slot* slot = slot_for(fragment.command_seq);
  if (slot == nullptr) {
    return OfferResult::kDroppedSuperseded;
  }
  if (slot->has_rpdo2) {
    return same(slot->rpdo2, fragment) ? OfferResult::kDuplicate : violate();
  }
  slot->has_rpdo2 = true;
  slot->rpdo2 = fragment;
  return OfferResult::kAccepted;
}

AssemblyResult CommandAssembler::on_sync(std::uint32_t now_us) {
  follow_issuer();
  AssemblyResult result;
  result.integrity_violation = violated_ || violation_pending_;
  violation_pending_ = false;
  if (!violated_) {
    // A slot holds one tag (the current one) and one seq, so a complete slot is a pair.
    const Slot* newest = nullptr;
    for (const Slot& slot : window_) {
      if (!slot.used) {
        continue;
      }
      if (!slot.has_rpdo1 || !slot.has_rpdo2) {
        ++dropped_.missing_half;
        continue;
      }
      if (newest != nullptr) {
        ++dropped_.superseded;  // One of the two complete pairs is not the newest.
        if (!runtime_core::seq16_newer(slot.seq, newest->seq)) {
          continue;
        }
      }
      newest = &slot;
    }
    if (newest != nullptr) {
      if (validity_open(newest->rpdo2.valid_until_us, now_us, lease_us_)) {
        result.command = Command{newest->rpdo1.target_position, newest->rpdo2.valid_until_us,
                                 newest->seq, newest->rpdo2.exec_tag};
        last_actuated_ = result.command;
      } else {
        ++dropped_.not_valid;
      }
    }
  }
  clear_window();
  result.dropped = dropped_;
  dropped_ = Counters{};
  return result;
}

}  // namespace drive_core
