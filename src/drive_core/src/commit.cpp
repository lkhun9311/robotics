// SPDX-License-Identifier: Apache-2.0
#include "drive_core/commit.hpp"

namespace drive_core {

std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFF'FFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB8'8320U : crc >> 1U;
    }
  }
  return crc ^ 0xFFFF'FFFFU;
}

std::uint32_t CommitRequest::crc() const {
  std::array<std::uint8_t, 16> bytes{};
  const std::array<std::uint32_t, 4> words{attempt_id, generation_lo, generation_hi, session_lo};
  for (std::size_t w = 0; w < words.size(); ++w) {
    for (std::size_t b = 0; b < 4; ++b) {
      bytes[(w * 4) + b] = static_cast<std::uint8_t>(words[w] >> (8U * b));
    }
  }
  return crc32_iso_hdlc(bytes.data(), bytes.size());
}

bool CommitStaging::window_open(runtime_core::MonoTime now) const {
  return (mask_ & bit(kSubAttemptId)) != 0U && now < window_end_;
}

bool CommitStaging::begin(std::uint32_t attempt_id, runtime_core::MonoTime now) {
  if (attempt_id <= committed_attempt_) {
    return false;  // D17: not greater than every committed attempt_id (0 never is).
  }
  staged_ = CommitRequest{};
  staged_.attempt_id = attempt_id;
  mask_ = bit(kSubAttemptId);
  window_end_ = now + kStagingWindow;
  return true;
}

bool CommitStaging::stage(std::uint8_t sub, std::uint32_t value, runtime_core::MonoTime now) {
  if (sub < kSubGenerationLo || sub > kSubSessionLo || !window_open(now)) {
    return false;
  }
  switch (sub) {
    case kSubGenerationLo:
      staged_.generation_lo = value;
      break;
    case kSubGenerationHi:
      staged_.generation_hi = value;
      break;
    default:
      staged_.session_lo = value;
      break;
  }
  mask_ = static_cast<std::uint8_t>(mask_ | bit(sub));
  return true;
}

std::uint32_t CommitStaging::staged(std::uint8_t sub) const {
  switch (sub) {
    case kSubGenerationLo:
      return staged_.generation_lo;
    case kSubGenerationHi:
      return staged_.generation_hi;
    case kSubSessionLo:
      return staged_.session_lo;
    case kSubAttemptId:
      return staged_.attempt_id;
    default:
      return 0;
  }
}

std::optional<CommitRequest> CommitStaging::check(std::uint32_t crc, runtime_core::MonoTime now) {
  if (mask_ != kMaskAll || !window_open(now) || crc != staged_.crc()) {
    clear();
    return std::nullopt;
  }
  return staged_;
}

void CommitStaging::committed(const CommitRequest& request) {
  committed_attempt_ = request.attempt_id;
  clear();
}

void CommitStaging::clear() {
  staged_ = CommitRequest{};
  mask_ = 0;
  window_end_ = runtime_core::MonoTime{};
}

void CommitStaging::new_boot() {
  clear();
  committed_attempt_ = 0;
}

}  // namespace drive_core
