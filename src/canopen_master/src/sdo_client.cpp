// SPDX-License-Identifier: Apache-2.0
#include "canopen_master/sdo_client.hpp"

#include <utility>

namespace canopen_master {

namespace cc = canopen_codec;
using runtime_core::Duration;
using runtime_core::MonoTime;

namespace {

// CiA 301 command byte fields (bits 7-5: command specifier).
constexpr std::uint8_t kSpecifierMask = 0xE0;
constexpr std::uint8_t kUploadResponse = 0x40;    // scs 2: initiate upload response
constexpr std::uint8_t kDownloadResponse = 0x60;  // scs 3: initiate download response
constexpr std::uint8_t kAbort = 0x80;             // cs 4: abort transfer
constexpr std::uint8_t kExpedited = 0x02;         // e
constexpr std::uint8_t kSizeIndicated = 0x01;     // s

std::uint32_t le32(const cc::Frame& frame, std::size_t at) {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    v |= static_cast<std::uint32_t>(frame.data.at(at + i)) << (8U * i);
  }
  return v;
}

}  // namespace

SdoClient::SdoClient(cc::NodeId node, Duration timeout) : node_(node), timeout_(timeout) {}

bool SdoClient::push(Request request) {
  if (count_ >= kCapacity) {
    return false;
  }
  ring_.at((head_ + count_) % kCapacity) = std::move(request);
  ++count_;
  return true;
}

bool SdoClient::upload(std::uint16_t index, std::uint8_t sub, SdoDone done, std::uint64_t tag) {
  return push(Request{true, index, sub, 0, 0, std::move(done), tag});
}

bool SdoClient::download(std::uint16_t index, std::uint8_t sub, std::span<const std::uint8_t> bytes,
                         SdoDone done, std::uint64_t tag) {
  if (bytes.empty() || bytes.size() > 4) {
    return false;
  }
  std::uint32_t value = 0;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    value |= static_cast<std::uint32_t>(bytes[i]) << (8U * i);
  }
  return push(Request{false, index, sub, static_cast<std::uint8_t>(bytes.size()), value,
                      std::move(done), tag});
}

std::size_t SdoClient::cancel(std::uint64_t tag) {
  if (tag == 0) {
    return 0;
  }
  // The outstanding request (the head while active) has left already; only queued ones go.
  const std::size_t first = active_ ? 1 : 0;
  std::size_t kept = first;
  for (std::size_t k = first; k < count_; ++k) {
    Request& r = ring_.at((head_ + k) % kCapacity);
    if (r.tag == tag) {
      r = Request{};
      continue;
    }
    if (kept != k) {
      ring_.at((head_ + kept) % kCapacity) = std::move(r);
      r = Request{};
    }
    ++kept;
  }
  const std::size_t dropped = count_ - kept;
  count_ = kept;
  return dropped;
}

std::optional<cc::Frame> SdoClient::start(MonoTime now) {
  if (active_ || count_ == 0) {
    return std::nullopt;
  }
  const Request& r = ring_.at(head_);
  cc::Sdo sdo{};
  sdo.type = r.upload ? cc::SdoType::kUploadRequest : cc::SdoType::kDownloadRequest;
  sdo.address = {r.index, r.sub};
  sdo.size = r.size;
  sdo.value = r.value;
  // The request was validated when queued (size 1..4), so encoding cannot fail.
  const auto frame = cc::encode_sdo(node_, sdo);
  active_ = true;
  deadline_ = now + timeout_;
  return frame;
}

SdoCompletion SdoClient::finish(SdoStatus status, std::uint32_t abort_code, bool send_abort,
                                std::optional<cc::od::Address> abort_at) {
  Request& r = ring_.at(head_);
  SdoCompletion c{};
  c.result.node = node_.value();
  c.result.index = r.index;
  c.result.sub = r.sub;
  c.result.status = status;
  c.result.abort_code = abort_code;
  if (status == SdoStatus::kOk && r.upload) {
    c.result.size = received_size_;
    c.result.value = received_value_;
  }
  if (send_abort) {
    cc::Sdo sdo{};
    sdo.type = cc::SdoType::kAbortToServer;
    sdo.address = abort_at.value_or(cc::od::Address{r.index, r.sub});
    sdo.abort_code = abort_code;
    c.abort = cc::encode_sdo(node_, sdo);
  }
  c.done = std::move(r.done);
  r = Request{};
  head_ = (head_ + 1) % kCapacity;
  --count_;
  active_ = false;
  return c;
}

std::optional<SdoCompletion> SdoClient::on_response(const cc::Frame& frame) {
  if (!active_) {
    return std::nullopt;
  }
  const Request& r = ring_.at(head_);
  if (frame.dlc != 8) {
    return finish(SdoStatus::kProtocol, sdo_abort::kInvalidCommand, true);
  }
  const std::uint8_t command = frame.data[0];
  const std::uint8_t specifier = command & kSpecifierMask;
  const auto index = static_cast<std::uint16_t>(frame.data[1] | (frame.data[2] << 8U));
  const bool same_object = index == r.index && frame.data[3] == r.sub;

  if (specifier == kAbort) {
    if (!same_object) {
      return finish(SdoStatus::kProtocol, 0, false);
    }
    return finish(SdoStatus::kAbort, le32(frame, 4), false);
  }
  if (r.upload && specifier == kUploadResponse) {
    if ((command & kExpedited) == 0) {
      // Segmented initiate: the server now waits for segment requests of the object it named.
      // Expedited only (C8).
      return finish(SdoStatus::kProtocol, sdo_abort::kInvalidCommand, true,
                    cc::od::Address{index, frame.data[3]});
    }
    if (!same_object) {
      return finish(SdoStatus::kProtocol, 0, false);
    }
    const auto unused = static_cast<std::uint8_t>((command >> 2U) & 0x03U);
    received_size_ =
        (command & kSizeIndicated) != 0 ? static_cast<std::uint8_t>(4U - unused) : std::uint8_t{4};
    const std::uint32_t all = le32(frame, 4);
    received_value_ =
        received_size_ == 4 ? all : all & ((std::uint32_t{1} << (8U * received_size_)) - 1U);
    return finish(SdoStatus::kOk, 0, false);
  }
  if (!r.upload && specifier == kDownloadResponse) {
    if (!same_object) {
      return finish(SdoStatus::kProtocol, 0, false);
    }
    return finish(SdoStatus::kOk, 0, false);
  }
  return finish(SdoStatus::kProtocol, sdo_abort::kInvalidCommand, true);
}

std::optional<SdoCompletion> SdoClient::expire(MonoTime now) {
  if (!active_ || now < deadline_) {
    return std::nullopt;
  }
  return finish(SdoStatus::kTimeout, sdo_abort::kTimeout, true);
}

}  // namespace canopen_master
