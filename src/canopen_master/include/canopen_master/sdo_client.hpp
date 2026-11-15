// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "canopen_codec/codec.hpp"
#include "canopen_master/events.hpp"
#include "runtime_core/time.hpp"

// The expedited SDO client of one node (C8): one request outstanding at a time, the others wait
// in a FIFO. A request that is not answered within the timeout fails and the client sends the
// server abort 0x05040000. A response that does not answer the request (other index or subindex,
// other command, a segmented initiate) fails the request as a protocol error; when the server may
// still be inside a transfer (segmented initiate, an unknown or wrong command, a wrong DLC) the
// client sends abort 0x05040001; a segmented initiate is aborted at the object the server named.
namespace canopen_master {

namespace sdo_abort {
inline constexpr std::uint32_t kTimeout = 0x0504'0000;         // SDO protocol timed out.
inline constexpr std::uint32_t kInvalidCommand = 0x0504'0001;  // Command specifier not valid.
}  // namespace sdo_abort

// A finished request: its result, the callback to call with it and the abort frame to send.
struct SdoCompletion {
  SdoResult result;
  SdoDone done;
  std::optional<canopen_codec::Frame> abort;
};

class SdoClient {
 public:
  // Requests accepted and not yet finished, the outstanding one included; more are refused.
  static constexpr std::size_t kCapacity = 16;

  SdoClient(canopen_codec::NodeId node, runtime_core::Duration timeout);

  // Queue a request. false (and `done` is never called) when the queue is full, or for a
  // download of other than 1..4 bytes.
  bool upload(std::uint16_t index, std::uint8_t sub, SdoDone done);
  bool download(std::uint16_t index, std::uint8_t sub, std::span<const std::uint8_t> bytes,
                SdoDone done);

  // When nothing is outstanding and a request waits: makes it outstanding with its deadline at
  // now + timeout and returns its frame.
  std::optional<canopen_codec::Frame> start(runtime_core::MonoTime now);

  // A frame on this node's SDO response COB-ID. std::nullopt when nothing is outstanding.
  std::optional<SdoCompletion> on_response(const canopen_codec::Frame& frame);

  // The outstanding request when its deadline has passed at `now`.
  std::optional<SdoCompletion> expire(runtime_core::MonoTime now);

  std::size_t accepted() const { return count_; }
  bool outstanding() const { return active_; }

 private:
  struct Request {
    bool upload{};
    std::uint16_t index{};
    std::uint8_t sub{};
    std::uint8_t size{};
    std::uint32_t value{};
    SdoDone done;
  };

  bool push(Request request);
  // The abort, when sent, names `abort_at`, or the request's own object when it is empty.
  SdoCompletion finish(SdoStatus status, std::uint32_t abort_code, bool send_abort,
                       std::optional<canopen_codec::od::Address> abort_at = std::nullopt);

  canopen_codec::NodeId node_;
  runtime_core::Duration timeout_;
  std::array<Request, kCapacity> ring_{};
  std::size_t head_{};
  std::size_t count_{};
  bool active_{};
  runtime_core::MonoTime deadline_{};
  std::uint8_t received_size_{};
  std::uint32_t received_value_{};
};

}  // namespace canopen_master
