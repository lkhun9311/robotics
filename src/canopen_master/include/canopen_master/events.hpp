// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <functional>
#include <variant>

#include "canopen_codec/codec.hpp"
#include "runtime_core/time.hpp"

// What the master core reports to its caller. Events are queued in the order they happen and the
// caller drains them (Master::drain_events).
namespace canopen_master {

enum class MasterEventKind : std::uint8_t {
  kState,             // StateDetail: a heartbeat reported a different NMT state.
  kBoot,              // A boot-up message (heartbeat byte 0x00).
  kBootTimeout,       // No first heartbeat within the boot timeout (C7).
  kHeartbeatLost,     // No heartbeat within the consumer time after the last one (C7).
  kHeartbeatResumed,  // A heartbeat after kHeartbeatLost or kBootTimeout.
  kSdoOk,             // SdoResult
  kSdoAbort,          // SdoResult: the server aborted; abort_code.
  kSdoTimeout,        // SdoResult: no response within the SDO timeout.
  kSdoProtocol,       // SdoResult: a response that does not answer the request (C8).
  kEmcy,              // EmcyDetail
  kSync,              // SyncDetail: the master produced a SYNC.
  kPdo,               // PdoDetail: a received PDO decoded through its registered layout.
  kPdoLengthError,    // PdoLengthDetail: a registered PDO with the wrong DLC; not decoded.
};

enum class SdoStatus : std::uint8_t { kOk, kAbort, kTimeout, kProtocol };

// The outcome of one expedited SDO request.
struct SdoResult {
  std::uint8_t node{};
  std::uint16_t index{};
  std::uint8_t sub{};
  SdoStatus status{};
  std::uint8_t size{};         // kOk upload: 1..4 bytes received; otherwise 0.
  std::uint32_t value{};       // kOk upload: the received bytes, little-endian, zero-extended.
  std::uint32_t abort_code{};  // kAbort: the server's code; kTimeout and kProtocol: the code the
                               // master sent to the server, 0 when it sent none.

  friend bool operator==(const SdoResult&, const SdoResult&) = default;
};

// Called once per accepted request, from inside Master::on_frame or Master::tick. It may queue
// further requests or NMT commands; it must not call on_frame or tick (that throws).
using SdoDone = std::function<void(const SdoResult&)>;

struct StateDetail {
  canopen_codec::NmtState state{};
};

using EventDetail = std::variant<std::monostate, StateDetail, SdoResult>;

struct MasterEvent {
  MasterEventKind kind{};
  std::uint8_t node{};  // The node the event is about; 0 for kSync.
  runtime_core::MonoTime at{};
  EventDetail detail;
};

}  // namespace canopen_master
