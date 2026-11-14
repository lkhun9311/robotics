// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "canopen_codec/od.hpp"  // Generated from config/od/vdrive_arm.eds at build time.

// CANopen (CiA 301) frame codec for the virtual arm drive: NMT, heartbeat, SYNC, EMCY,
// expedited SDO and the PDOs of the generated object dictionary. All multi-byte values are
// little-endian. Every decoder returns std::nullopt for a frame it does not fully understand
// (wrong COB-ID, wrong DLC, DLC above 8, unknown command); nothing is ever partially decoded.
namespace canopen_codec {

// A classic CAN frame with an 11-bit identifier. Bytes at and beyond `dlc` are zero in every
// frame this codec encodes and are ignored when decoding.
struct Frame {
  std::uint16_t cob_id{};
  std::uint8_t dlc{};
  std::array<std::uint8_t, 8> data{};

  friend bool operator==(const Frame&, const Frame&) = default;
};

// A CANopen node-ID, 1..127. Invalid IDs cannot be constructed, so no encoder can produce a
// COB-ID that collides with another service's identifier.
class NodeId {
 public:
  static constexpr std::uint8_t kMin = 1;
  static constexpr std::uint8_t kMax = 127;

  static constexpr std::optional<NodeId> make(std::uint8_t value) {
    if (value < kMin || value > kMax) {
      return std::nullopt;
    }
    return NodeId{value};
  }

  constexpr std::uint8_t value() const { return value_; }
  friend constexpr bool operator==(NodeId, NodeId) = default;

 private:
  constexpr explicit NodeId(std::uint8_t value) : value_(value) {}
  std::uint8_t value_;
};

// ---- NMT (COB-ID 0x000, DLC 2) ----

enum class NmtCommand : std::uint8_t {
  kStart = 0x01,
  kStop = 0x02,
  kEnterPreOperational = 0x80,
  kResetNode = 0x81,
  kResetCommunication = 0x82,
};

struct NmtMessage {
  NmtCommand command{};
  std::optional<NodeId> target;  // std::nullopt addresses every node (node-ID 0 on the wire).
};

Frame encode_nmt(const NmtMessage& message);
std::optional<NmtMessage> decode_nmt(const Frame& frame);

// ---- Heartbeat (COB-ID 0x700 + node, DLC 1) ----

enum class NmtState : std::uint8_t {
  kBootUp = 0x00,
  kStopped = 0x04,
  kOperational = 0x05,
  kPreOperational = 0x7F,
};

struct Heartbeat {
  NodeId node;
  NmtState state{};
};

Frame encode_heartbeat(const Heartbeat& heartbeat);
std::optional<Heartbeat> decode_heartbeat(const Frame& frame);

// ---- SYNC (COB-ID from 0x1005, DLC 0, no counter) ----

Frame encode_sync();
bool is_sync(const Frame& frame);

// ---- EMCY (COB-ID from 0x1014 + node, DLC 8) ----

struct Emcy {
  NodeId node;
  std::uint16_t error_code{};
  std::uint8_t error_register{};
  std::array<std::uint8_t, 5> manufacturer{};  // Manufacturer-specific bytes 3..7, as sent.
};

Frame encode_emcy(const Emcy& emcy);
std::optional<Emcy> decode_emcy(const Frame& frame);

// ---- Expedited SDO (request 0x600 + node, response 0x580 + node, DLC 8) ----
// Segmented and block transfers are not part of this drive's contract and do not decode; the
// drive answers such a request with abort 0x05040001 itself (drive_core).
//
// Frames reach the codec from a SocketCAN adapter (E3b). That adapter must drop remote (RTR)
// frames, extended (29-bit) frames and error frames before calling the codec: Frame carries an
// 11-bit identifier only and has no flag that could tell them apart.

enum class SdoType : std::uint8_t {
  kUploadRequest,     // client -> server
  kUploadResponse,    // server -> client, carries size and value
  kDownloadRequest,   // client -> server, carries size and value
  kDownloadResponse,  // server -> client
  kAbortToClient,     // server -> client, carries abort_code
  kAbortToServer,     // client -> server, carries abort_code
};

struct Sdo {
  SdoType type{};
  od::Address address{};
  // 1..4 for kUploadResponse and kDownloadRequest, otherwise 0. A decoded kDownloadRequest may
  // also have size 0: expedited with the size indicator clear (e = 1, s = 0, command byte 0x22),
  // size unspecified; `value` then holds all four data bytes. encode_sdo never writes size 0.
  std::uint8_t size{};
  std::uint32_t value{};       // The `size` low bytes; zero-extended (all four for size 0).
  std::uint32_t abort_code{};  // Only for the abort types.
};

// std::nullopt when a data-carrying SDO has a size outside 1..4 or a value wider than size.
// Fields that the type does not carry are not encoded.
std::optional<Frame> encode_sdo(NodeId node, const Sdo& sdo);
std::optional<Sdo> decode_sdo(NodeId node, const Frame& frame);

// ---- PDOs (layouts generated from the EDS mappings) ----
// unpack_* returns std::nullopt unless the COB-ID is that PDO's for `node` and the DLC equals
// the PDO's generated length exactly (8 for RPDO1/2 and TPDO1-3, 2 for RPDO3).

Frame pack_rpdo1(NodeId node, const od::Rpdo1& pdo);
Frame pack_rpdo2(NodeId node, const od::Rpdo2& pdo);
Frame pack_rpdo3(NodeId node, const od::Rpdo3& pdo);
Frame pack_tpdo1(NodeId node, const od::Tpdo1& pdo);
Frame pack_tpdo2(NodeId node, const od::Tpdo2& pdo);
Frame pack_tpdo3(NodeId node, const od::Tpdo3& pdo);

std::optional<od::Rpdo1> unpack_rpdo1(NodeId node, const Frame& frame);
std::optional<od::Rpdo2> unpack_rpdo2(NodeId node, const Frame& frame);
std::optional<od::Rpdo3> unpack_rpdo3(NodeId node, const Frame& frame);
std::optional<od::Tpdo1> unpack_tpdo1(NodeId node, const Frame& frame);
std::optional<od::Tpdo2> unpack_tpdo2(NodeId node, const Frame& frame);
std::optional<od::Tpdo3> unpack_tpdo3(NodeId node, const Frame& frame);

}  // namespace canopen_codec
