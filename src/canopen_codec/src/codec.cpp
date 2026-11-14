// SPDX-License-Identifier: Apache-2.0
#include "canopen_codec/codec.hpp"

#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace canopen_codec {
namespace {

// Predefined connection set (CiA 301); the EDS does not carry these bases.
constexpr std::uint16_t kNmtCobId = 0x000;
constexpr std::uint16_t kHeartbeatBase = 0x700;
constexpr std::uint16_t kSdoRequestBase = 0x600;
constexpr std::uint16_t kSdoResponseBase = 0x580;

constexpr std::uint8_t kMaxDlc = 8;

// SDO command specifiers (byte 0).
constexpr std::uint8_t kSdoUploadRequest = 0x40;
constexpr std::uint8_t kSdoUploadResponseExpedited = 0x43;   // scs 2, e = 1, s = 1, n in bits 2-3
constexpr std::uint8_t kSdoDownloadRequestExpedited = 0x23;  // ccs 1, e = 1, s = 1, n in bits 2-3
constexpr std::uint8_t kSdoDownloadRequestUnsized = 0x22;    // ccs 1, e = 1, s = 0, n = 0
constexpr std::uint8_t kSdoDownloadResponse = 0x60;
constexpr std::uint8_t kSdoAbort = 0x80;
constexpr std::uint8_t kSdoSizeMask = 0x0C;

template <class T>
void put_le(std::array<std::uint8_t, 8>& data, std::size_t offset, T value) {
  using U = std::make_unsigned_t<T>;
  const auto bits = static_cast<U>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    data.at(offset + i) = static_cast<std::uint8_t>(bits >> (8 * i));
  }
}

template <class T>
T get_le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
  using U = std::make_unsigned_t<T>;
  U bits = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    bits = static_cast<U>(bits | static_cast<U>(static_cast<U>(data.at(offset + i)) << (8 * i)));
  }
  return static_cast<T>(bits);  // Two's complement for signed T (defined in C++20).
}

constexpr std::uint16_t node_cob(std::uint16_t base, NodeId node) {
  return static_cast<std::uint16_t>(base + node.value());
}

// The node whose node-relative identifier `cob_id` is, if any.
std::optional<NodeId> node_from_cob(std::uint16_t cob_id, std::uint16_t base) {
  if (cob_id <= base || cob_id > base + NodeId::kMax) {
    return std::nullopt;
  }
  return NodeId::make(static_cast<std::uint8_t>(cob_id - base));
}

bool known_nmt_command(std::uint8_t value) {
  switch (static_cast<NmtCommand>(value)) {
    case NmtCommand::kStart:
    case NmtCommand::kStop:
    case NmtCommand::kEnterPreOperational:
    case NmtCommand::kResetNode:
    case NmtCommand::kResetCommunication:
      return true;
  }
  return false;
}

bool known_nmt_state(std::uint8_t value) {
  switch (static_cast<NmtState>(value)) {
    case NmtState::kBootUp:
    case NmtState::kStopped:
    case NmtState::kOperational:
    case NmtState::kPreOperational:
      return true;
  }
  return false;
}

// ---- PDO layout binding ----
// Each generated PDO's members are bound to the generated field table by name; the
// static_asserts below fail the build if the EDS mapping changes order, name, width or
// signedness without this binding following.

template <class Pdo, class T>
struct Member {
  std::string_view name;
  T Pdo::*ptr;
};

template <class Pdo, class T>
constexpr Member<Pdo, T> member(std::string_view name, T Pdo::*ptr) {
  return {name, ptr};
}

template <class Pdo, class... T, std::size_t... I>
constexpr bool layout_matches(const std::tuple<Member<Pdo, T>...>& members,
                              std::index_sequence<I...> /*unused*/) {
  return ((Pdo::kFields[I].name == std::get<I>(members).name && Pdo::kFields[I].size == sizeof(T) &&
           Pdo::kFields[I].is_signed == std::is_signed_v<T>) &&
          ...);
}

template <class Pdo, class... T>
constexpr bool layout_matches(const std::tuple<Member<Pdo, T>...>& members) {
  if (sizeof...(T) != Pdo::kFields.size() || Pdo::kLength > kMaxDlc) {
    return false;
  }
  std::size_t offset = 0;
  for (const auto& f : Pdo::kFields) {
    if (f.offset != offset) {
      return false;
    }
    offset += f.size;
  }
  return offset == Pdo::kLength && layout_matches(members, std::index_sequence_for<T...>{});
}

constexpr auto kRpdo1 = std::make_tuple(member("target_position", &od::Rpdo1::target_position),
                                        member("command_seq", &od::Rpdo1::command_seq),
                                        member("exec_tag", &od::Rpdo1::exec_tag));
constexpr auto kRpdo2 = std::make_tuple(member("valid_until_us", &od::Rpdo2::valid_until_us),
                                        member("command_seq", &od::Rpdo2::command_seq),
                                        member("exec_tag", &od::Rpdo2::exec_tag));
constexpr auto kRpdo3 = std::make_tuple(member("controlword", &od::Rpdo3::controlword));
constexpr auto kTpdo1 =
    std::make_tuple(member("statusword", &od::Tpdo1::statusword),
                    member("position_actual_value", &od::Tpdo1::position_actual_value),
                    member("sample_seq", &od::Tpdo1::sample_seq));
constexpr auto kTpdo2 = std::make_tuple(
    member("velocity_actual_value", &od::Tpdo2::velocity_actual_value),
    member("sample_seq", &od::Tpdo2::sample_seq), member("applied_seq", &od::Tpdo2::applied_seq));
constexpr auto kTpdo3 = std::make_tuple(member("sample_time_us", &od::Tpdo3::sample_time_us),
                                        member("sample_seq", &od::Tpdo3::sample_seq),
                                        member("applied_tag", &od::Tpdo3::applied_tag));

static_assert(layout_matches(kRpdo1), "RPDO1 binding does not match the generated mapping");
static_assert(layout_matches(kRpdo2), "RPDO2 binding does not match the generated mapping");
static_assert(layout_matches(kRpdo3), "RPDO3 binding does not match the generated mapping");
static_assert(layout_matches(kTpdo1), "TPDO1 binding does not match the generated mapping");
static_assert(layout_matches(kTpdo2), "TPDO2 binding does not match the generated mapping");
static_assert(layout_matches(kTpdo3), "TPDO3 binding does not match the generated mapping");

template <class Pdo>
constexpr std::uint16_t pdo_cob(NodeId node) {
  return Pdo::kCobIdAddsNodeId ? node_cob(Pdo::kCobIdBase, node) : Pdo::kCobIdBase;
}

template <class Pdo, class Members>
Frame pack(NodeId node, const Pdo& pdo, const Members& members) {
  Frame frame{pdo_cob<Pdo>(node), static_cast<std::uint8_t>(Pdo::kLength), {}};
  std::size_t i = 0;
  std::apply(
      [&](const auto&... m) { (put_le(frame.data, Pdo::kFields.at(i++).offset, pdo.*m.ptr), ...); },
      members);
  return frame;
}

template <class Pdo, class Members>
std::optional<Pdo> unpack(NodeId node, const Frame& frame, const Members& members) {
  if (frame.cob_id != pdo_cob<Pdo>(node) || frame.dlc != Pdo::kLength) {
    return std::nullopt;
  }
  Pdo pdo{};
  std::size_t i = 0;
  std::apply(
      [&](const auto&... m) {
        ((pdo.*m.ptr = get_le<std::remove_reference_t<decltype(pdo.*m.ptr)>>(
              frame.data, Pdo::kFields.at(i++).offset)),
         ...);
      },
      members);
  return pdo;
}

bool carries_data(SdoType type) {
  return type == SdoType::kUploadResponse || type == SdoType::kDownloadRequest;
}

}  // namespace

// ---- NMT ----

Frame encode_nmt(const NmtMessage& message) {
  Frame frame{kNmtCobId, 2, {}};
  frame.data[0] = static_cast<std::uint8_t>(message.command);
  frame.data[1] = message.target ? message.target->value() : 0;
  return frame;
}

std::optional<NmtMessage> decode_nmt(const Frame& frame) {
  if (frame.cob_id != kNmtCobId || frame.dlc != 2 || !known_nmt_command(frame.data[0])) {
    return std::nullopt;
  }
  NmtMessage message{static_cast<NmtCommand>(frame.data[0]), std::nullopt};
  if (frame.data[1] != 0) {
    message.target = NodeId::make(frame.data[1]);
    if (!message.target) {
      return std::nullopt;
    }
  }
  return message;
}

// ---- Heartbeat ----

Frame encode_heartbeat(const Heartbeat& heartbeat) {
  Frame frame{node_cob(kHeartbeatBase, heartbeat.node), 1, {}};
  frame.data[0] = static_cast<std::uint8_t>(heartbeat.state);
  return frame;
}

std::optional<Heartbeat> decode_heartbeat(const Frame& frame) {
  const auto node = node_from_cob(frame.cob_id, kHeartbeatBase);
  if (!node || frame.dlc != 1 || !known_nmt_state(frame.data[0])) {
    return std::nullopt;
  }
  return Heartbeat{*node, static_cast<NmtState>(frame.data[0])};
}

// ---- SYNC ----

Frame encode_sync() { return Frame{od::kSyncCobId, 0, {}}; }

bool is_sync(const Frame& frame) { return frame.cob_id == od::kSyncCobId && frame.dlc == 0; }

// ---- EMCY ----

Frame encode_emcy(const Emcy& emcy) {
  Frame frame{node_cob(od::kEmcyCobIdBase, emcy.node), 8, {}};
  put_le(frame.data, 0, emcy.error_code);
  frame.data[2] = emcy.error_register;
  for (std::size_t i = 0; i < emcy.manufacturer.size(); ++i) {
    frame.data.at(3 + i) = emcy.manufacturer.at(i);
  }
  return frame;
}

std::optional<Emcy> decode_emcy(const Frame& frame) {
  const auto node = node_from_cob(frame.cob_id, od::kEmcyCobIdBase);
  if (!node || frame.dlc != 8) {
    return std::nullopt;
  }
  Emcy emcy{*node, get_le<std::uint16_t>(frame.data, 0), frame.data[2], {}};
  for (std::size_t i = 0; i < emcy.manufacturer.size(); ++i) {
    emcy.manufacturer.at(i) = frame.data.at(3 + i);
  }
  return emcy;
}

// ---- SDO ----

std::optional<Frame> encode_sdo(NodeId node, const Sdo& sdo) {
  const bool to_server = sdo.type == SdoType::kUploadRequest ||
                         sdo.type == SdoType::kDownloadRequest ||
                         sdo.type == SdoType::kAbortToServer;
  Frame frame{node_cob(to_server ? kSdoRequestBase : kSdoResponseBase, node), 8, {}};
  if (carries_data(sdo.type)) {
    if (sdo.size < 1 || sdo.size > 4 || (sdo.size < 4 && (sdo.value >> (8 * sdo.size)) != 0)) {
      return std::nullopt;
    }
  }
  // Expedited transfers encode the count of unused data bytes, 4 - size, in bits 2-3.
  const auto unused =
      carries_data(sdo.type) ? static_cast<std::uint8_t>((4U - sdo.size) << 2U) : std::uint8_t{0};
  switch (sdo.type) {
    case SdoType::kUploadRequest:
      frame.data[0] = kSdoUploadRequest;
      break;
    case SdoType::kUploadResponse:
      frame.data[0] = static_cast<std::uint8_t>(kSdoUploadResponseExpedited | unused);
      break;
    case SdoType::kDownloadRequest:
      frame.data[0] = static_cast<std::uint8_t>(kSdoDownloadRequestExpedited | unused);
      break;
    case SdoType::kDownloadResponse:
      frame.data[0] = kSdoDownloadResponse;
      break;
    case SdoType::kAbortToClient:
    case SdoType::kAbortToServer:
      frame.data[0] = kSdoAbort;
      break;
  }
  put_le(frame.data, 1, sdo.address.index);
  frame.data[3] = sdo.address.sub;
  if (carries_data(sdo.type)) {
    put_le(frame.data, 4, sdo.value);  // Bytes above `size` are zero (checked above).
  } else if (sdo.type == SdoType::kAbortToClient || sdo.type == SdoType::kAbortToServer) {
    put_le(frame.data, 4, sdo.abort_code);
  }
  return frame;
}

std::optional<Sdo> decode_sdo(NodeId node, const Frame& frame) {
  if (frame.dlc != 8) {
    return std::nullopt;
  }
  const std::uint8_t command = frame.data[0];
  Sdo sdo{};
  sdo.address = {get_le<std::uint16_t>(frame.data, 1), frame.data[3]};
  const auto with_data = [&](SdoType type) {
    sdo.type = type;
    sdo.size = static_cast<std::uint8_t>(4 - ((command & kSdoSizeMask) >> 2));
    const std::uint32_t all = get_le<std::uint32_t>(frame.data, 4);
    sdo.value = sdo.size == 4 ? all : all & ((std::uint32_t{1} << (8 * sdo.size)) - 1);
  };
  const auto with_abort = [&](SdoType type) {
    sdo.type = type;
    sdo.abort_code = get_le<std::uint32_t>(frame.data, 4);
  };
  if (frame.cob_id == node_cob(kSdoRequestBase, node)) {
    if (command == kSdoUploadRequest) {
      sdo.type = SdoType::kUploadRequest;
    } else if ((command & ~kSdoSizeMask) == kSdoDownloadRequestExpedited) {
      with_data(SdoType::kDownloadRequest);
    } else if (command == kSdoDownloadRequestUnsized) {
      sdo.type = SdoType::kDownloadRequest;  // Size unspecified: all four data bytes, size 0.
      sdo.value = get_le<std::uint32_t>(frame.data, 4);
    } else if (command == kSdoAbort) {
      with_abort(SdoType::kAbortToServer);
    } else {
      return std::nullopt;
    }
  } else if (frame.cob_id == node_cob(kSdoResponseBase, node)) {
    if ((command & ~kSdoSizeMask) == kSdoUploadResponseExpedited) {
      with_data(SdoType::kUploadResponse);
    } else if (command == kSdoDownloadResponse) {
      sdo.type = SdoType::kDownloadResponse;
    } else if (command == kSdoAbort) {
      with_abort(SdoType::kAbortToClient);
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
  return sdo;
}

// ---- PDOs ----

Frame pack_rpdo1(NodeId node, const od::Rpdo1& pdo) { return pack(node, pdo, kRpdo1); }
Frame pack_rpdo2(NodeId node, const od::Rpdo2& pdo) { return pack(node, pdo, kRpdo2); }
Frame pack_rpdo3(NodeId node, const od::Rpdo3& pdo) { return pack(node, pdo, kRpdo3); }
Frame pack_tpdo1(NodeId node, const od::Tpdo1& pdo) { return pack(node, pdo, kTpdo1); }
Frame pack_tpdo2(NodeId node, const od::Tpdo2& pdo) { return pack(node, pdo, kTpdo2); }
Frame pack_tpdo3(NodeId node, const od::Tpdo3& pdo) { return pack(node, pdo, kTpdo3); }

std::optional<od::Rpdo1> unpack_rpdo1(NodeId node, const Frame& frame) {
  return unpack<od::Rpdo1>(node, frame, kRpdo1);
}
std::optional<od::Rpdo2> unpack_rpdo2(NodeId node, const Frame& frame) {
  return unpack<od::Rpdo2>(node, frame, kRpdo2);
}
std::optional<od::Rpdo3> unpack_rpdo3(NodeId node, const Frame& frame) {
  return unpack<od::Rpdo3>(node, frame, kRpdo3);
}
std::optional<od::Tpdo1> unpack_tpdo1(NodeId node, const Frame& frame) {
  return unpack<od::Tpdo1>(node, frame, kTpdo1);
}
std::optional<od::Tpdo2> unpack_tpdo2(NodeId node, const Frame& frame) {
  return unpack<od::Tpdo2>(node, frame, kTpdo2);
}
std::optional<od::Tpdo3> unpack_tpdo3(NodeId node, const Frame& frame) {
  return unpack<od::Tpdo3>(node, frame, kTpdo3);
}

}  // namespace canopen_codec
