// SPDX-License-Identifier: Apache-2.0
#include "drive_core/drive.hpp"

namespace drive_core {

namespace {

namespace cc = canopen_codec;

constexpr std::uint16_t kCobNmt = 0x000;
constexpr std::uint16_t kCobHeartbeatBase = 0x700;
constexpr std::uint16_t kCobSdoRequestBase = 0x600;
constexpr std::uint16_t kNodeMask = 0x07F;
constexpr std::uint16_t kFunctionMask = 0x780;

// The boot_id of the boot after a reset node. The process draws the first boot_id at start; a
// reset node within the process derives the next one (MurmurHash3 finalizer of the previous id
// plus one), which is never equal to the previous id.
std::uint32_t next_boot_id(std::uint32_t previous) {
  std::uint32_t h = previous + 1U;
  h ^= h >> 16U;
  h *= 0x85EB'CA6BU;
  h ^= h >> 13U;
  h *= 0xC2B2'AE35U;
  h ^= h >> 16U;
  return h != previous ? h : h + 1U;
}

}  // namespace

std::optional<FrameKind> classify(cc::NodeId node, const cc::Frame& frame) {
  const std::uint16_t cob = frame.cob_id;
  const auto own = [node](std::uint16_t base) {
    return static_cast<std::uint16_t>(base + node.value());
  };
  if (cob == kCobNmt) {
    return FrameKind::kNmt;
  }
  if (cob == cc::od::kSyncCobId) {
    return FrameKind::kSync;
  }
  const auto function = static_cast<std::uint16_t>(cob & kFunctionMask);
  const bool has_node = (cob & kNodeMask) != 0U;
  if (function == cc::od::kEmcyCobIdBase && has_node) {
    return FrameKind::kEmcy;  // Another node's EMCY (this node only produces its own).
  }
  if (function == kCobHeartbeatBase && has_node) {
    return FrameKind::kHeartbeat;
  }
  if (cob == own(kCobSdoRequestBase)) {
    return FrameKind::kSdo;
  }
  if (cob == own(cc::od::Rpdo1::kCobIdBase) || cob == own(cc::od::Rpdo2::kCobIdBase) ||
      cob == own(cc::od::Rpdo3::kCobIdBase)) {
    return FrameKind::kPdo;
  }
  return std::nullopt;
}

Drive::Drive(cc::NodeId node, std::uint32_t boot_id, runtime_core::MonoTime now)
    : node_(node), issuer_(boot_id, now), assembler_(issuer_) {}

cc::Frame Drive::boot_up_frame() const {
  return cc::encode_heartbeat({node_, cc::NmtState::kBootUp});
}

std::vector<cc::Frame> Drive::power_on() {
  cia402_.complete_initialisation();
  nmt_.boot_up();
  return {boot_up_frame()};
}

std::vector<cc::Frame> Drive::on_frame(const cc::Frame& frame, runtime_core::MonoTime now) {
  const std::optional<FrameKind> kind = classify(node_, frame);
  if (!kind) {
    ++counts_.foreign;
    return {};
  }
  if (!nmt_.accepts(*kind)) {
    ++counts_.gated;
    return {};
  }
  switch (*kind) {
    case FrameKind::kNmt:
      return on_nmt(frame, now);
    case FrameKind::kHeartbeat:
      // Heartbeat consumption is not part of this drive's contract.
      if (cc::decode_heartbeat(frame)) {
        ++counts_.heartbeat;
      } else {
        ++counts_.malformed;
      }
      return {};
    case FrameKind::kSync:
      if (!cc::is_sync(frame)) {
        ++counts_.malformed;
        return {};
      }
      // The SYNC timeline (D7) is the drive loop's; it reads the window from the assembler.
      ++counts_.sync;
      return {};
    case FrameKind::kEmcy:
      if (cc::decode_emcy(frame)) {
        ++counts_.emcy;
      } else {
        ++counts_.malformed;
      }
      return {};
    case FrameKind::kSdo:
      return on_sdo(frame);
    case FrameKind::kPdo:
      on_pdo(frame);
      return {};
  }
  return {};
}

std::vector<cc::Frame> Drive::on_nmt(const cc::Frame& frame, runtime_core::MonoTime now) {
  const std::optional<cc::NmtMessage> message = cc::decode_nmt(frame);
  if (!message) {
    ++counts_.malformed;
    return {};
  }
  if (message->target && *message->target != node_) {
    ++counts_.foreign;
    return {};
  }
  ++counts_.nmt;
  switch (nmt_.on_command(message->command)) {
    case NmtAction::kNone:
      return {};
    case NmtAction::kResetCommunication:
      // D12: the tag dies, session and generation clear, per-command state clears; boot_id and
      // the tag allocator stay. The CiA 402 state is application state and stays too.
      issuer_.reset_communication();
      assembler_.reset();
      break;
    case NmtAction::kResetNode:
      // D3: a new boot with a new boot_id, a new tag start and a new quarantine.
      issuer_.boot(next_boot_id(issuer_.boot_id()), now);
      assembler_.reset();
      cia402_ = Cia402{};
      cia402_.complete_initialisation();
      break;
  }
  nmt_.boot_up();
  return {boot_up_frame()};
}

std::vector<cc::Frame> Drive::on_sdo(const cc::Frame& frame) {
  if (!cc::decode_sdo(node_, frame)) {
    ++counts_.malformed;
    return {};
  }
  ++counts_.sdo;
  return {};  // The SDO server is not implemented yet; until then no request is answered.
}

void Drive::on_pdo(const cc::Frame& frame) {
  if (const auto rpdo1 = cc::unpack_rpdo1(node_, frame)) {
    ++counts_.pdo;
    assembler_.offer(*rpdo1);
  } else if (const auto rpdo2 = cc::unpack_rpdo2(node_, frame)) {
    ++counts_.pdo;
    assembler_.offer(*rpdo2);
  } else if (const auto rpdo3 = cc::unpack_rpdo3(node_, frame)) {
    ++counts_.pdo;
    cia402_.on_controlword(rpdo3->controlword);
  } else {
    ++counts_.malformed;
  }
}

}  // namespace drive_core
