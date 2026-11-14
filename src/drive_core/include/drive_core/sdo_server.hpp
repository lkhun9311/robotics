// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

#include "canopen_codec/codec.hpp"

// Expedited SDO server (CiA 301) built from the generated object dictionary (od::kEntries, which
// tools/od/gen_header.py generates from config/od/vdrive_arm.eds). The server owns everything
// the EDS states: whether an object and sub-index exist, access rights, size and the value
// limits. What a value means is the drive's: it answers through ObjectAccess.
namespace drive_core {

// SDO abort codes (CiA 301) used by this drive.
namespace sdo_abort {
inline constexpr std::uint32_t kUnsupportedAccess = 0x0601'0000;  // E.g. a PDO-only write.
inline constexpr std::uint32_t kWriteOnly = 0x0601'0001;          // Read of a write-only object.
inline constexpr std::uint32_t kReadOnly = 0x0601'0002;           // Write of a read-only object.
inline constexpr std::uint32_t kNoObject = 0x0602'0000;           // Index not in the OD.
inline constexpr std::uint32_t kLength = 0x0607'0010;             // Size does not match.
inline constexpr std::uint32_t kNoSubIndex = 0x0609'0011;         // Index exists, sub does not.
inline constexpr std::uint32_t kRange = 0x0609'0030;              // Value outside the limits.
inline constexpr std::uint32_t kTransfer = 0x0800'0020;           // Cannot be stored (D17).
inline constexpr std::uint32_t kDeviceState = 0x0800'0022;        // Not in this device state.
}  // namespace sdo_abort

// The drive's side of the object dictionary. The server calls it only for an entry that exists
// and only with a request that passed the access, size and limit checks.
class ObjectAccess {
 public:
  // The raw value of `entry` (two's complement for signed types, the low `entry.size` bytes
  // significant), or an abort code.
  struct ReadResult {
    std::uint32_t value{};
    std::uint32_t abort{};  // 0: no abort.
  };
  virtual ReadResult read(const canopen_codec::od::EntryInfo& entry) = 0;
  // Applies a write; returns 0 or an abort code. A write that aborts changes nothing unless its
  // object documents otherwise (0x2011 clears the staging, D17).
  virtual std::uint32_t write(const canopen_codec::od::EntryInfo& entry, std::uint32_t value) = 0;

 protected:
  ObjectAccess() = default;
  ObjectAccess(const ObjectAccess&) = default;
  ObjectAccess& operator=(const ObjectAccess&) = default;
  ~ObjectAccess() = default;
};

class SdoServer {
 public:
  // The OD entry at `address`, or nullptr.
  static const canopen_codec::od::EntryInfo* find(canopen_codec::od::Address address);
  // Whether any entry of `index` exists.
  static bool has_index(std::uint16_t index);

  // Serves one decoded SDO request addressed to this node and returns the response, or nullopt
  // for a request that gets none (a client abort). Checks, in order: object (0x06020000), sub
  // (0x06090011), access (0x06010001 / 0x06010002), size (0x06070010), limits (0x06090030);
  // then the drive's ObjectAccess. Never throws.
  static std::optional<canopen_codec::Sdo> handle(const canopen_codec::Sdo& request,
                                                  ObjectAccess& objects);
};

}  // namespace drive_core
