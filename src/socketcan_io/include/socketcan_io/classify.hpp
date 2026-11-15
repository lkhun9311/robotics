// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <linux/can.h>

#include <cstddef>
#include <optional>

#include "canopen_codec/codec.hpp"

// Which received frames may reach the CANopen codec. Only a classic, standard-identifier data
// frame of up to 8 bytes does; everything else is counted by class and dropped (codec.hpp).
namespace socketcan_io {

enum class RxClass { kData, kRtr, kError, kFd, kExtendedId, kBadLength };
inline constexpr std::size_t kRxClassCount = 6;

// `nbytes` is what recvmsg returned: CAN_MTU for a classic frame, CANFD_MTU for an FD frame.
RxClass classify(const canfd_frame& frame, std::size_t nbytes);

// A value only for RxClass::kData.
std::optional<canopen_codec::Frame> to_codec(const canfd_frame& frame, std::size_t nbytes);

}  // namespace socketcan_io
