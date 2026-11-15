// SPDX-License-Identifier: Apache-2.0
#include "socketcan_io/classify.hpp"

#include <algorithm>

namespace socketcan_io {

RxClass classify(const canfd_frame& frame, std::size_t nbytes) {
  if (nbytes == CANFD_MTU) {
    return RxClass::kFd;
  }
  if (nbytes != CAN_MTU) {
    return RxClass::kBadLength;
  }
  if ((frame.can_id & CAN_ERR_FLAG) != 0) {
    return RxClass::kError;
  }
  if ((frame.can_id & CAN_EFF_FLAG) != 0) {
    return RxClass::kExtendedId;
  }
  if ((frame.can_id & CAN_RTR_FLAG) != 0) {
    return RxClass::kRtr;
  }
  if (frame.len > 8) {
    return RxClass::kBadLength;
  }
  return RxClass::kData;
}

std::optional<canopen_codec::Frame> to_codec(const canfd_frame& frame, std::size_t nbytes) {
  if (classify(frame, nbytes) != RxClass::kData) {
    return std::nullopt;
  }
  canopen_codec::Frame out;
  out.cob_id = static_cast<std::uint16_t>(frame.can_id & CAN_SFF_MASK);
  out.dlc = frame.len;
  std::copy_n(frame.data, frame.len, out.data.begin());
  return out;
}

}  // namespace socketcan_io
