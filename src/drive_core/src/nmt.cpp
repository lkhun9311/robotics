// SPDX-License-Identifier: Apache-2.0
#include "drive_core/nmt.hpp"

namespace drive_core {

using canopen_codec::NmtCommand;
using canopen_codec::NmtState;

bool NmtSlave::accepts(FrameKind kind) const {
  switch (state_) {
    case NmtState::kBootUp:
      return false;
    case NmtState::kPreOperational:
      return kind != FrameKind::kPdo;
    case NmtState::kOperational:
      return true;
    case NmtState::kStopped:
      return kind == FrameKind::kNmt || kind == FrameKind::kHeartbeat;
  }
  return false;
}

void NmtSlave::boot_up() { state_ = NmtState::kPreOperational; }

NmtAction NmtSlave::on_command(NmtCommand command) {
  if (state_ == NmtState::kBootUp) {
    return NmtAction::kNone;
  }
  switch (command) {
    case NmtCommand::kStart:
      state_ = NmtState::kOperational;
      return NmtAction::kNone;
    case NmtCommand::kStop:
      state_ = NmtState::kStopped;
      return NmtAction::kNone;
    case NmtCommand::kEnterPreOperational:
      state_ = NmtState::kPreOperational;
      return NmtAction::kNone;
    case NmtCommand::kResetNode:
      state_ = NmtState::kBootUp;
      return NmtAction::kResetNode;
    case NmtCommand::kResetCommunication:
      state_ = NmtState::kBootUp;
      return NmtAction::kResetCommunication;
  }
  return NmtAction::kNone;
}

}  // namespace drive_core
