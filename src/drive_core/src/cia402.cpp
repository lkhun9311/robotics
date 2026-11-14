// SPDX-License-Identifier: Apache-2.0
#include "drive_core/cia402.hpp"

namespace drive_core {

namespace {

// Device control commands (CiA 402, 6040h), decoded from bits 3..0 with bit 7 clear.
enum class Command : std::uint8_t {
  kNone,
  kShutdown,         // 0xxx x110
  kSwitchOn,         // 0xxx 0111 (Disable Operation in Operation Enabled)
  kEnableOperation,  // 0xxx 1111 (Switch On + Enable Operation in Ready to Switch On)
  kDisableVoltage,   // 0xxx xx0x
  kQuickStop,        // 0xxx x01x
};

constexpr std::uint16_t kBitSwitchOn = 1U << 0U;
constexpr std::uint16_t kBitEnableVoltage = 1U << 1U;
constexpr std::uint16_t kBitQuickStop = 1U << 2U;  // Active low.
constexpr std::uint16_t kBitEnableOperation = 1U << 3U;
constexpr std::uint16_t kBitFaultReset = 1U << 7U;

Command decode(std::uint16_t cw) {
  if ((cw & kBitFaultReset) != 0U) {
    return Command::kNone;  // Bit 7 set is Fault Reset only, handled on its edge.
  }
  if ((cw & kBitEnableVoltage) == 0U) {
    return Command::kDisableVoltage;
  }
  if ((cw & kBitQuickStop) == 0U) {
    return Command::kQuickStop;
  }
  if ((cw & kBitSwitchOn) == 0U) {
    return Command::kShutdown;
  }
  return (cw & kBitEnableOperation) != 0U ? Command::kEnableOperation : Command::kSwitchOn;
}

// Statusword bits (CiA 402, 6041h).
constexpr std::uint16_t kReadyToSwitchOn = 1U << 0U;
constexpr std::uint16_t kSwitchedOn = 1U << 1U;
constexpr std::uint16_t kOperationEnabled = 1U << 2U;
constexpr std::uint16_t kFault = 1U << 3U;
constexpr std::uint16_t kVoltageEnabled = 1U << 4U;
constexpr std::uint16_t kQuickStopOff = 1U << 5U;  // 0 while a quick stop is active.
constexpr std::uint16_t kSwitchOnDisabled = 1U << 6U;
constexpr std::uint16_t kRemote = 1U << 9U;  // The controlword is processed.

}  // namespace

void Cia402::enter(Cia402State next) {
  if (next != Cia402State::kQuickStopActive) {
    quick_stop_reason_.reset();
  }
  if (next != Cia402State::kFaultReactionActive && next != Cia402State::kFault) {
    fault_code_.reset();
  }
  state_ = next;
}

void Cia402::complete_initialisation() {
  if (state_ == Cia402State::kNotReadyToSwitchOn) {
    enter(Cia402State::kSwitchOnDisabled);  // Transition 1.
  }
}

void Cia402::on_controlword(std::uint16_t controlword) {
  const bool reset_bit = (controlword & kBitFaultReset) != 0U;
  const bool reset_edge = reset_bit && !fault_reset_bit_;
  fault_reset_bit_ = reset_bit;

  const Command cmd = decode(controlword);
  switch (state_) {
    case Cia402State::kNotReadyToSwitchOn:
    case Cia402State::kFaultReactionActive:
      break;
    case Cia402State::kSwitchOnDisabled:
      if (cmd == Command::kShutdown) {
        enter(Cia402State::kReadyToSwitchOn);  // 2
      }
      break;
    case Cia402State::kReadyToSwitchOn:
      if (cmd == Command::kSwitchOn) {
        enter(Cia402State::kSwitchedOn);  // 3
      } else if (cmd == Command::kEnableOperation && !enable_inhibited_) {
        enter(Cia402State::kOperationEnabled);  // 3 + 4
      } else if (cmd == Command::kDisableVoltage || cmd == Command::kQuickStop) {
        enter(Cia402State::kSwitchOnDisabled);  // 7
      }
      break;
    case Cia402State::kSwitchedOn:
      if (cmd == Command::kEnableOperation && !enable_inhibited_) {
        enter(Cia402State::kOperationEnabled);  // 4
      } else if (cmd == Command::kShutdown) {
        enter(Cia402State::kReadyToSwitchOn);  // 6
      } else if (cmd == Command::kDisableVoltage || cmd == Command::kQuickStop) {
        enter(Cia402State::kSwitchOnDisabled);  // 10
      }
      break;
    case Cia402State::kOperationEnabled:
      if (cmd == Command::kSwitchOn) {
        enter(Cia402State::kSwitchedOn);  // 5 (Disable Operation)
      } else if (cmd == Command::kShutdown) {
        enter(Cia402State::kReadyToSwitchOn);  // 8
      } else if (cmd == Command::kDisableVoltage) {
        enter(Cia402State::kSwitchOnDisabled);  // 9
      } else if (cmd == Command::kQuickStop) {
        enter(Cia402State::kQuickStopActive);  // 11
        quick_stop_reason_ = QuickStopReason::kControlword;
      }
      break;
    case Cia402State::kQuickStopActive:
      // Option 6 (D10): no automatic transition 12; the state is left only by command.
      if (cmd == Command::kDisableVoltage) {
        enter(Cia402State::kSwitchOnDisabled);  // 12
      } else if (cmd == Command::kEnableOperation && !enable_inhibited_) {
        enter(Cia402State::kOperationEnabled);  // 16
      }
      break;
    case Cia402State::kFault:
      if (reset_edge) {
        enter(Cia402State::kSwitchOnDisabled);  // 15
      }
      break;
  }
}

bool Cia402::quick_stop(QuickStopReason reason) {
  if (state_ != Cia402State::kOperationEnabled) {
    return false;
  }
  enter(Cia402State::kQuickStopActive);  // 11
  quick_stop_reason_ = reason;
  return true;
}

void Cia402::drive_fault(std::uint16_t error_code) {
  if (state_ == Cia402State::kFaultReactionActive || state_ == Cia402State::kFault) {
    return;
  }
  enter(Cia402State::kFaultReactionActive);  // 13
  fault_code_ = error_code;
}

void Cia402::fault_reaction_done() {
  if (state_ == Cia402State::kFaultReactionActive) {
    enter(Cia402State::kFault);  // 14
  }
}

void Cia402::fault_without_reaction(std::uint16_t error_code) {
  if (state_ != Cia402State::kFault) {
    enter(Cia402State::kFault);  // 13 + 14, no reaction.
  }
  fault_code_ = error_code;
}

bool Cia402::request_mode(std::int8_t mode) { return mode == kModeCsp; }

std::uint16_t Cia402::statusword() const {
  std::uint16_t sw = kRemote;
  switch (state_) {
    case Cia402State::kNotReadyToSwitchOn:
      sw |= kQuickStopOff;
      break;
    case Cia402State::kSwitchOnDisabled:
      sw |= kSwitchOnDisabled | kQuickStopOff;
      break;
    case Cia402State::kReadyToSwitchOn:
      sw |= kReadyToSwitchOn | kQuickStopOff | kVoltageEnabled;
      break;
    case Cia402State::kSwitchedOn:
      sw |= kReadyToSwitchOn | kSwitchedOn | kQuickStopOff | kVoltageEnabled;
      break;
    case Cia402State::kOperationEnabled:
      sw |= kReadyToSwitchOn | kSwitchedOn | kOperationEnabled | kQuickStopOff | kVoltageEnabled;
      break;
    case Cia402State::kQuickStopActive:
      sw |= kReadyToSwitchOn | kSwitchedOn | kOperationEnabled | kVoltageEnabled;
      break;
    case Cia402State::kFaultReactionActive:
      sw |= kReadyToSwitchOn | kSwitchedOn | kOperationEnabled | kFault | kVoltageEnabled;
      break;
    case Cia402State::kFault:
      sw |= kFault;
      break;
  }
  return sw;
}

}  // namespace drive_core
