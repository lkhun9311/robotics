// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

// CiA 402 power drive system state machine (device control, 6040h/6041h) for the virtual drive,
// with quick stop option code 605Ah = 6 (contract D10) and the fault reaction of D11. It holds
// state only: the ramp, the joint model and the tag revocation that go with a transition belong
// to the drive loop, which reads state() and the reasons recorded here.
namespace drive_core {

enum class Cia402State : std::uint8_t {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
};

// Why the drive is in Quick Stop Active. kControlword is the master's Quick Stop command; the
// others are the policy stops of D11.
enum class QuickStopReason : std::uint8_t {
  kControlword,
  kTimeout,
  kSyncLoss,
  kIntegrity,
  kExpiry,
  kCommState,  // The NMT state left Operational while Operation Enabled (amended D12).
};

class Cia402 {
 public:
  // Modes of operation (6060h/6061h): only cyclic synchronous position is implemented.
  static constexpr std::int8_t kModeCsp = 8;

  // Starts in Not Ready to Switch On.
  Cia402() = default;

  // Transition 1 (automatic after initialisation): Not Ready to Switch On -> Switch On Disabled.
  void complete_initialisation();

  // Applies a controlword (6040h). Commands that have no transition in the current state are
  // ignored. Bit 7 is tracked on every call, so Fault Reset acts only on a 0 -> 1 edge.
  void on_controlword(std::uint16_t controlword);

  // Policy stop (D11): Operation Enabled -> Quick Stop Active (transition 11). Returns false and
  // changes nothing in any other state.
  bool quick_stop(QuickStopReason reason);

  // Fault occurrence (transition 13): any state but Fault Reaction Active and Fault -> Fault
  // Reaction Active. A fault while one is already being handled keeps the first code.
  void drive_fault(std::uint16_t error_code);
  // Transition 14: Fault Reaction Active -> Fault once the reaction (ramp or none) is done.
  void fault_reaction_done();
  // Loss of drive authority (D11 torque loss): Fault at once from any state (13 and 14 with no
  // reaction), and the error code becomes `error_code` even if another fault was being handled,
  // since the loss of authority supersedes it.
  void fault_without_reaction(std::uint16_t error_code);

  // While inhibited, every entry into Operation Enabled is refused: transitions 4, 3+4 and 16
  // (16 exists with option 6). The command is ignored and the state stays where it is. The D11
  // policy latch sets this.
  void set_operation_enable_inhibited(bool inhibited) { enable_inhibited_ = inhibited; }
  bool operation_enable_inhibited() const { return enable_inhibited_; }

  // Mode request (6060h). Only kModeCsp is accepted; any other value changes nothing.
  bool request_mode(std::int8_t mode);
  // Mode display (6061h).
  std::int8_t mode_display() const { return mode_; }

  Cia402State state() const { return state_; }
  std::uint16_t statusword() const;
  std::optional<QuickStopReason> quick_stop_reason() const { return quick_stop_reason_; }
  // The error code of the fault being handled; nullopt outside Fault Reaction Active and Fault.
  std::optional<std::uint16_t> fault_code() const { return fault_code_; }

 private:
  void enter(Cia402State next);

  Cia402State state_{Cia402State::kNotReadyToSwitchOn};
  bool fault_reset_bit_{false};  // Bit 7 of the previous controlword.
  bool enable_inhibited_{false};
  std::int8_t mode_{kModeCsp};
  std::optional<QuickStopReason> quick_stop_reason_;
  std::optional<std::uint16_t> fault_code_;
};

}  // namespace drive_core
