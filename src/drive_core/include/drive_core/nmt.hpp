// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "canopen_codec/codec.hpp"

// NMT slave state and the service matrix of contract D12 (CiA 301).
namespace drive_core {

// The communication objects a received frame can belong to.
enum class FrameKind : std::uint8_t {
  kNmt,
  kHeartbeat,
  kSync,
  kEmcy,
  kSdo,
  kPdo,
};

// What the node must do after an NMT command, beyond the state change made here.
enum class NmtAction : std::uint8_t {
  kNone,
  kResetCommunication,  // Reset the communication state (D12), then send boot-up.
  kResetNode,           // Start a new boot (D3), then send boot-up.
};

// State kBootUp stands for Initialisation: before boot_up() and while a reset runs, nothing is
// accepted. A node never leaves Pre-operational on its own; only an NMT Start does that.
class NmtSlave {
 public:
  canopen_codec::NmtState state() const { return state_; }

  // D12: Pre-operational: NMT, heartbeat, SYNC, EMCY, SDO, no PDO. Operational: all. Stopped:
  // NMT and heartbeat only. Initialisation: nothing.
  bool accepts(FrameKind kind) const;

  // Initialisation is complete: the boot-up message is sent and the node enters Pre-operational.
  void boot_up();

  // Applies an NMT command addressed to this node. A reset puts the node back in
  // Initialisation; the caller performs the reset and then calls boot_up().
  NmtAction on_command(canopen_codec::NmtCommand command);

 private:
  canopen_codec::NmtState state_{canopen_codec::NmtState::kBootUp};
};

}  // namespace drive_core
