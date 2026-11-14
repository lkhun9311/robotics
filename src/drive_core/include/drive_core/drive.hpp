// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/assembly.hpp"
#include "drive_core/cia402.hpp"
#include "drive_core/exec_tag.hpp"
#include "drive_core/nmt.hpp"
#include "runtime_core/time.hpp"

// The virtual drive's frame dispatcher. Every received frame is classified, passed through the
// NMT service matrix (D12) and only then handed to its service. The drive loop (SYNC timeline,
// guards, quick-stop ramp, joint model) and the SDO server come later; this class already owns
// the state they act on.
namespace drive_core {

// Frames handed to a service after the NMT gate, and frames dropped before it.
struct DispatchCounts {
  std::uint32_t nmt{};
  std::uint32_t heartbeat{};
  std::uint32_t sync{};
  std::uint32_t emcy{};
  std::uint32_t sdo{};
  std::uint32_t pdo{};
  std::uint32_t gated{};      // Classified, but the NMT state does not accept the service.
  std::uint32_t foreign{};    // Not addressed to this node or not a service it consumes.
  std::uint32_t malformed{};  // Addressed to this node, but did not decode (e.g. wrong DLC).

  friend bool operator==(const DispatchCounts&, const DispatchCounts&) = default;
};

// The service a frame belongs to, from its COB-ID alone; nullopt for a frame this node does not
// consume (another node's SDO or PDO, an SDO response, an unknown identifier).
std::optional<FrameKind> classify(canopen_codec::NodeId node, const canopen_codec::Frame& frame);

class Drive {
 public:
  // boot_id (0x2013) of the first boot comes from the caller (the process draws it at start).
  Drive(canopen_codec::NodeId node, std::uint32_t boot_id, runtime_core::MonoTime now);
  // The assembler refers to the issuer, so a Drive stays where it was made.
  Drive(const Drive&) = delete;
  Drive& operator=(const Drive&) = delete;

  // Power on: CiA 402 initialisation, NMT boot-up. Returns the boot-up message. The node stays
  // in Pre-operational until an NMT Start.
  std::vector<canopen_codec::Frame> power_on();

  // Handles one received frame and returns the frames to send in reply. Never throws for any
  // frame content.
  std::vector<canopen_codec::Frame> on_frame(const canopen_codec::Frame& frame,
                                             runtime_core::MonoTime now);

  canopen_codec::NodeId node() const { return node_; }
  const NmtSlave& nmt() const { return nmt_; }
  const Cia402& cia402() const { return cia402_; }
  Cia402& cia402() { return cia402_; }
  const ExecTagIssuer& issuer() const { return issuer_; }
  ExecTagIssuer& issuer() { return issuer_; }
  const CommandAssembler& assembler() const { return assembler_; }
  const DispatchCounts& counts() const { return counts_; }

 private:
  std::vector<canopen_codec::Frame> on_nmt(const canopen_codec::Frame& frame,
                                           runtime_core::MonoTime now);
  std::vector<canopen_codec::Frame> on_sdo(const canopen_codec::Frame& frame);
  void on_pdo(const canopen_codec::Frame& frame);
  canopen_codec::Frame boot_up_frame() const;

  canopen_codec::NodeId node_;
  NmtSlave nmt_;
  Cia402 cia402_;
  ExecTagIssuer issuer_;
  CommandAssembler assembler_;
  DispatchCounts counts_;
};

}  // namespace drive_core
