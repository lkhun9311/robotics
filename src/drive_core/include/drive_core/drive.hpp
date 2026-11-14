// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/assembly.hpp"
#include "drive_core/cia402.hpp"
#include "drive_core/commit.hpp"
#include "drive_core/exec_tag.hpp"
#include "drive_core/joint.hpp"
#include "drive_core/nmt.hpp"
#include "drive_core/sdo_server.hpp"
#include "runtime_core/time.hpp"

// The virtual drive: frame dispatcher and drive loop. Every received frame is classified, passed
// through the NMT service matrix (D12) and only then handed to its service. The drive loop runs
// the SYNC timeline (D7), assembly and integrity (D8), guards (D9), the quick-stop ramp (D10),
// the stop and fault layers with the policy latch (D11) and the TPDO feedback (D13). The SDO
// server (sdo_server.hpp) serves the object dictionary, including the execution commit
// transaction (D17).
namespace drive_core {

// 0x2030:05 drive_flags (D11, D14): the one definition of its bit layout.
namespace drive_flag {
inline constexpr std::uint16_t kPolicyLatched = 1U << 0U;     // D11 policy latch is set.
inline constexpr std::uint16_t kDriveUnavailable = 1U << 1U;  // Torque loss: no drive authority.
// An integrity violation (D8) was detected and its latch is not yet reset.
inline constexpr std::uint16_t kIntegrity = 1U << 2U;
inline constexpr std::uint16_t kStopUnconfirmed = 1U << 3U;  // The latest stop is unconfirmed.
}  // namespace drive_flag

// Project-declared timing (E1; contract D7, D9, D10).
inline constexpr runtime_core::Duration kBusPeriod{10'000'000};       // T_bus 10 ms
inline constexpr runtime_core::Duration kSyncTimeout{30'000'000};     // 3 x T_bus (D7)
inline constexpr runtime_core::Duration kCommandTimeout{30'000'000};  // 3 x T_bus (D9)
// A SYNC sooner than this after the previous accepted one is counted and ignored (D7).
inline constexpr runtime_core::Duration kSyncMinInterval{5'000'000};  // T_bus / 2
// stop_confirmed: |v| below this for kQuietTicks consecutive W ticks (D10).
inline constexpr std::int64_t kQuietVelocityNradS = 10'000'000;  // 0.01 rad/s
inline constexpr int kQuietTicks = 5;
// v_max / a_stop + 6 W = 0.112 s; a stop not confirmed by then is stop_unconfirmed (D10).
inline constexpr runtime_core::Duration kStopDeadline{112'000'000};

// Error codes (603Fh / EMCY) of the injected faults. 0x8611 is CiA 402 "following error";
// 0xFF01 is in the CiA 301 device-specific range, chosen by this project for torque loss.
inline constexpr std::uint16_t kErrorFollowing = 0x8611;
inline constexpr std::uint16_t kErrorTorqueLoss = 0xFF01;

// Why a command or a frame did not act.
enum class DropReason : std::uint8_t {
  kOtherTag,       // Fragment of another (or no) tag.
  kStaleSeq,       // Fragment seq not newer than the last actuated.
  kNotValid,       // Pair expired or beyond the lease at actuation.
  kMissingHalf,    // A lone half at SYNC: an ordinary gap.
  kBlocked,        // Current-tag fragment after an integrity violation.
  kSuperseded,     // Complete pair older than the newest at the same SYNC.
  kNotEnabled,     // Complete valid pair while the drive is not Operation Enabled.
  kDuplicateSync,  // A SYNC sooner than kSyncMinInterval after the previous one.
};

enum class DriveEventKind : std::uint8_t {
  kActuated,   // seq, tag
  kDropped,    // drop, count
  kQuickStop,  // reason
  kStopConfirmed,
  kStopUnconfirmed,
  kFault,      // fault_code
  kIntegrity,  // An integrity violation was detected (D8).
};

// One drive event. Only the fields named next to the kind above are meaningful.
struct DriveEvent {
  DriveEventKind kind{};
  runtime_core::MonoTime at{};
  std::uint16_t seq{};
  std::uint16_t tag{};
  DropReason drop{};
  std::uint32_t count{};
  QuickStopReason reason{};
  std::uint16_t fault_code{};
};

// Receives drive events synchronously. It must not throw and must not call back into the drive.
using EventSink = std::function<void(const DriveEvent&)>;

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

  // Handles one received frame and returns the frames to send in reply (TPDO1-3 after an
  // accepted SYNC in Operational, D7/D13). Never throws for any frame content; the only throw is
  // runtime_core's checked time arithmetic, reachable only with times near +/-292 years. The tag
  // lifetime is enforced first (ExecTagIssuer::tick), so a frame never sees a dead tag.
  std::vector<canopen_codec::Frame> on_frame(const canopen_codec::Frame& frame,
                                             runtime_core::MonoTime now);

  // The W tick. Call it every kTickPeriod, and before on_frame for frames of the same instant;
  // each call advances the joint model by exactly one W. Order: tag lifetime, pending integrity
  // stop, guards (expiry, SYNC loss, command timeout; only while Operation Enabled), joint step,
  // stop monitor (stop_confirmed / stop_unconfirmed, end of a fault reaction).
  void tick(runtime_core::MonoTime now);

  // The tag allocation step of a commit, without the D17 staging. The SDO path (0x2011) calls it
  // after its own checks (staging, CRC, CiA 402 state, idempotent repeat); tests of the drive
  // loop call it directly. Refused (nullopt, the current tag untouched) while the policy latch
  // is set.
  std::optional<std::uint16_t> commit(std::uint64_t generation, std::uint32_t session,
                                      runtime_core::MonoTime now);
  // 0x2015 policy_reset = 1 (D11; the SDO write calls this). Accepted only while latched and
  // after the latest stop is confirmed; then the latch clears and the drive goes to Switch On
  // Disabled. Returns whether it was accepted.
  bool policy_reset(runtime_core::MonoTime now);

  // Injected faults (D11). A recoverable fault ramps in Fault Reaction Active and enters Fault
  // once the stop is confirmed. Torque loss enters Fault at once with no ramp: the joint keeps
  // its velocity, stop_unconfirmed is reported and drive_unavailable stays set for the process.
  void inject_fault(std::uint16_t error_code, runtime_core::MonoTime now);
  void inject_torque_loss(runtime_core::MonoTime now);
  // A stuck joint ignores the brake (tests stop_unconfirmed).
  void inject_stuck_joint(bool stuck) { joint_.set_stuck(stuck); }

  void set_event_sink(EventSink sink) { sink_ = std::move(sink); }

  bool policy_latched() const { return policy_latched_; }
  bool stop_confirmed() const { return stop_confirmed_; }
  bool stop_unconfirmed() const { return stop_unconfirmed_; }
  bool drive_unavailable() const { return drive_unavailable_; }
  // 0x2030:05 (bit layout in drive_flag).
  std::uint16_t drive_flags() const;
  // 0x2016: attempt_id of the last successful commit in this boot (D17).
  std::uint32_t committed_attempt() const { return staging_.committed_attempt(); }
  // The command being tracked; nullopt when nothing actuates.
  const std::optional<Command>& actuated() const { return actuated_; }
  const JointState& joint() const { return joint_.state(); }
  std::uint32_t sync_duplicates() const { return sync_duplicates_; }

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
  class Objects;  // The drive's ObjectAccess (drive.cpp).

  std::vector<canopen_codec::Frame> on_sdo(const canopen_codec::Frame& frame,
                                           runtime_core::MonoTime now);
  // 0x2010 and 0x2011 writes (D17); each returns 0 or an SDO abort code.
  std::uint32_t write_exec_request(std::uint8_t sub, std::uint32_t value,
                                   runtime_core::MonoTime now);
  std::uint32_t write_exec_commit(std::uint32_t crc, runtime_core::MonoTime now);
  // While drive authority is lost, the drive stays in (or re-enters) Fault with the torque-loss
  // code, so the statusword never offers an enable-able state.
  void hold_unavailable_fault();
  void on_pdo(const canopen_codec::Frame& frame, runtime_core::MonoTime now);
  std::vector<canopen_codec::Frame> on_sync(runtime_core::MonoTime now);
  std::vector<canopen_codec::Frame> sample_tpdos(runtime_core::MonoTime now);
  canopen_codec::Frame boot_up_frame() const;

  // Bookkeeping after any CiA 402 transition: leaving Operation Enabled revokes the tag and
  // stops tracking; entering it restarts the guard references; entering Quick Stop Active or
  // Fault Reaction Active starts the stop monitor.
  void after_transition(Cia402State before, runtime_core::MonoTime now);
  void policy_stop(QuickStopReason reason, runtime_core::MonoTime now);
  void note_integrity(runtime_core::MonoTime now);
  void start_stop_monitor(runtime_core::MonoTime now);
  void apply_inhibit();
  void emit(DriveEvent event);
  void emit_drops(const Counters& dropped, runtime_core::MonoTime now);

  canopen_codec::NodeId node_;
  NmtSlave nmt_;
  Cia402 cia402_;
  ExecTagIssuer issuer_;
  CommandAssembler assembler_;
  DispatchCounts counts_;

  JointModel joint_;
  EventSink sink_;
  std::optional<Command> actuated_;
  std::uint16_t applied_seq_{0};  // Last actuated seq and tag, reported in TPDO2/TPDO3.
  std::uint16_t applied_tag_{0};
  std::uint16_t sample_seq_{0};
  std::optional<runtime_core::MonoTime> last_sync_;
  runtime_core::MonoTime sync_ref_{};     // Last SYNC or Operation Enabled entry, if later.
  runtime_core::MonoTime command_ref_{};  // Last actuation or Operation Enabled entry, if later.
  std::uint32_t sync_duplicates_{0};
  bool integrity_pending_{false};    // Detected, quick stop not yet started (next W tick).
  bool integrity_flag_{false};       // drive_flag::kIntegrity; cleared by an accepted policy reset.
  std::uint32_t sample_time_us_{0};  // 0x2030:03 of the latest TPDO sample.
  std::uint16_t controlword_{0};     // 6040h as last received (RPDO3).

  // The commit transaction (D17). last_commit_ is the latest success, kept for the idempotent
  // repeat: the same 0x2011 value again answers with the same tag while that tag is current.
  CommitStaging staging_;
  struct LastCommit {
    CommitRequest request;
    std::uint16_t tag{};
    std::uint64_t epoch{};
  };
  std::optional<LastCommit> last_commit_;
  std::uint32_t exec_commit_written_{0};  // 0x2011 as last written.
  bool policy_latched_{false};
  bool drive_unavailable_{false};
  // Stop monitor of the latest stop (D10).
  bool stop_active_{false};
  runtime_core::MonoTime stop_started_{};
  int quiet_ticks_{0};
  bool stop_confirmed_{false};
  bool stop_unconfirmed_{false};
};

}  // namespace drive_core
