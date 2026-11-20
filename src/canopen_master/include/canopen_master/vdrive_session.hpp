// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "canopen_master/feedback_join.hpp"
#include "canopen_master/master.hpp"
#include "hal_contract/faults.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

// The vdrive session (C9): the six-node lifecycle on top of the master core, in logic time.
// configure probes every node (C2) and resets the vdrive ones, activate brings them to Switched
// On, arm commits a new execution identity group-wide and enables after two hold updates, the
// command stream sends approved commands at each SYNC slot under the dispatch margin (C6), and
// deactivate stops and confirms before Pre-operational. Feedback is joined per node (C4). Any
// unexpected event latches the group motion inhibit (C17) and each node's stop is tracked and
// confirmed from the drive's own evidence (C18). A cancelled arm fences every node it may have
// enabled with NMT Enter Pre-operational (C10).
//
// configure clears the inhibit from whatever an inhibit left behind: it first sends NMT Enter
// Pre-operational to every node still Operational, and until this configure ends the TPDOs of
// those nodes (Quick Stop Active, Fault, an old applied_tag) are not events of the op.
//
// deactivate with a stop that is not confirmed fails, keeps the nodes Operational (TPDOs keep
// flowing; communication is not cut before the stop completes, spec §8.1 STOPPING) and latches
// the inhibit; only a configure goes on from there.
//
// Calling convention (the same queued style as Master):
// - The session owns the master's event queue: the caller calls the session's on_frame and tick
//   instead of the master's, and never drains the master's events itself.
// - Lifecycle ops (configure, activate, arm, deactivate) are posted with an OpTicket {op_id,
//   deadline}. They start at the next on_frame or tick, with that call's time, and progress in
//   later calls. Completion is read with op_state(op_id) / op_error(op_id); there are no
//   callbacks. An op still pending at its deadline fails at the first call at or after it,
//   whether or not anyone waits. Posting an op while another is pending fails the pending one.
// - Every op runs under an epoch. Ending an op bumps it, and every SDO completion and every
//   frame sent for an op checks it first; its SDO requests carry the epoch as their tag and
//   ending the op cancels the ones still queued in the master. So a failed or timed-out op never
//   sends anything later: no SDO request, no RPDO, no 0x0F. (A request already on the wire when
//   the op ends cannot be recalled; its answer is ignored.)
// - A failed op latches the inhibit (C17); only a new configure clears it.
// - Frames the session sends itself (RPDOs) leave with the return value of the call that made
//   them, after the master's frames of that call, so a SYNC is always followed by that slot's
//   RPDOs. An NMT or SDO request the session queues while handling a received frame leaves at
//   the next call (the master applies queued commands at the start of tick); ops that fail at
//   their deadline do so at the start of a call, so their fence leaves with that call.
//
// Pure logic: no sockets, threads or clock reads. Not thread-safe; one caller serializes every
// call (in E3c, the I/O worker).
namespace canopen_master {

// Project values (E3a/E3b) the session's rules are built from.
inline constexpr runtime_core::Duration kBusPeriod{10'000'000};         // T_bus
inline constexpr runtime_core::Duration kDriveTick{2'000'000};          // W
inline constexpr runtime_core::Duration kDeliveryAllowance{1'000'000};  // C6
inline constexpr runtime_core::Duration kMaxLease{48'000'000};
inline constexpr runtime_core::Duration kMinFeedbackAge{20'000'000};
inline constexpr runtime_core::Duration kMaxFeedbackAge{30'000'000};
inline constexpr runtime_core::Duration kBootQuarantine{100'000'000};  // D3
// C18: 0.100 s braking + one W of fault reaction + 5 samples x T_bus + worst SYNC phase T_bus +
// transport allowance T_bus.
inline constexpr runtime_core::Duration kStopConfirmDeadline{172'000'000};
// C2: every node reports the armed tag and a stream seq this long after the enable.
inline constexpr runtime_core::Duration kArmConfirmWindow{40'000'000};  // 4 x T_bus
inline constexpr int kRestSamples = 5;
inline constexpr std::int32_t kRestVelocityUradS = 10'000;  // 0.01 rad/s
// The EMCY code of the drive's torque loss (drive_core kErrorTorqueLoss, CiA 301 device range).
inline constexpr std::uint16_t kEmcyTorqueLoss = 0xFF01;
// 0x2030:05 drive_flags bits (the EDS comment at [2030sub5]).
inline constexpr std::uint16_t kFlagDriveUnavailable = 1U << 1U;
inline constexpr std::uint16_t kFlagStopUnconfirmed = 1U << 3U;

// Controlwords (CiA 402, 6040h) the session sends by RPDO3.
namespace controlword {
inline constexpr std::uint16_t kShutdown = 0x06;
inline constexpr std::uint16_t kSwitchOn = 0x07;  // Also Disable Operation from Operation Enabled.
inline constexpr std::uint16_t kEnableOperation = 0x0F;
inline constexpr std::uint16_t kQuickStop = 0x02;
}  // namespace controlword

// CiA 402 state from a statusword (6041h).
enum class DriveState : std::uint8_t {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
};
DriveState decode_statusword(std::uint16_t statusword);

// The 0x1018 vendor, product and revision the EDS states (what vdrive_bank reports).
struct Identity {
  std::uint32_t vendor{};
  std::uint32_t product{};
  std::uint32_t revision{};
  friend bool operator==(const Identity&, const Identity&) = default;
};
Identity eds_identity();

struct SessionConfig {
  std::array<canopen_codec::NodeId, kSessionNodes> nodes{
      canopen_codec::NodeId::make(1).value(), canopen_codec::NodeId::make(2).value(),
      canopen_codec::NodeId::make(3).value(), canopen_codec::NodeId::make(4).value(),
      canopen_codec::NodeId::make(5).value(), canopen_codec::NodeId::make(6).value()};
  // C20: exactly 10 ms; it must equal the master's sync_period.
  runtime_core::Duration bus_period{kBusPeriod};
  // C20: (0, 48] ms. Hold commands carry min(hold_until, slot + lease).
  runtime_core::Duration lease{kMaxLease};
  // C20: [20, 30] ms. While armed, an older group sample latches the inhibit (C17).
  runtime_core::Duration max_feedback_age{kMinFeedbackAge};
};

enum class NodeProfile : std::uint8_t { kUnknown, kVirtual, kGeneric };

enum class SessionPhase : std::uint8_t {
  kIdle,  // Before the first configure.
  kConfiguring,
  kConfigured,  // Pre-operational (also after deactivate).
  kActivating,
  kActive,  // Switched On, no tag in use (also after a failed arm).
  kArming,
  kArmed,
  kDeactivating,
};

// What the probe of configure read from one node (C2); std::nullopt for a read that failed.
struct Probe {
  std::optional<std::uint32_t> device_type;
  std::array<std::optional<std::uint32_t>, 4> identity{};  // 0x1018:01..04
  std::optional<std::uint32_t> boot_id;
  std::optional<std::uint32_t> command_timeout_ms;
  std::optional<std::uint32_t> committed_attempt;
  std::optional<std::uint32_t> mode_display;
  std::optional<std::uint32_t> exec_tag;
};

// Read-only view of one node.
struct NodeView {
  std::uint8_t node{0};  // Node-ID.
  NodeProfile profile{NodeProfile::kUnknown};
  Probe probe;
  std::optional<std::uint32_t> boot_id;            // Latest read of 0x2013.
  std::optional<std::uint32_t> committed_attempt;  // Latest read of 0x2016.
  std::optional<std::uint16_t> pending_tag;        // Committed, not yet seen in applied_tag.
  std::optional<std::uint16_t> armed_tag;          // Seen in applied_tag (C17).
  hal_contract::StopStatus stop{hal_contract::StopStatus::kNone};
  bool fenced{false};  // Sent NMT Enter Pre-operational by a cancelled arm (C10).
  int commit_retries{0};
};

struct SessionCounts {
  // Events that latched (or, once latched, would have latched) the inhibit (C17).
  std::uint32_t unexpected{};
  // Offers dropped because the session was not armed or the identity was not the armed one.
  std::uint32_t discarded{};
  // Offers refused because a position is non-finite or beyond the wire's int32 urad.
  std::uint32_t refused{};
  // Commit retries after a refusal inside the boot quarantine (C9 phase 3).
  std::uint32_t quarantine_retries{};
};

// C6 for a session that is not at hand (the I/O worker's control side): whether a command closing
// at `valid_until`, offered at `now`, can still be dispatched when the master's last SYNC was at
// `last_sync` (std::nullopt before the first) on a grid of `bus_period`. VdriveSession::can_apply
// is this with its own last SYNC.
bool command_can_apply(std::optional<runtime_core::MonoTime> last_sync,
                       runtime_core::Duration bus_period, runtime_core::MonoTime valid_until,
                       runtime_core::MonoTime now);

class VdriveSession {
 public:
  // Registers the TPDO1-3 layouts of every node with the master. Throws std::invalid_argument
  // for a configuration outside C20 (the message lists the allowed values), duplicate nodes, or
  // a node the master does not know.
  VdriveSession(Master& master, SessionConfig config);

  // Call these instead of Master::on_frame / Master::tick; they return every frame to send.
  std::vector<canopen_codec::Frame> on_frame(const canopen_codec::Frame& frame,
                                             runtime_core::MonoTime now);
  std::vector<canopen_codec::Frame> tick(runtime_core::MonoTime now);

  // ---- lifecycle ops (posted; see the calling convention above) ----
  void configure(const hal_contract::OpTicket& ticket);
  void activate(const hal_contract::OpTicket& ticket);
  void arm(const hal_contract::OpTicket& ticket, const hal_contract::ArmRequest& request);
  void deactivate(const hal_contract::OpTicket& ticket);

  // kPending from posting until the op ends; kUnknown for an id not among the last kOpRing.
  hal_contract::OpState op_state(std::uint64_t op_id) const;
  // Why the op failed (static storage); nullptr unless it failed.
  const char* op_error(std::uint64_t op_id) const;

  // ---- control path (no allocation) ----
  // An approved command for the armed identity. The newest offer wins at the next slot; one
  // overwritten before it is counted as superseded. Dropped (counted as discarded) unless armed,
  // not inhibited and `command.id` is the armed identity; refused (counted as refused) when a
  // position is not finite or does not fit the wire's int32 urad. Never clamped.
  void offer(const hal_contract::ApprovedCommand& command);
  // C6: whether a command closing at `valid_until` offered at `now` can still be dispatched: its
  // deadline floored to 1 us must be strictly later than the SYNC after its dispatch slot + W +
  // 1 ms.
  bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const;
  // on_error: latches the inhibit at `now`; its frames leave with the next call.
  void inhibit(runtime_core::MonoTime now);
  // C10: fails the op `op_id` now, as its deadline would (the caller's wait timed out). It ends
  // with `why` (static storage), the epoch moves on, the inhibit latches and a cancelled arm
  // fences; its frames leave with the next call. Nothing for an op that ended or is not known.
  void cancel(std::uint64_t op_id, const char* why, runtime_core::MonoTime now);
  // Called with every master event, inside on_frame and tick, before the session handles it
  // (the I/O worker forwards them). It must not call into the session or the master.
  using EventObserver = std::function<void(const MasterEvent&)>;
  void set_event_observer(EventObserver observer) { observer_ = std::move(observer); }
  // The instant of the master's last SYNC; std::nullopt before the first.
  std::optional<runtime_core::MonoTime> last_sync() const { return last_sync_; }
  bool inhibited() const { return inhibited_; }

  // The robot snapshot (C4) with faults, stop states and counters.
  hal_contract::Feedback feedback() const;
  // C2: generic if any node probed generic, or once an arm failed confirmation; otherwise
  // virtual (provisional until profile_confirmed()).
  runtime_core::DeviceCapabilities capabilities() const;
  bool profile_confirmed() const { return profile_confirmed_; }
  std::optional<hal_contract::AuthorityId> armed_id() const { return armed_id_; }
  SessionPhase phase() const { return phase_; }
  NodeView node(std::size_t index) const;
  const SessionCounts& counts() const { return counts_; }
  const FeedbackJoin& join() const { return join_; }
  // What the stream sent under a wire command_seq, among the last kSeqRing dispatches: an
  // approved command (with its source_seq) or an arm hold. For observers of the device side
  // (traces, the contract fixture); std::nullopt for a seq not among them.
  struct Dispatch {
    bool approved{false};
    std::uint64_t source_seq{0};
  };
  std::optional<Dispatch> dispatched(std::uint16_t wire_seq) const;

  static constexpr std::size_t kOpRing = 16;
  static constexpr std::size_t kSeqRing = 64;

 private:
  enum class OpKind : std::uint8_t { kConfigure, kActivate, kArm, kDeactivate };
  enum class ArmStage : std::uint8_t { kDisable, kCommit, kHold, kConfirm };
  enum class CommitStep : std::uint8_t { kIdle, kWriting, kWaitRetry, kReading, kDone };
  enum class StopMode : std::uint8_t { kSamples, kSdoPoll };

  struct Op {
    std::uint64_t id{0};
    OpKind kind{OpKind::kConfigure};
    runtime_core::MonoTime deadline{};
    hal_contract::ArmRequest request{};
  };
  struct OpRecord {
    std::uint64_t id{0};
    hal_contract::OpState state{hal_contract::OpState::kUnknown};
    const char* error{nullptr};
  };

  struct StopTrack {
    hal_contract::StopStatus status{hal_contract::StopStatus::kNone};
    StopMode mode{StopMode::kSamples};
    std::uint8_t accept{0};  // Bit per DriveState accepted at rest.
    runtime_core::MonoTime started{};
    int rest{0};
    std::optional<std::uint16_t> last_seq;
    bool flags_pending{false};
    std::uint64_t gen{0};  // Bumped whenever the track restarts: older SDO answers are ignored.
    // kSdoPoll: one round of 0x6041, 0x606C, 0x2030:05 at a time, every T_bus.
    bool round_out{false};
    int round_left{0};
    runtime_core::MonoTime next_round{};
    std::uint16_t poll_statusword{};
    std::int32_t poll_velocity{};
    std::uint16_t poll_flags{};
    bool poll_failed{false};
    std::uint32_t rounds{0};
  };

  struct NodeState {
    NodeProfile profile{NodeProfile::kUnknown};
    Probe probe;
    std::optional<std::uint32_t> boot_id;
    std::optional<std::uint32_t> committed_attempt;
    std::optional<runtime_core::MonoTime> boot_at;  // Latest boot-up that started a new boot.
    bool awaiting_boot{false};                      // configure sent Reset Communication.
    bool leaving_operational{false};  // configure sent Enter Pre-operational to this node.
    std::optional<canopen_codec::NmtState> commanded;
    std::optional<canopen_codec::NmtState> previous;
    runtime_core::MonoTime commanded_at{};
    std::optional<std::uint16_t> pending_tag;
    std::optional<std::uint16_t> armed_tag;
    std::uint16_t stream_tag{0};  // The tag the RPDOs of this arm carry.
    bool activated{false};        // A fresh sample in Switched On during activate.
    bool enable_sent{false};      // 0x0F sent in the current arm.
    bool confirmed{false};        // C2 confirmation of the current arm.
    bool torque_loss{false};
    bool fenced{false};
    CommitStep commit{CommitStep::kIdle};
    std::uint32_t attempt{0};
    bool retried{false};
    int retries{0};
    runtime_core::MonoTime retry_at{};
    StopTrack stop;
  };

  struct Stream {
    bool approved{false};  // An approved command (else a hold).
    std::uint64_t source_seq{0};
    std::array<std::int32_t, kSessionNodes> target_urad{};
    runtime_core::MonoTime valid_until{};
    std::uint16_t seq{0};
    bool dispatched{false};
  };
  struct SeqEntry {
    bool used{false};
    std::uint16_t seq{0};
    bool approved{false};
    std::uint64_t source_seq{0};
  };

  // Calls.
  void before(runtime_core::MonoTime now);
  std::vector<canopen_codec::Frame> after(std::vector<canopen_codec::Frame> frames,
                                          runtime_core::MonoTime now);
  void handle(const MasterEvent& event, runtime_core::MonoTime now);
  void handle_pdo(std::size_t i, const PdoDetail& pdo, runtime_core::MonoTime now);
  void on_sample(std::size_t i, const NodeSample& s, runtime_core::MonoTime now);

  // Ops.
  void post(OpKind kind, const hal_contract::OpTicket& ticket,
            const hal_contract::ArmRequest& request = {});
  void start(const Op& op, runtime_core::MonoTime now);
  void start_configure(runtime_core::MonoTime now);
  void start_activate(runtime_core::MonoTime now);
  void start_arm(runtime_core::MonoTime now);
  void start_deactivate(runtime_core::MonoTime now);
  void succeed();
  // Ends the current op as failed and latches the inhibit.
  void fail(const char* why, runtime_core::MonoTime now);
  void end_op(hal_contract::OpState state, const char* error);
  // Ends the current op, if any, as failed and leaves the phase it stopped in; true
  // for an arm. Does not latch.
  bool end_failed(const char* why);
  void record(std::uint64_t id, hal_contract::OpState state, const char* error);
  bool op_is(OpKind kind) const { return op_ && op_->kind == kind; }

  // configure
  void probe(std::size_t i, std::size_t k);
  void probe_done(runtime_core::MonoTime now);
  void reread(std::size_t i);
  // arm
  void arm_disable_or_commit(runtime_core::MonoTime now);
  void commit(std::size_t i);
  void commit_write(std::size_t i, std::size_t k);
  void commit_failed(std::size_t i, bool on_commit_write, runtime_core::MonoTime now);
  void commit_read(std::size_t i);
  void commits_progress(runtime_core::MonoTime now);
  void arm_progress(runtime_core::MonoTime now);
  // deactivate
  void deactivate_progress(runtime_core::MonoTime now);

  // Stream (C6).
  void slot(runtime_core::MonoTime now);
  bool open_at_slot(runtime_core::MonoTime valid_until, runtime_core::MonoTime slot) const;
  void send_stream(Stream& s);
  void remember(const Stream& s);
  const SeqEntry* lookup(std::uint16_t seq) const;
  void stop_streaming();

  // Inhibit and stops (C17, C18).
  void latch(std::optional<hal_contract::Fault> fault, runtime_core::MonoTime now,
             bool cancelled_arm = false);
  void begin_stop(std::size_t i, std::uint8_t accept, runtime_core::MonoTime now);
  void stop_on_sample(std::size_t i, const NodeSample& s);
  void request_flags(std::size_t i);
  void poll(std::size_t i, runtime_core::MonoTime now);
  void poll_round_done(std::size_t i);
  void resolve(std::size_t i, hal_contract::StopStatus status);
  bool stops_resolved() const;
  void check_state(std::size_t i, DriveState state, runtime_core::MonoTime now);
  void check_tag(std::size_t i, const NodeSample& s, runtime_core::MonoTime now);
  void command_nmt(std::size_t i, canopen_codec::NmtCommand command, runtime_core::MonoTime now);

  void rpdo3(std::size_t i, std::uint16_t cw);
  std::optional<std::size_t> index_of(std::uint8_t node) const;

  Master& master_;
  SessionConfig config_;
  std::array<NodeState, kSessionNodes> nodes_;
  FeedbackJoin join_;
  SessionPhase phase_{SessionPhase::kIdle};
  runtime_core::MonoTime now_{};
  std::vector<canopen_codec::Frame> extra_;

  std::optional<Op> posted_;
  std::optional<Op> op_;
  runtime_core::MonoTime op_started_{};
  std::uint64_t epoch_{1};
  std::array<OpRecord, kOpRing> ops_{};
  std::size_t ops_next_{0};
  int probe_left_{0};
  int reset_left_{0};

  ArmStage stage_{ArmStage::kDisable};
  int hold_updates_{0};
  runtime_core::MonoTime enable_at_{};
  runtime_core::MonoTime hold_until_{};
  hal_contract::ArmRequest arm_request_{};
  std::optional<hal_contract::AuthorityId> armed_id_;

  bool inhibited_{false};
  // While set, the error a latch gives the op it ends (static storage); else the generic one.
  const char* latch_reason_{nullptr};
  hal_contract::FaultSet faults_{};
  hal_contract::CommandCounters counters_{};
  SessionCounts counts_{};
  bool profile_confirmed_{false};
  bool profile_dropped_{false};

  std::optional<hal_contract::ApprovedCommand> box_;
  std::optional<Stream> current_;
  bool holding_{false};
  std::array<std::int32_t, kSessionNodes> hold_urad_{};
  std::uint16_t next_seq_{1};
  std::array<SeqEntry, kSeqRing> seqs_{};
  std::size_t seqs_next_{0};
  std::optional<runtime_core::MonoTime> last_sync_;
  EventObserver observer_;
};

}  // namespace canopen_master
