// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "drive_core/joint.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/faults.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

namespace direct_backend {

// The drive's own numbers (E3a/E3b): W = 2 ms, command timeout 30 ms, a_stop = 10 rad/s^2 and
// v_max = 1 rad/s (those two live in drive_core::JointModel).
inline constexpr runtime_core::Duration kStep = drive_core::kTickPeriod;
inline constexpr runtime_core::Duration kCommandTimeout = runtime_core::milliseconds(30);
// C20: the feedback age the backend tolerates while armed is one of the drive profile's values.
inline constexpr runtime_core::Duration kMinFeedbackAge = runtime_core::milliseconds(20);
inline constexpr runtime_core::Duration kMaxFeedbackAge = runtime_core::milliseconds(30);
// C18: distinct fresh samples at rest, the rest threshold, and the host deadline from the
// inhibit (0.100 s braking + Fault Reaction 1 W + 5 samples x T_bus + 2 x T_bus).
inline constexpr int kConfirmSamples = 5;
inline constexpr double kRestVelocity = 0.01;  // rad/s
inline constexpr runtime_core::Duration kStopConfirmDeadline = runtime_core::milliseconds(172);
// Samples the feedback path can hold in flight (a transport delay of up to 64 x W).
inline constexpr std::size_t kSampleRing = 64;
inline constexpr std::size_t kOpRing = 16;

struct DirectConfig {
  // Where the six joints rest before time starts.
  hal_contract::Joints start_position{};
  // The feedback age beyond which an armed backend latches the group inhibit (C17). Outside
  // [20, 30] ms configure() fails (C20).
  runtime_core::Duration max_feedback_age{kMinFeedbackAge};
};

// What a joint did, as the joint saw it. An observer (a test, a trace) receives these through
// an optional sink; the backend never allocates for one.
enum class EventKind : std::uint8_t {
  kActuated,   // a position target was applied this step (approved command or arm hold)
  kDisabled,   // operation disabled (re-arm phase 2, deactivate)
  kCommitted,  // a new execution identity was committed (re-arm phase 3)
  kEnabled,    // operation enabled under the committed identity
  kLocalStop,  // the joint stopped on its own: expiry, command timeout, hold end, injected
  kQuickStop,  // the group inhibit ordered this joint to stop
};

struct Event {
  runtime_core::MonoTime at{};
  std::size_t node{0};
  EventKind kind{EventKind::kActuated};
  double target{0.0};    // kActuated: the applied target
  double position{0.0};  // the joint position when the event happened
  double velocity{0.0};  // the joint velocity when the event happened
  // kActuated: the approved command's source_seq; nullopt for an arm hold.
  std::optional<std::uint64_t> source_seq{};
  // kActuated: the command's identity; kCommitted and kEnabled: the committed identity.
  hal_contract::AuthorityId id{};
};

using EventSink = void (*)(void* context, const Event& event);

// The in-process backend (C11): six drive_core::JointModel behind the hal_contract Backend
// interface, run in logic time. The caller advances it with step(t), once per W; nothing here
// reads a clock. Acceptance of a command takes one W; arming from rest takes one W.
//
// Semantics are the drive's: arm holds the actual position until hold_until; a command tracks
// while now < valid_until; expiry, 30 ms without a new command, or the end of the hold brakes at
// a_stop and latches a policy stop; any unexpected stop, a stale feedback or a failed op latches
// the group inhibit (C17) and every enabled joint is stopped, then each stop is confirmed (C18)
// from the joint's own availability first and then 5 distinct fresh samples at rest.
//
// Not thread-safe. The control path (submit, poll, step, op_state, inhibited, can_apply) does not
// allocate: every buffer is fixed. wait() builds an OpResult and may allocate.
class DirectBackend final : public hal_contract::Backend {
 public:
  // Places every joint at config.start_position at rest. Throws std::invalid_argument on a
  // non-finite or out-of-range start position (|x| > 100 rad).
  explicit DirectBackend(const DirectConfig& config);

  // One device period W at time t: arm and deactivate phases, command intake, the stop decisions,
  // motion, one feedback sample, then stop confirmation and the feedback-age check.
  void step(runtime_core::MonoTime t);

  // ---- hal_contract::Backend ----
  void configure(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  runtime_core::DeviceCapabilities capabilities() const override;
  void activate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  void arm(const hal_contract::OpTicket& ticket, const hal_contract::ArmRequest& request,
           runtime_core::MonoTime now) override;
  void submit(const hal_contract::ApprovedCommand& command, runtime_core::MonoTime now) override;
  hal_contract::Feedback poll(runtime_core::MonoTime now) override;
  void deactivate(const hal_contract::OpTicket& ticket, runtime_core::MonoTime now) override;
  void on_error_nonblocking() override;
  hal_contract::OpState op_state(std::uint64_t op_id) const override;
  std::optional<hal_contract::OpResult> wait(std::uint64_t op_id,
                                             runtime_core::Duration timeout) override;
  bool inhibited() const override { return inhibited_; }
  // Applied at the next step, at most W away: the command must still be open then.
  bool can_apply(runtime_core::MonoTime valid_until, runtime_core::MonoTime now) const override;

  // ---- the joints as they truly are, not as the feedback reports them ----
  hal_contract::Joints joint_position() const;
  hal_contract::Joints joint_velocity() const;
  void set_event_sink(EventSink sink, void* context);

  // ---- fault injection (test seams; each names an effect, not a mechanism) ----
  // While true, no new sample reaches the host: the samples are lost, not queued.
  void stall_feedback(bool on) { stalled_ = on; }
  // From now on a sample reaches the host `d` after its source time, in order.
  void delay_feedback(runtime_core::Duration d) { delay_ = d; }
  // From now on no submitted command reaches the joints.
  void drop_commands() { dropping_ = true; }
  // Joint `node` stops on its own at its next step.
  void inject_stop(std::size_t node);
  // A joint whose drive reports itself unavailable (drive_flags, C18): its stop is
  // unconfirmed whatever its velocity says.
  void set_drive_available(std::size_t node, bool available);

 private:
  enum class Mode : std::uint8_t { kDisabled, kHolding, kTracking, kDisabling, kStopping };

  struct Node {
    drive_core::JointModel model{};
    Mode mode{Mode::kDisabled};
    hal_contract::StopStatus stop{hal_contract::StopStatus::kNone};
    int rest_samples{0};
    bool inject{false};
    bool drive_available{true};
    std::int64_t hold_nrad{0};
  };

  struct ArmOp {
    hal_contract::OpTicket ticket{};
    hal_contract::ArmRequest request{};
    bool disabled{false};
  };

  struct Sample {
    runtime_core::MonoTime at{};
    std::uint64_t seq{0};  // distinct per sample (C18)
    hal_contract::Joints position{};
    hal_contract::Joints velocity{};
    std::array<bool, hal_contract::kJoints> in_stop{};  // statusword: Quick Stop Active
    std::optional<std::uint64_t> applied{};
  };

  struct OpEntry {
    std::uint64_t id{0};
    hal_contract::OpState state{hal_contract::OpState::kUnknown};
    const char* error{nullptr};  // static storage
  };

  static bool enabled(Mode m) { return m == Mode::kHolding || m == Mode::kTracking; }

  void set_op(std::uint64_t id, hal_contract::OpState state, const char* error = nullptr);
  void emit(runtime_core::MonoTime t, std::size_t i, EventKind kind, double target = 0.0,
            std::optional<std::uint64_t> seq = std::nullopt, hal_contract::AuthorityId id = {});
  void fail_arm(const char* error);
  void latch(runtime_core::MonoTime t);
  void progress_arm(runtime_core::MonoTime t);
  void progress_deactivate(runtime_core::MonoTime t);
  void take_command(runtime_core::MonoTime t);
  void local_stops(runtime_core::MonoTime t);
  void move(runtime_core::MonoTime t);
  void sample(runtime_core::MonoTime t);
  void count_rest(const Sample& s);
  void judge_stops(runtime_core::MonoTime t);
  void check_feedback_age(runtime_core::MonoTime t);
  bool all_at_rest() const;

  runtime_core::Duration max_feedback_age_;
  runtime_core::MonoTime now_{};
  std::array<Node, hal_contract::kJoints> nodes_{};

  bool configured_{false};
  bool active_{false};
  bool inhibited_{false};
  runtime_core::MonoTime inhibit_at_{};
  hal_contract::FaultSet faults_{};
  hal_contract::CommandCounters counters_{};

  std::optional<hal_contract::AuthorityId> armed_{};
  runtime_core::MonoTime hold_until_{};
  std::optional<ArmOp> arm_op_{};
  std::optional<hal_contract::OpTicket> deactivate_op_{};

  std::optional<hal_contract::ApprovedCommand> box_{};      // newest submitted, not yet taken
  std::optional<hal_contract::ApprovedCommand> command_{};  // the one the joints track
  runtime_core::MonoTime received_at_{};
  std::optional<std::uint64_t> applied_{};

  bool dropping_{false};
  bool stalled_{false};
  runtime_core::Duration delay_{};
  std::array<Sample, kSampleRing> ring_{};
  std::size_t ring_head_{0};
  std::size_t ring_size_{0};
  std::uint64_t next_sample_seq_{1};
  std::optional<Sample> latest_{};

  std::array<OpEntry, kOpRing> ops_{};
  EventSink sink_{nullptr};
  void* sink_context_{nullptr};
};

}  // namespace direct_backend
