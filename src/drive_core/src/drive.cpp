// SPDX-License-Identifier: Apache-2.0
#include "drive_core/drive.hpp"

#include <limits>

#include "drive_core/validity.hpp"

namespace drive_core {

namespace {

namespace cc = canopen_codec;

constexpr std::uint16_t kCobNmt = 0x000;
constexpr std::uint16_t kCobHeartbeatBase = 0x700;
constexpr std::uint16_t kCobSdoRequestBase = 0x600;
constexpr std::uint16_t kNodeMask = 0x07F;
constexpr std::uint16_t kFunctionMask = 0x780;

// The boot_id of the boot after a reset node. The process draws the first boot_id at start; a
// reset node within the process derives the next one (MurmurHash3 finalizer of the previous id
// plus one), which is never equal to the previous id.
std::uint32_t next_boot_id(std::uint32_t previous) {
  std::uint32_t h = previous + 1U;
  h ^= h >> 16U;
  h *= 0x85EB'CA6BU;
  h ^= h >> 13U;
  h *= 0xC2B2'AE35U;
  h ^= h >> 16U;
  return h != previous ? h : h + 1U;
}

}  // namespace

std::optional<FrameKind> classify(cc::NodeId node, const cc::Frame& frame) {
  const std::uint16_t cob = frame.cob_id;
  const auto own = [node](std::uint16_t base) {
    return static_cast<std::uint16_t>(base + node.value());
  };
  if (cob == kCobNmt) {
    return FrameKind::kNmt;
  }
  if (cob == cc::od::kSyncCobId) {
    return FrameKind::kSync;
  }
  const auto function = static_cast<std::uint16_t>(cob & kFunctionMask);
  const bool has_node = (cob & kNodeMask) != 0U;
  if (function == cc::od::kEmcyCobIdBase && has_node) {
    return FrameKind::kEmcy;  // Another node's EMCY (this node only produces its own).
  }
  if (function == kCobHeartbeatBase && has_node) {
    return FrameKind::kHeartbeat;
  }
  if (cob == own(kCobSdoRequestBase)) {
    return FrameKind::kSdo;
  }
  if (cob == own(cc::od::Rpdo1::kCobIdBase) || cob == own(cc::od::Rpdo2::kCobIdBase) ||
      cob == own(cc::od::Rpdo3::kCobIdBase)) {
    return FrameKind::kPdo;
  }
  return std::nullopt;
}

Drive::Drive(cc::NodeId node, std::uint32_t boot_id, runtime_core::MonoTime now)
    : node_(node), issuer_(boot_id, now), assembler_(issuer_) {}

cc::Frame Drive::boot_up_frame() const {
  return cc::encode_heartbeat({node_, cc::NmtState::kBootUp});
}

std::vector<cc::Frame> Drive::power_on() {
  cia402_.complete_initialisation();
  nmt_.boot_up();
  return {boot_up_frame()};
}

std::vector<cc::Frame> Drive::on_frame(const cc::Frame& frame, runtime_core::MonoTime now) {
  // The tag lifetime first (D3): a fragment must never meet a tag that is already dead.
  issuer_.tick(now);
  const std::optional<FrameKind> kind = classify(node_, frame);
  if (!kind) {
    ++counts_.foreign;
    return {};
  }
  if (!nmt_.accepts(*kind)) {
    ++counts_.gated;
    return {};
  }
  switch (*kind) {
    case FrameKind::kNmt:
      return on_nmt(frame, now);
    case FrameKind::kHeartbeat:
      // Heartbeat consumption is not part of this drive's contract.
      if (cc::decode_heartbeat(frame)) {
        ++counts_.heartbeat;
      } else {
        ++counts_.malformed;
      }
      return {};
    case FrameKind::kSync:
      if (!cc::is_sync(frame)) {
        ++counts_.malformed;
        return {};
      }
      ++counts_.sync;
      return on_sync(now);
    case FrameKind::kEmcy:
      if (cc::decode_emcy(frame)) {
        ++counts_.emcy;
      } else {
        ++counts_.malformed;
      }
      return {};
    case FrameKind::kSdo:
      return on_sdo(frame, now);
    case FrameKind::kPdo:
      on_pdo(frame, now);
      return {};
  }
  return {};
}

std::vector<cc::Frame> Drive::on_nmt(const cc::Frame& frame, runtime_core::MonoTime now) {
  const std::optional<cc::NmtMessage> message = cc::decode_nmt(frame);
  if (!message) {
    ++counts_.malformed;
    return {};
  }
  if (message->target && *message->target != node_) {
    ++counts_.foreign;
    return {};
  }
  ++counts_.nmt;
  const bool was_operational = nmt_.state() == cc::NmtState::kOperational;
  const NmtAction action = nmt_.on_command(message->command);
  // Amended D12: leaving Operational while Operation Enabled is a policy stop (comm_state).
  if (was_operational && nmt_.state() != cc::NmtState::kOperational &&
      cia402_.state() == Cia402State::kOperationEnabled) {
    policy_stop(QuickStopReason::kCommState, now);
  }
  switch (action) {
    case NmtAction::kNone:
      return {};
    case NmtAction::kResetCommunication:
      // D12: the tag dies, session and generation clear, per-command state clears; boot_id and
      // the tag allocator stay. The CiA 402 state is application state and stays too, and so
      // does the policy latch (D11).
      issuer_.reset_communication();
      staging_.reset_communication();  // The highest committed attempt_id stays (D17).
      last_commit_.reset();
      assembler_.reset();
      actuated_.reset();
      applied_seq_ = 0;
      applied_tag_ = 0;
      break;
    case NmtAction::kResetNode: {
      // D3: a new boot with a new boot_id, a new tag start and a new quarantine. The policy
      // latch survives (D11): the re-initialised CiA 402 machine gets the inhibit again, and a
      // lost drive authority puts it straight back into Fault (after_transition). The joint is
      // physical and keeps its motion.
      const Cia402State before = cia402_.state();
      issuer_.boot(next_boot_id(issuer_.boot_id()), now);
      staging_.new_boot();  // attempt_ids start over only with a new boot (D17).
      last_commit_.reset();
      assembler_.reset();
      actuated_.reset();
      applied_seq_ = 0;
      applied_tag_ = 0;
      cia402_ = Cia402{};
      cia402_.complete_initialisation();
      apply_inhibit();
      after_transition(before, now);
      break;
    }
  }
  nmt_.boot_up();
  return {boot_up_frame()};
}

void Drive::on_pdo(const cc::Frame& frame, runtime_core::MonoTime now) {
  if (const auto rpdo1 = cc::unpack_rpdo1(node_, frame)) {
    ++counts_.pdo;
    if (assembler_.offer(*rpdo1) == OfferResult::kIntegrityViolation) {
      note_integrity(now);
    }
  } else if (const auto rpdo2 = cc::unpack_rpdo2(node_, frame)) {
    ++counts_.pdo;
    if (assembler_.offer(*rpdo2) == OfferResult::kIntegrityViolation) {
      note_integrity(now);
    }
  } else if (const auto rpdo3 = cc::unpack_rpdo3(node_, frame)) {
    ++counts_.pdo;
    controlword_ = rpdo3->controlword;
    const Cia402State before = cia402_.state();
    cia402_.on_controlword(rpdo3->controlword);
    after_transition(before, now);
  } else {
    ++counts_.malformed;
  }
}

namespace {

constexpr std::int64_t kNradPerUrad = 1'000;

// µrad (or µrad/s) on the wire from the model's nrad, truncated and saturated to int32.
std::int32_t to_wire_micro(std::int64_t nano) {
  const std::int64_t micro = nano / kNradPerUrad;
  constexpr std::int64_t lo = std::numeric_limits<std::int32_t>::min();
  constexpr std::int64_t hi = std::numeric_limits<std::int32_t>::max();
  return static_cast<std::int32_t>(micro < lo ? lo : (micro > hi ? hi : micro));
}

}  // namespace

void Drive::emit(DriveEvent event) {
  if (sink_) {
    sink_(event);
  }
}

void Drive::emit_drops(const Counters& dropped, runtime_core::MonoTime now) {
  const auto one = [&](DropReason reason, std::uint32_t count) {
    if (count != 0U) {
      DriveEvent e{DriveEventKind::kDropped, now};
      e.drop = reason;
      e.count = count;
      emit(e);
    }
  };
  one(DropReason::kOtherTag, dropped.other_tag);
  one(DropReason::kStaleSeq, dropped.stale_seq);
  one(DropReason::kNotValid, dropped.not_valid);
  one(DropReason::kMissingHalf, dropped.missing_half);
  one(DropReason::kBlocked, dropped.blocked);
  one(DropReason::kSuperseded, dropped.superseded);
}

void Drive::apply_inhibit() {
  cia402_.set_operation_enable_inhibited(policy_latched_ || drive_unavailable_);
}

void Drive::start_stop_monitor(runtime_core::MonoTime now) {
  stop_active_ = true;
  stop_started_ = now;
  quiet_ticks_ = 0;
  stop_confirmed_ = false;
  stop_unconfirmed_ = false;
}

void Drive::hold_unavailable_fault() {
  if (drive_unavailable_ && cia402_.state() != Cia402State::kFault) {
    cia402_.fault_without_reaction(kErrorTorqueLoss);
  }
}

void Drive::after_transition(Cia402State before, runtime_core::MonoTime now) {
  // A Fault Reset or a reset node cannot leave Fault while the drive authority is lost.
  hold_unavailable_fault();
  const Cia402State after = cia402_.state();
  if (before == after) {
    return;
  }
  if (before == Cia402State::kOperationEnabled) {
    // Disable, stop or fault: nothing actuates any more and the tag dies (D11). A stop by
    // controlword revokes it too, so transition 16 cannot resume the old command stream.
    actuated_.reset();
    issuer_.revoke();
  }
  if (after == Cia402State::kOperationEnabled) {
    // The guards count from the entry; without a tag the command timeout still runs.
    sync_ref_ = now;
    command_ref_ = now;
  }
  if (after == Cia402State::kQuickStopActive || after == Cia402State::kFaultReactionActive) {
    start_stop_monitor(now);
  }
}

void Drive::policy_stop(QuickStopReason reason, runtime_core::MonoTime now) {
  actuated_.reset();
  issuer_.revoke();
  if (policy_latched_) {
    return;  // Already latched: the first stop stands, its monitor keeps running.
  }
  const Cia402State before = cia402_.state();
  cia402_.quick_stop(reason);  // False outside Operation Enabled; the latch applies regardless.
  after_transition(before, now);
  policy_latched_ = true;
  apply_inhibit();
  if (!stop_active_) {
    start_stop_monitor(now);
  }
  DriveEvent e{DriveEventKind::kQuickStop, now};
  e.reason = reason;
  emit(e);
}

void Drive::note_integrity(runtime_core::MonoTime now) {
  // D8: normal actuation is blocked at once; the quick stop starts at the next W tick.
  actuated_.reset();
  integrity_pending_ = true;
  integrity_flag_ = true;
  emit(DriveEvent{DriveEventKind::kIntegrity, now});
}

std::vector<cc::Frame> Drive::on_sync(runtime_core::MonoTime now) {
  if (last_sync_ && now - *last_sync_ < kSyncMinInterval) {
    ++sync_duplicates_;  // D7: counted and ignored.
    DriveEvent e{DriveEventKind::kDropped, now};
    e.drop = DropReason::kDuplicateSync;
    e.count = 1;
    emit(e);
    return {};
  }
  last_sync_ = now;
  sync_ref_ = now;
  const std::uint32_t now_us = to_wire_us(now);
  const AssemblyResult result = assembler_.on_sync(now_us);
  // Every violation was already seen at ingestion; this catches one the drive has not acted on.
  if (result.integrity_violation && !integrity_pending_ && !policy_latched_) {
    note_integrity(now);
  }
  emit_drops(result.dropped, now);
  if (result.command) {
    const Command& cmd = *result.command;
    if (cia402_.state() != Cia402State::kOperationEnabled || integrity_pending_ ||
        policy_latched_) {
      DriveEvent e{DriveEventKind::kDropped, now};
      e.drop = DropReason::kNotEnabled;
      e.count = 1;
      emit(e);
    } else if (!validity_open(cmd.valid_until_us, now_us, kLeaseUs)) {
      // D9: validity is checked again when the candidate is actuated.
      DriveEvent e{DriveEventKind::kDropped, now};
      e.drop = DropReason::kNotValid;
      e.count = 1;
      emit(e);
    } else {
      actuated_ = cmd;
      command_ref_ = now;
      applied_seq_ = cmd.command_seq;
      applied_tag_ = cmd.exec_tag;
      DriveEvent e{DriveEventKind::kActuated, now};
      e.seq = cmd.command_seq;
      e.tag = cmd.exec_tag;
      emit(e);
    }
  }
  if (nmt_.state() != cc::NmtState::kOperational) {
    return {};  // D12: no PDO outside Operational.
  }
  return sample_tpdos(now);
}

std::vector<cc::Frame> Drive::sample_tpdos(runtime_core::MonoTime now) {
  ++sample_seq_;
  sample_time_us_ = to_wire_us(now);
  const JointState& joint = joint_.state();
  return {
      cc::pack_tpdo1(node_, cc::od::Tpdo1{cia402_.statusword(), to_wire_micro(joint.position_nrad),
                                          sample_seq_}),
      cc::pack_tpdo2(
          node_, cc::od::Tpdo2{to_wire_micro(joint.velocity_nrad_s), sample_seq_, applied_seq_}),
      cc::pack_tpdo3(node_, cc::od::Tpdo3{sample_time_us_, sample_seq_, applied_tag_}),
  };
}

void Drive::tick(runtime_core::MonoTime now) {
  issuer_.tick(now);
  if (integrity_pending_) {
    integrity_pending_ = false;
    policy_stop(QuickStopReason::kIntegrity, now);
  }
  if (cia402_.state() == Cia402State::kOperationEnabled) {
    if (actuated_ && !validity_open(actuated_->valid_until_us, to_wire_us(now), kLeaseUs)) {
      policy_stop(QuickStopReason::kExpiry, now);
    } else if (now - sync_ref_ >= kSyncTimeout) {
      policy_stop(QuickStopReason::kSyncLoss, now);
    } else if (now - command_ref_ >= kCommandTimeout) {
      policy_stop(QuickStopReason::kTimeout, now);
    }
  }

  if (drive_unavailable_) {
    joint_.coast();
  } else if (cia402_.state() == Cia402State::kOperationEnabled && actuated_) {
    joint_.track(static_cast<std::int64_t>(actuated_->target_position) * kNradPerUrad);
  } else {
    joint_.brake();  // Every other state holds: decelerate on the quick-stop ramp to rest.
  }

  if (stop_active_) {
    const std::int64_t v = joint_.state().velocity_nrad_s;
    quiet_ticks_ = (v < kQuietVelocityNradS && v > -kQuietVelocityNradS) ? quiet_ticks_ + 1 : 0;
    if (quiet_ticks_ >= kQuietTicks) {
      stop_active_ = false;
      stop_confirmed_ = true;
      emit(DriveEvent{DriveEventKind::kStopConfirmed, now});
      if (cia402_.state() == Cia402State::kFaultReactionActive) {
        const Cia402State before = cia402_.state();
        cia402_.fault_reaction_done();
        after_transition(before, now);
      }
    } else if (!stop_unconfirmed_ && now - stop_started_ >= kStopDeadline) {
      stop_unconfirmed_ = true;  // The monitor keeps watching; a later rest is still reported.
      emit(DriveEvent{DriveEventKind::kStopUnconfirmed, now});
    }
  }
}

std::optional<std::uint16_t> Drive::commit(std::uint64_t generation, std::uint32_t session,
                                           runtime_core::MonoTime now) {
  if (policy_latched_) {
    return std::nullopt;  // D11: refused while latched; the (already revoked) tag stays dead.
  }
  return issuer_.commit(generation, session, now);
}

bool Drive::policy_reset(runtime_core::MonoTime now) {
  if (!policy_latched_ || !stop_confirmed_) {
    return false;
  }
  policy_latched_ = false;
  integrity_flag_ = false;
  apply_inhibit();
  const Cia402State before = cia402_.state();
  switch (before) {
    case Cia402State::kReadyToSwitchOn:
    case Cia402State::kSwitchedOn:
    case Cia402State::kOperationEnabled:
    case Cia402State::kQuickStopActive:
      cia402_.on_controlword(0x0000);  // Disable Voltage: transitions 7, 10, 9, 12.
      break;
    case Cia402State::kNotReadyToSwitchOn:
    case Cia402State::kSwitchOnDisabled:
    case Cia402State::kFaultReactionActive:
    case Cia402State::kFault:
      break;
  }
  after_transition(before, now);
  return true;
}

void Drive::inject_fault(std::uint16_t error_code, runtime_core::MonoTime now) {
  const Cia402State before = cia402_.state();
  cia402_.drive_fault(error_code);
  actuated_.reset();
  issuer_.revoke();
  after_transition(before, now);
  DriveEvent e{DriveEventKind::kFault, now};
  e.fault_code = error_code;
  emit(e);
}

void Drive::inject_torque_loss(runtime_core::MonoTime now) {
  const Cia402State before = cia402_.state();
  drive_unavailable_ = true;
  // No reaction: there is no actuator to run one. The torque-loss code replaces a fault that was
  // already being handled (Fault Reaction Active or Fault): the authority loss supersedes it.
  cia402_.fault_without_reaction(kErrorTorqueLoss);
  actuated_.reset();
  issuer_.revoke();
  after_transition(before, now);
  apply_inhibit();
  // Never a controlled stop: no monitor, the stop is reported unconfirmed at once.
  stop_active_ = false;
  stop_confirmed_ = false;
  stop_unconfirmed_ = true;
  DriveEvent fault{DriveEventKind::kFault, now};
  fault.fault_code = kErrorTorqueLoss;
  emit(fault);
  emit(DriveEvent{DriveEventKind::kStopUnconfirmed, now});
}

std::uint16_t Drive::drive_flags() const {
  std::uint16_t flags = 0;
  if (policy_latched_) {
    flags |= drive_flag::kPolicyLatched;
  }
  if (drive_unavailable_) {
    flags |= drive_flag::kDriveUnavailable;
  }
  if (integrity_flag_ || integrity_pending_) {
    flags |= drive_flag::kIntegrity;
  }
  if (stop_unconfirmed_) {
    flags |= drive_flag::kStopUnconfirmed;
  }
  return flags;
}

// ---- SDO server: the drive's objects ----

namespace {

namespace addr = cc::od::addr;

constexpr bool at(const cc::od::EntryInfo& entry, cc::od::Address a) {
  return entry.index == a.index && entry.sub == a.sub;
}

// Fragment fields and the controlword reach the drive by PDO only (D4); an SDO write of them is
// refused, so the command path has exactly one entrance.
constexpr bool pdo_only(const cc::od::EntryInfo& entry) {
  return entry.index == addr::controlword.index || entry.index == addr::target_position.index ||
         entry.index == addr::rpdo1_ext_command_seq.index ||
         entry.index == addr::rpdo2_ext_valid_until_us.index;
}

constexpr std::uint32_t raw_i32(std::int32_t v) { return static_cast<std::uint32_t>(v); }

}  // namespace

class Drive::Objects final : public ObjectAccess {
 public:
  Objects(Drive& drive, runtime_core::MonoTime now) : d_(drive), now_(now) {}

  ReadResult read(const cc::od::EntryInfo& e) override { return {value(e), 0}; }

  std::uint32_t write(const cc::od::EntryInfo& e, std::uint32_t v) override {
    if (e.index == addr::exec_request_attempt_id.index) {
      return d_.write_exec_request(e.sub, v, now_);
    }
    if (at(e, addr::exec_commit)) {
      return d_.write_exec_commit(v, now_);
    }
    if (at(e, addr::policy_reset)) {
      // D11: 1 asks for the reset; it is refused until the latest stop is confirmed. The EDS
      // limits (0..1) already refused anything else; 0 asks for nothing.
      return v == 0U || d_.policy_reset(now_) ? 0U : sdo_abort::kDeviceState;
    }
    if (at(e, addr::modes_of_operation)) {
      // The EDS limits (8..8) already refused every other mode.
      return d_.cia402_.request_mode(static_cast<std::int8_t>(v)) ? 0U : sdo_abort::kRange;
    }
    if (pdo_only(e)) {
      return sdo_abort::kUnsupportedAccess;
    }
    // Every other writable object holds a value this drive does not change at run time (COB-IDs,
    // transmission types, heartbeat time, option codes, quick-stop deceleration): writing the
    // value it has is accepted, any other value is an invalid value for the parameter.
    return v == value(e) ? 0U : sdo_abort::kRange;
  }

 private:
  std::uint32_t value(const cc::od::EntryInfo& e) const {
    const Drive& d = d_;
    const std::optional<Command>& cmd = d.actuated_;
    if (e.index == addr::exec_request_attempt_id.index && e.sub != 0U) {
      return d.staging_.staged(e.sub);
    }
    if (at(e, addr::error_register)) {
      return d.cia402_.fault_code() ? 1U : 0U;  // Bit 0: generic error.
    }
    if (at(e, addr::controlword)) {
      return d.controlword_;
    }
    if (at(e, addr::statusword)) {
      return d.cia402_.statusword();
    }
    if (at(e, addr::modes_of_operation) || at(e, addr::modes_of_operation_display)) {
      return static_cast<std::uint8_t>(d.cia402_.mode_display());
    }
    if (at(e, addr::position_actual_value)) {
      return raw_i32(to_wire_micro(d.joint_.state().position_nrad));
    }
    if (at(e, addr::velocity_actual_value)) {
      return raw_i32(to_wire_micro(d.joint_.state().velocity_nrad_s));
    }
    if (at(e, addr::target_position)) {
      return cmd ? raw_i32(cmd->target_position) : 0U;
    }
    if (at(e, addr::rpdo1_ext_command_seq) || at(e, addr::rpdo2_ext_command_seq)) {
      return cmd ? cmd->command_seq : 0U;
    }
    if (at(e, addr::rpdo1_ext_exec_tag) || at(e, addr::rpdo2_ext_exec_tag)) {
      return cmd ? cmd->exec_tag : 0U;
    }
    if (at(e, addr::rpdo2_ext_valid_until_us)) {
      return cmd ? cmd->valid_until_us : 0U;
    }
    if (at(e, addr::exec_commit)) {
      return d.exec_commit_written_;
    }
    if (at(e, addr::exec_tag)) {
      return d.issuer_.current();
    }
    if (at(e, addr::boot_id)) {
      return d.issuer_.boot_id();
    }
    if (at(e, addr::policy_reset)) {
      return 0U;
    }
    if (at(e, addr::committed_attempt)) {
      return d.staging_.committed_attempt();
    }
    if (at(e, addr::feedback_ext_sample_seq)) {
      return d.sample_seq_;
    }
    if (at(e, addr::feedback_ext_applied_seq)) {
      return d.applied_seq_;
    }
    if (at(e, addr::feedback_ext_sample_time_us)) {
      return d.sample_time_us_;
    }
    if (at(e, addr::feedback_ext_applied_tag)) {
      return d.applied_tag_;
    }
    if (at(e, addr::feedback_ext_drive_flags)) {
      return d.drive_flags();
    }
    // Static objects: the EDS default, node-relative ones with this node-ID added.
    const std::int64_t base = e.default_value + (e.default_adds_node_id ? d.node_.value() : 0);
    return static_cast<std::uint32_t>(base);
  }

  Drive& d_;
  runtime_core::MonoTime now_;
};

std::vector<cc::Frame> Drive::on_sdo(const cc::Frame& frame, runtime_core::MonoTime now) {
  const std::optional<cc::Sdo> request = cc::decode_sdo(node_, frame);
  if (!request) {
    ++counts_.malformed;
    return {};
  }
  ++counts_.sdo;
  Objects objects(*this, now);
  const std::optional<cc::Sdo> response = SdoServer::handle(*request, objects);
  if (!response) {
    return {};
  }
  if (const std::optional<cc::Frame> out = cc::encode_sdo(node_, *response)) {
    return {*out};
  }
  return {};  // Unreachable: the server answers with the entry's own size.
}

std::uint32_t Drive::write_exec_request(std::uint8_t sub, std::uint32_t value,
                                        runtime_core::MonoTime now) {
  // 0x2010 is writable only outside Operation Enabled with an active command: a running command
  // stream is never re-staged under it.
  if (cia402_.state() == Cia402State::kOperationEnabled && actuated_) {
    return sdo_abort::kDeviceState;
  }
  const bool ok = sub == CommitStaging::kSubAttemptId ? staging_.begin(value, now)
                                                      : staging_.stage(sub, value, now);
  // D17: an attempt_id not greater than every committed one, or a staging write with no open
  // staging window, cannot be stored.
  return ok ? 0U : sdo_abort::kTransfer;
}

std::uint32_t Drive::write_exec_commit(std::uint32_t crc, runtime_core::MonoTime now) {
  exec_commit_written_ = crc;
  // D17 idempotent repeat, decided before any allocation (ExecTagIssuer::commit always allocates):
  // the value of the latest successful commit again, while its tag is still the current one,
  // answers success with that same tag. Staging in progress for a newer attempt is untouched.
  if (last_commit_ && crc == last_commit_->request.crc() &&
      issuer_.epoch() == last_commit_->epoch && issuer_.current() == last_commit_->tag) {
    return 0U;
  }
  const Cia402State state = cia402_.state();
  const bool state_ok =
      state == Cia402State::kSwitchedOn || (state == Cia402State::kOperationEnabled && !actuated_);
  if (!state_ok || policy_latched_) {
    staging_.clear();  // A refused commit leaves nothing staged behind.
    return sdo_abort::kDeviceState;
  }
  const std::optional<CommitRequest> request = staging_.check(crc, now);  // Clears on failure.
  if (!request) {
    return sdo_abort::kTransfer;
  }
  const ExecTagIssuer::CommitOutcome outcome =
      issuer_.commit_ex(request->generation(), request->session_lo, now);
  if (!outcome.tag) {
    staging_.clear();  // Boot quarantine or an exhausted tag counter (needs reset node).
    return sdo_abort::kDeviceState;
  }
  staging_.committed(*request);
  last_commit_ = LastCommit{*request, *outcome.tag, issuer_.epoch()};
  return 0U;
}

}  // namespace drive_core
