// SPDX-License-Identifier: Apache-2.0
#include "canopen_master/vdrive_session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace canopen_master {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;
using hal_contract::Fault;
using hal_contract::OpState;
using hal_contract::StopStatus;
using runtime_core::Duration;
using runtime_core::MonoTime;

namespace {

constexpr std::uint8_t bit(DriveState s) {
  return static_cast<std::uint8_t>(1U << static_cast<unsigned>(s));
}
// A stop the inhibit ordered ends in Quick Stop Active (controlword or the drive's own policy
// stop) or in Fault (after a fault reaction); a disable ends in Switched On.
constexpr std::uint8_t kAcceptInhibit = bit(DriveState::kQuickStopActive) | bit(DriveState::kFault);
constexpr std::uint8_t kAcceptDisable = bit(DriveState::kSwitchedOn);

constexpr std::uint16_t kIdxDeviceType = 0x1000;
constexpr std::uint16_t kIdxIdentity = 0x1018;
constexpr std::uint16_t kIdxExecRequest = 0x2010;
constexpr std::uint16_t kIdxExecCommit = 0x2011;
constexpr std::uint16_t kIdxExecTag = 0x2012;
constexpr std::uint16_t kIdxBootId = 0x2013;
constexpr std::uint16_t kIdxCommandTimeout = 0x2014;
constexpr std::uint16_t kIdxCommittedAttempt = 0x2016;
constexpr std::uint16_t kIdxFeedbackExt = 0x2030;
constexpr std::uint8_t kSubDriveFlags = 0x05;
constexpr std::uint16_t kIdxStatusword = 0x6041;
constexpr std::uint16_t kIdxModeDisplay = 0x6061;
constexpr std::uint16_t kIdxVelocity = 0x606C;
constexpr std::uint32_t kCspMode = 8;
constexpr std::uint32_t kCommandTimeoutMs = 30;

// The EDS default of an object; 0 if absent.
std::uint32_t eds_default(std::uint16_t index, std::uint8_t sub) {
  for (const od::EntryInfo& e : od::kEntries) {
    if (e.index == index && e.sub == sub) {
      return static_cast<std::uint32_t>(e.default_value);
    }
  }
  return 0;
}

// floor(t / 1 us) in ns: the instant the wire value of D5 stands for.
std::int64_t floor_us_ns(MonoTime t) {
  std::int64_t us = t.ns / 1000;
  if (t.ns % 1000 < 0) {
    --us;
  }
  return us * 1000;
}

std::uint32_t wire_us(MonoTime t) {
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(floor_us_ns(t) / 1000));
}

std::int32_t to_urad(double rad) {
  const double urad = std::round(rad * 1e6);
  constexpr double lo = std::numeric_limits<std::int32_t>::min();
  constexpr double hi = std::numeric_limits<std::int32_t>::max();
  if (!(urad >= lo)) {
    return std::numeric_limits<std::int32_t>::min();
  }
  if (urad > hi) {
    return std::numeric_limits<std::int32_t>::max();
  }
  return static_cast<std::int32_t>(urad);
}

// CRC-32/ISO-HDLC over the little-endian bytes of the four staged values (D17). Written here
// from the contract, not taken from drive_core: the host side computes it independently.
std::uint32_t commit_crc(std::uint32_t attempt, std::uint32_t gen_lo, std::uint32_t gen_hi,
                         std::uint32_t session_lo) {
  std::uint32_t crc = 0xFFFF'FFFFU;
  for (const std::uint32_t word : {attempt, gen_lo, gen_hi, session_lo}) {
    for (unsigned b = 0; b < 4; ++b) {
      crc ^= (word >> (8U * b)) & 0xFFU;
      for (int k = 0; k < 8; ++k) {
        crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB8'8320U : crc >> 1U;
      }
    }
  }
  return crc ^ 0xFFFF'FFFFU;
}

std::array<std::uint8_t, 4> le32(std::uint32_t v) {
  return {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8U),
          static_cast<std::uint8_t>(v >> 16U), static_cast<std::uint8_t>(v >> 24U)};
}

template <class Pdo>
PdoLayout layout_of() {
  PdoLayout layout;
  for (const od::Field& f : Pdo::kFields) {
    layout.fields.push_back(
        PdoField{f.index, f.sub, static_cast<std::uint8_t>(f.size * 8U), f.is_signed});
  }
  return layout;
}

template <class Pdo>
std::uint16_t cob_of(cc::NodeId node) {
  return Pdo::kCobIdAddsNodeId ? static_cast<std::uint16_t>(Pdo::kCobIdBase + node.value())
                               : Pdo::kCobIdBase;
}

bool at_rest(std::int32_t velocity_urad_s) {
  const std::int64_t v = velocity_urad_s;
  return v < kRestVelocityUradS && v > -kRestVelocityUradS;
}

cc::NmtState state_after(cc::NmtCommand command) {
  switch (command) {
    case cc::NmtCommand::kStart:
      return cc::NmtState::kOperational;
    case cc::NmtCommand::kStop:
      return cc::NmtState::kStopped;
    case cc::NmtCommand::kEnterPreOperational:
    case cc::NmtCommand::kResetNode:
    case cc::NmtCommand::kResetCommunication:
      break;
  }
  return cc::NmtState::kPreOperational;
}

}  // namespace

DriveState decode_statusword(std::uint16_t sw) {
  const unsigned low4f = sw & 0x4FU;
  const unsigned low6f = sw & 0x6FU;
  if (low4f == 0x00U) {
    return DriveState::kNotReadyToSwitchOn;
  }
  if (low4f == 0x40U) {
    return DriveState::kSwitchOnDisabled;
  }
  if (low6f == 0x21U) {
    return DriveState::kReadyToSwitchOn;
  }
  if (low6f == 0x23U) {
    return DriveState::kSwitchedOn;
  }
  if (low6f == 0x27U) {
    return DriveState::kOperationEnabled;
  }
  if (low6f == 0x07U) {
    return DriveState::kQuickStopActive;
  }
  if (low4f == 0x0FU) {
    return DriveState::kFaultReactionActive;
  }
  // 0x08, and any pattern CiA 402 does not define: treated as Fault, never as a usable state.
  return DriveState::kFault;
}

Identity eds_identity() {
  return Identity{eds_default(kIdxIdentity, 1), eds_default(kIdxIdentity, 2),
                  eds_default(kIdxIdentity, 3)};
}

// ---- construction and calls ----

VdriveSession::VdriveSession(Master& master, SessionConfig config)
    : master_(master), config_(config), join_(Duration{config.bus_period.ns + kDriveTick.ns}) {
  if (config_.bus_period != kBusPeriod || config_.lease.ns <= 0 || config_.lease > kMaxLease ||
      config_.max_feedback_age < kMinFeedbackAge || config_.max_feedback_age > kMaxFeedbackAge) {
    throw std::invalid_argument(
        "vdrive_session: unsupported timing profile; allowed: bus_period_ms = 10, lease_ms in "
        "(0, 48], max_feedback_age_ms in [20, 30] (C20)");
  }
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      if (config_.nodes[i] == config_.nodes[j]) {
        throw std::invalid_argument("vdrive_session: duplicate node");
      }
    }
    const cc::NodeId id = config_.nodes[i];
    if (!master_.register_rx_pdo(id, cob_of<od::Tpdo1>(id), layout_of<od::Tpdo1>()) ||
        !master_.register_rx_pdo(id, cob_of<od::Tpdo2>(id), layout_of<od::Tpdo2>()) ||
        !master_.register_rx_pdo(id, cob_of<od::Tpdo3>(id), layout_of<od::Tpdo3>())) {
      throw std::invalid_argument("vdrive_session: a node the master does not know");
    }
  }
  extra_.reserve(64);
}

std::vector<cc::Frame> VdriveSession::on_frame(const cc::Frame& frame, MonoTime now) {
  now_ = now;
  before(now);
  return after(master_.on_frame(frame, now), now);
}

std::vector<cc::Frame> VdriveSession::tick(MonoTime now) {
  now_ = now;
  before(now);
  return after(master_.tick(now), now);
}

void VdriveSession::before(MonoTime now) {
  if (op_ && now >= op_->deadline) {
    fail("the op deadline passed", now);
  }
  if (posted_) {
    const Op op = *posted_;
    posted_.reset();
    start(op, now);
  }
  if (op_is(OpKind::kArm)) {
    if (stage_ == ArmStage::kCommit) {
      for (std::size_t i = 0; i < kSessionNodes && op_is(OpKind::kArm); ++i) {
        if (nodes_[i].commit == CommitStep::kWaitRetry && now >= nodes_[i].retry_at) {
          commit(i);
        }
      }
    } else if (stage_ == ArmStage::kConfirm && now >= enable_at_ + kArmConfirmWindow) {
      profile_dropped_ = true;  // C2: the profile drops to generic.
      fail("a node did not report the armed tag and a stream seq within 4 x T_bus", now);
    }
  }
  if (phase_ == SessionPhase::kArmed && !inhibited_) {
    const std::optional<hal_contract::Feedback> group = join_.latest();
    if (!group || now - group->sampled_at > config_.max_feedback_age) {
      latch(std::nullopt, now);  // C17: feedback age while armed.
    }
  }
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    StopTrack& t = nodes_[i].stop;
    if (t.status != StopStatus::kStopping) {
      continue;
    }
    if (now >= t.started + kStopConfirmDeadline) {
      resolve(i, StopStatus::kUnconfirmed);
    } else if (t.mode == StopMode::kSdoPoll) {
      poll(i, now);
    }
  }
}

std::vector<cc::Frame> VdriveSession::after(std::vector<cc::Frame> frames, MonoTime now) {
  for (const MasterEvent& e : master_.drain_events()) {
    handle(e, now);
  }
  frames.insert(frames.end(), extra_.begin(), extra_.end());
  extra_.clear();
  return frames;
}

std::optional<std::size_t> VdriveSession::index_of(std::uint8_t node) const {
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    if (config_.nodes[i].value() == node) {
      return i;
    }
  }
  return std::nullopt;
}

void VdriveSession::handle(const MasterEvent& e, MonoTime now) {
  if (e.kind == MasterEventKind::kSync) {
    last_sync_ = e.at;
    slot(now);
    return;
  }
  const std::optional<std::size_t> idx = index_of(e.node);
  if (!idx) {
    return;
  }
  const std::size_t i = *idx;
  NodeState& n = nodes_[i];
  switch (e.kind) {
    case MasterEventKind::kPdo:
      if (const auto* pdo = std::get_if<PdoDetail>(&e.detail)) {
        handle_pdo(i, *pdo, now);
      }
      return;
    case MasterEventKind::kBoot:
      join_.on_boot(i);
      if (n.awaiting_boot) {
        n.awaiting_boot = false;  // The boot-up our Reset Communication asked for (C9).
        reread(i);
        return;
      }
      n.boot_at = now;  // A new boot: its quarantine counts from here (D3).
      n.commanded = cc::NmtState::kPreOperational;
      n.previous.reset();
      latch(std::nullopt, now);
      return;
    case MasterEventKind::kState: {
      const auto* d = std::get_if<StateDetail>(&e.detail);
      if (phase_ == SessionPhase::kIdle || d == nullptr || !n.commanded ||
          d->state == *n.commanded) {
        return;
      }
      // A heartbeat sent before our command but handled after it (one worker period of grace).
      if (n.previous && d->state == *n.previous &&
          now < n.commanded_at + Duration{2 * config_.bus_period.ns}) {
        return;
      }
      latch(std::nullopt, now);
      return;
    }
    case MasterEventKind::kBootTimeout:
      latch(Fault::kBootTimeout, now);
      return;
    case MasterEventKind::kHeartbeatLost:
      latch(Fault::kNodeLost, now);
      return;
    case MasterEventKind::kEmcy:
      if (const auto* d = std::get_if<EmcyDetail>(&e.detail); d != nullptr && d->error_code != 0) {
        if (d->error_code == kEmcyTorqueLoss) {
          n.torque_loss = true;  // C18: the node's stop is unconfirmed whatever its velocity.
        }
        latch(Fault::kDriveFault, now);
        if (n.torque_loss && n.stop.status == StopStatus::kStopping) {
          resolve(i, StopStatus::kUnconfirmed);
        }
      }
      return;
    default:
      return;
  }
}

void VdriveSession::handle_pdo(std::size_t i, const PdoDetail& pdo, MonoTime now) {
  if (pdo.fields.size() != 3) {
    return;
  }
  const cc::NodeId id = config_.nodes[i];
  const auto u16 = [&](std::size_t k) { return static_cast<std::uint16_t>(pdo.fields[k].raw); };
  const auto s32 = [&](std::size_t k) {
    return static_cast<std::int32_t>(pdo.fields[k].signed_value());
  };
  std::optional<NodeSample> done;
  if (pdo.cob == cob_of<od::Tpdo1>(id)) {
    done = join_.on_tpdo(i, od::Tpdo1{u16(0), s32(1), u16(2)}, now);
  } else if (pdo.cob == cob_of<od::Tpdo2>(id)) {
    done = join_.on_tpdo(i, od::Tpdo2{s32(0), u16(1), u16(2)}, now);
  } else if (pdo.cob == cob_of<od::Tpdo3>(id)) {
    done = join_.on_tpdo(
        i, od::Tpdo3{static_cast<std::uint32_t>(pdo.fields[0].raw), u16(1), u16(2)}, now);
  }
  if (done) {
    on_sample(i, *done, now);
  }
}

void VdriveSession::on_sample(std::size_t i, const NodeSample& s, MonoTime now) {
  if (phase_ == SessionPhase::kIdle) {
    return;
  }
  NodeState& n = nodes_[i];
  const DriveState state = decode_statusword(s.statusword);
  check_tag(i, s, now);
  if (!inhibited_) {
    check_state(i, state, now);
  }
  stop_on_sample(i, s);
  if (op_is(OpKind::kActivate)) {
    if (s.sampled_at >= op_started_ && state == DriveState::kSwitchedOn) {
      n.activated = true;
    }
    if (std::all_of(nodes_.begin(), nodes_.end(), [](const NodeState& x) { return x.activated; })) {
      phase_ = SessionPhase::kActive;
      succeed();
    }
  } else if (op_is(OpKind::kArm) && stage_ == ArmStage::kConfirm) {
    const SeqEntry* e = lookup(s.applied_seq);
    if (s.applied_tag == n.stream_tag && state == DriveState::kOperationEnabled && e != nullptr &&
        !e->approved) {
      n.confirmed = true;
    }
    if (std::all_of(nodes_.begin(), nodes_.end(), [](const NodeState& x) { return x.confirmed; })) {
      armed_id_ = arm_request_.id;
      phase_ = SessionPhase::kArmed;
      profile_confirmed_ =
          !profile_dropped_ && std::all_of(nodes_.begin(), nodes_.end(), [](const NodeState& x) {
            return x.profile == NodeProfile::kVirtual;
          });
      succeed();
    }
  }
}

void VdriveSession::check_tag(std::size_t i, const NodeSample& s, MonoTime now) {
  NodeState& n = nodes_[i];
  const std::uint16_t tag = s.applied_tag;
  if (n.pending_tag && tag == *n.pending_tag) {
    n.armed_tag = tag;  // C17: the first applied_tag equal to the pending tag arms it.
    n.pending_tag.reset();
    return;
  }
  if ((n.armed_tag && tag == *n.armed_tag) || (!n.armed_tag && tag == 0)) {
    return;
  }
  latch(std::nullopt, now);  // Neither the pending nor the armed tag of this node.
}

void VdriveSession::check_state(std::size_t i, DriveState state, MonoTime now) {
  switch (state) {
    case DriveState::kQuickStopActive:
      latch(Fault::kPolicyStop, now);
      return;
    case DriveState::kFaultReactionActive:
    case DriveState::kFault:
      latch(Fault::kDriveFault, now);
      return;
    case DriveState::kOperationEnabled: {
      const bool arming =
          op_is(OpKind::kArm) && (nodes_[i].enable_sent || stage_ == ArmStage::kDisable);
      if (phase_ != SessionPhase::kArmed && phase_ != SessionPhase::kDeactivating && !arming) {
        latch(std::nullopt, now);
      }
      return;
    }
    default:
      if (phase_ == SessionPhase::kArmed) {
        latch(std::nullopt, now);  // An armed node left Operation Enabled on its own.
      }
      return;
  }
}

void VdriveSession::command_nmt(std::size_t i, cc::NmtCommand command, MonoTime now) {
  NodeState& n = nodes_[i];
  master_.nmt(command, config_.nodes[i]);
  n.previous = n.commanded;
  n.commanded = state_after(command);
  n.commanded_at = now;
}

void VdriveSession::rpdo3(std::size_t i, std::uint16_t cw) {
  extra_.push_back(cc::pack_rpdo3(config_.nodes[i], od::Rpdo3{cw}));
}

// ---- ops ----

void VdriveSession::record(std::uint64_t id, OpState state, const char* error) {
  for (OpRecord& r : ops_) {
    if (r.id == id) {
      r.state = state;
      r.error = error;
      return;
    }
  }
  ops_[ops_next_] = OpRecord{id, state, error};
  ops_next_ = (ops_next_ + 1) % kOpRing;
}

OpState VdriveSession::op_state(std::uint64_t op_id) const {
  for (const OpRecord& r : ops_) {
    if (r.id == op_id && op_id != 0) {
      return r.state;
    }
  }
  return OpState::kUnknown;
}

const char* VdriveSession::op_error(std::uint64_t op_id) const {
  for (const OpRecord& r : ops_) {
    if (r.id == op_id && op_id != 0) {
      return r.error;
    }
  }
  return nullptr;
}

void VdriveSession::post(OpKind kind, const hal_contract::OpTicket& ticket,
                         const hal_contract::ArmRequest& request) {
  if (posted_) {
    record(posted_->id, OpState::kFailed, "superseded by a newer op before it started");
  }
  posted_ = Op{ticket.op_id, kind, ticket.deadline, request};
  record(ticket.op_id, OpState::kPending, nullptr);
}

void VdriveSession::configure(const hal_contract::OpTicket& ticket) {
  post(OpKind::kConfigure, ticket);
}
void VdriveSession::activate(const hal_contract::OpTicket& ticket) {
  post(OpKind::kActivate, ticket);
}
void VdriveSession::arm(const hal_contract::OpTicket& ticket,
                        const hal_contract::ArmRequest& request) {
  post(OpKind::kArm, ticket, request);
}
void VdriveSession::deactivate(const hal_contract::OpTicket& ticket) {
  post(OpKind::kDeactivate, ticket);
}

void VdriveSession::end_op(OpState state, const char* error) {
  if (!op_) {
    return;
  }
  record(op_->id, state, error);
  op_.reset();
  ++epoch_;  // Every SDO completion and frame of the ended op is ignored from here on.
}

void VdriveSession::succeed() { end_op(OpState::kSucceeded, nullptr); }

void VdriveSession::fail(const char* why, MonoTime now) {
  if (!op_) {
    return;
  }
  const bool arm = end_failed(why);
  latch(std::nullopt, now, arm);  // C17: a failed or timed-out op.
}

bool VdriveSession::end_failed(const char* why) {
  if (!op_) {
    return false;
  }
  const OpKind kind = op_->kind;
  end_op(OpState::kFailed, why);
  switch (kind) {
    case OpKind::kConfigure:
      phase_ = SessionPhase::kConfigured;
      break;
    case OpKind::kArm:
      faults_.set(Fault::kArmFailed);
      phase_ = SessionPhase::kActive;
      break;
    case OpKind::kActivate:
    case OpKind::kDeactivate:
      phase_ = SessionPhase::kActive;
      break;
  }
  return kind == OpKind::kArm;
}

void VdriveSession::start(const Op& op, MonoTime now) {
  if (op_) {
    fail("superseded by a newer op", now);
  }
  op_ = op;
  op_started_ = now;
  ++epoch_;
  if (now >= op.deadline) {
    fail("the op deadline passed before it started", now);
    return;
  }
  switch (op.kind) {
    case OpKind::kConfigure:
      start_configure(now);
      return;
    case OpKind::kActivate:
      start_activate(now);
      return;
    case OpKind::kArm:
      start_arm(now);
      return;
    case OpKind::kDeactivate:
      start_deactivate(now);
      return;
  }
}

// ---- configure (C2, C9) ----

void VdriveSession::start_configure(MonoTime /*now*/) {
  inhibited_ = false;  // C17: only a new configure clears the inhibit.
  faults_ = {};
  profile_confirmed_ = false;
  profile_dropped_ = false;
  armed_id_.reset();
  stop_streaming();
  join_.reset();
  phase_ = SessionPhase::kConfiguring;
  for (NodeState& n : nodes_) {
    NodeState fresh;
    fresh.boot_at = n.boot_at;
    fresh.stop.gen = n.stop.gen + 1;
    n = fresh;
  }
  probe_left_ = static_cast<int>(kSessionNodes);
  for (std::size_t i = 0; i < kSessionNodes && op_is(OpKind::kConfigure); ++i) {
    probe(i, 0);
  }
}

// One request at a time per node: the next leaves only from the previous one's completion and
// only while the op's epoch is current, so an ended op sends nothing more (C10).
void VdriveSession::probe(std::size_t i, std::size_t k) {
  Probe& p = nodes_[i].probe;
  struct Item {
    std::uint16_t index;
    std::uint8_t sub;
    std::optional<std::uint32_t>* into;
  };
  const std::array<Item, 10> items{{
      {kIdxDeviceType, 0, &p.device_type},
      {kIdxIdentity, 1, &p.identity[0]},
      {kIdxIdentity, 2, &p.identity[1]},
      {kIdxIdentity, 3, &p.identity[2]},
      {kIdxIdentity, 4, &p.identity[3]},
      {kIdxBootId, 0, &p.boot_id},
      {kIdxCommandTimeout, 0, &p.command_timeout_ms},
      {kIdxCommittedAttempt, 0, &p.committed_attempt},
      {kIdxModeDisplay, 0, &p.mode_display},
      {kIdxExecTag, 0, &p.exec_tag},
  }};
  if (k == items.size()) {
    if (--probe_left_ == 0) {
      probe_done(now_);
    }
    return;
  }
  const std::uint64_t epoch = epoch_;
  std::optional<std::uint32_t>* into = items[k].into;
  if (!master_.sdo_upload(config_.nodes[i], items[k].index, items[k].sub,
                          [this, epoch, into, i, k](const SdoResult& r) {
                            if (epoch != epoch_) {
                              return;
                            }
                            if (r.status == SdoStatus::kOk) {
                              *into = r.value;  // A failed read stays empty: not a vdrive.
                            }
                            probe(i, k + 1);
                          })) {
    fail("a probe request was refused", now_);
  }
}

void VdriveSession::probe_done(MonoTime now) {
  const Identity eds = eds_identity();
  const std::uint32_t eds_type = eds_default(kIdxDeviceType, 0);
  reset_left_ = 0;
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    NodeState& n = nodes_[i];
    const Probe& p = n.probe;
    const bool identity = p.identity[3].has_value() && p.identity[0] == eds.vendor &&
                          p.identity[1] == eds.product && p.identity[2] == eds.revision;
    const bool vdrive = p.device_type == eds_type && identity &&
                        p.command_timeout_ms == kCommandTimeoutMs && p.mode_display == kCspMode &&
                        p.boot_id && p.committed_attempt && p.exec_tag;
    n.profile = vdrive ? NodeProfile::kVirtual : NodeProfile::kGeneric;
    n.boot_id = p.boot_id;
    n.committed_attempt = p.committed_attempt;
    if (vdrive) {
      // C9: only vdrive nodes are reset; a generic node gets nothing after the probe.
      n.awaiting_boot = true;
      command_nmt(i, cc::NmtCommand::kResetCommunication, now);
      ++reset_left_;
    }
  }
  if (reset_left_ == 0) {
    phase_ = SessionPhase::kConfigured;
    succeed();
  }
}

void VdriveSession::reread(std::size_t i) {
  if (!op_is(OpKind::kConfigure)) {
    return;
  }
  const std::uint64_t epoch = epoch_;
  const auto ok = [this, epoch](const SdoResult& r) {
    if (epoch != epoch_) {
      return false;
    }
    if (r.status != SdoStatus::kOk) {
      fail("a re-read after Reset Communication failed", now_);
      return false;
    }
    return true;
  };
  const cc::NodeId id = config_.nodes[i];
  if (!master_.sdo_upload(id, kIdxBootId, 0, [this, ok, i, id](const SdoResult& r) {
        if (!ok(r)) {
          return;
        }
        nodes_[i].boot_id = r.value;
        if (!master_.sdo_upload(id, kIdxCommittedAttempt, 0, [this, ok, i](const SdoResult& r2) {
              if (!ok(r2)) {
                return;
              }
              nodes_[i].committed_attempt = r2.value;
              if (--reset_left_ == 0) {
                phase_ = SessionPhase::kConfigured;
                succeed();
              }
            })) {
          fail("a re-read was refused", now_);
        }
      })) {
    fail("a re-read was refused", now_);
  }
}

// ---- activate (C9) ----

void VdriveSession::start_activate(MonoTime now) {
  if (inhibited_) {
    fail("the inhibit is latched", now);
    return;
  }
  if (phase_ != SessionPhase::kConfigured) {
    fail("not configured", now);
    return;
  }
  phase_ = SessionPhase::kActivating;
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    NodeState& n = nodes_[i];
    n.activated = false;
    const std::uint64_t gen = n.stop.gen + 1;  // Answers for the old track are ignored.
    n.stop = StopTrack{};
    n.stop.gen = gen;
    command_nmt(i, cc::NmtCommand::kStart, now);
    // The RPDO3s leave after the master's NMT frames of this call.
    rpdo3(i, controlword::kShutdown);
    rpdo3(i, controlword::kSwitchOn);
  }
}

// ---- arm (C9) ----

void VdriveSession::start_arm(MonoTime now) {
  if (inhibited_) {
    fail("the inhibit is latched", now);
    return;
  }
  if (phase_ != SessionPhase::kActive && phase_ != SessionPhase::kArmed) {
    fail("not active", now);
    return;
  }
  arm_request_ = op_->request;
  if (!(arm_request_.hold_until > now)) {
    fail("hold_until is not in the future", now);
    return;
  }
  // Phase 1: normal submissions are blocked and the command box is cleared.
  phase_ = SessionPhase::kArming;
  armed_id_.reset();
  stop_streaming();
  hold_until_ = arm_request_.hold_until;
  hold_updates_ = 0;
  for (NodeState& n : nodes_) {
    n.enable_sent = false;
    n.confirmed = false;
    n.commit = CommitStep::kIdle;
    n.retried = false;
    n.pending_tag.reset();
    n.fenced = false;
    const std::uint64_t gen = n.stop.gen + 1;  // Answers for the old track are ignored.
    n.stop = StopTrack{};
    n.stop.gen = gen;
  }
  // Phase 2: an enabled node is disabled and its stop confirmed before anything is committed.
  stage_ = ArmStage::kDisable;
  bool any_enabled = false;
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    const std::optional<NodeSample>& s = join_.node(i);
    if (s && decode_statusword(s->statusword) == DriveState::kOperationEnabled) {
      any_enabled = true;
    }
  }
  if (any_enabled) {
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      rpdo3(i, controlword::kSwitchOn);
      const std::optional<NodeSample>& s = join_.node(i);
      if (s && decode_statusword(s->statusword) == DriveState::kOperationEnabled) {
        begin_stop(i, kAcceptDisable, now);
      }
    }
    return;
  }
  arm_progress(now);
}

void VdriveSession::arm_progress(MonoTime now) {
  if (!op_is(OpKind::kArm) || stage_ != ArmStage::kDisable || !stops_resolved()) {
    return;
  }
  for (const NodeState& n : nodes_) {
    if (n.stop.status == StopStatus::kUnconfirmed) {
      fail("a disable before the commit was not confirmed", now);
      return;
    }
  }
  // The disable stops belong to the old arm: from here the nodes will be enabled again, and a
  // later inhibit must stop and track every one of them afresh (C17, C18).
  for (NodeState& n : nodes_) {
    const std::uint64_t gen = n.stop.gen + 1;
    n.stop = StopTrack{};
    n.stop.gen = gen;
  }
  // Phase 3: commit on all.
  stage_ = ArmStage::kCommit;
  for (std::size_t i = 0; i < kSessionNodes && op_is(OpKind::kArm); ++i) {
    commit(i);
  }
}

void VdriveSession::commit(std::size_t i) {
  NodeState& n = nodes_[i];
  if (!n.committed_attempt) {
    fail("the committed attempt of a node is unknown", now_);
    return;
  }
  if (*n.committed_attempt == std::numeric_limits<std::uint32_t>::max()) {
    fail("attempt_id would wrap", now_);
    return;
  }
  n.attempt = *n.committed_attempt + 1U;  // D17: strictly greater than the observed 0x2016.
  n.commit = CommitStep::kWriting;
  commit_write(i, 0);
}

// D17: 0x2010:04 attempt_id, 0x2010:01-03, then 0x2011 = CRC-32 over the four; one at a time.
void VdriveSession::commit_write(std::size_t i, std::size_t k) {
  NodeState& n = nodes_[i];
  const hal_contract::AuthorityId& id = arm_request_.id;
  const auto gen_lo = static_cast<std::uint32_t>(id.generation.value);
  const auto gen_hi = static_cast<std::uint32_t>(id.generation.value >> 32U);
  const auto session_lo = static_cast<std::uint32_t>(id.session.value);
  struct Write {
    std::uint16_t index;
    std::uint8_t sub;
    std::uint32_t value;
  };
  const std::array<Write, 5> writes{{
      {kIdxExecRequest, 4, n.attempt},
      {kIdxExecRequest, 1, gen_lo},
      {kIdxExecRequest, 2, gen_hi},
      {kIdxExecRequest, 3, session_lo},
      {kIdxExecCommit, 0, commit_crc(n.attempt, gen_lo, gen_hi, session_lo)},
  }};
  const std::uint64_t epoch = epoch_;
  const bool last = k + 1 == writes.size();
  const std::array<std::uint8_t, 4> bytes = le32(writes[k].value);
  if (!master_.sdo_download(config_.nodes[i], writes[k].index, writes[k].sub, bytes,
                            [this, epoch, i, k, last](const SdoResult& r) {
                              if (epoch != epoch_) {
                                return;
                              }
                              if (r.status != SdoStatus::kOk) {
                                commit_failed(i, last, now_);
                              } else if (last) {
                                commit_read(i);
                              } else {
                                commit_write(i, k + 1);
                              }
                            })) {
    fail("a commit request was refused", now_);
  }
}

void VdriveSession::commit_failed(std::size_t i, bool on_commit_write, MonoTime now) {
  NodeState& n = nodes_[i];
  if (!on_commit_write || n.retried) {
    fail("a node refused the commit", now);
    return;
  }
  // One retry at boot-up + 100 ms + T_bus: the refusal may be the boot quarantine (D3).
  n.retried = true;
  ++n.retries;
  ++counts_.quarantine_retries;
  n.commit = CommitStep::kWaitRetry;
  n.retry_at = n.boot_at ? std::max(now, *n.boot_at + kBootQuarantine + config_.bus_period)
                         : now + kBootQuarantine + config_.bus_period;
}

void VdriveSession::commit_read(std::size_t i) {
  NodeState& n = nodes_[i];
  n.commit = CommitStep::kReading;
  const std::uint64_t epoch = epoch_;
  const auto ok = [this, epoch](const SdoResult& r) {
    if (epoch != epoch_) {
      return false;
    }
    if (r.status != SdoStatus::kOk) {
      fail("reading back the commit failed", now_);
      return false;
    }
    return true;
  };
  const cc::NodeId id = config_.nodes[i];
  if (!master_.sdo_upload(id, kIdxCommittedAttempt, 0, [this, ok, i, id](const SdoResult& r) {
        if (!ok(r)) {
          return;
        }
        if (r.value != nodes_[i].attempt) {
          fail("0x2016 does not show the commit", now_);
          return;
        }
        if (!master_.sdo_upload(id, kIdxExecTag, 0, [this, ok, i](const SdoResult& r2) {
              if (!ok(r2)) {
                return;
              }
              const auto tag = static_cast<std::uint16_t>(r2.value);
              if (tag == 0) {
                fail("0x2012 shows no tag after the commit", now_);
                return;
              }
              NodeState& node = nodes_[i];
              node.committed_attempt = node.attempt;
              node.pending_tag = tag;
              node.stream_tag = tag;
              node.commit = CommitStep::kDone;
              commits_progress(now_);
            })) {
          fail("a commit read-back was refused", now_);
        }
      })) {
    fail("a commit read-back was refused", now_);
  }
}

void VdriveSession::commits_progress(MonoTime now) {
  if (!op_is(OpKind::kArm) || !std::all_of(nodes_.begin(), nodes_.end(), [](const NodeState& n) {
        return n.commit == CommitStep::kDone;
      })) {
    return;
  }
  // Phase 4: hold each node's actual position under the request's own bounded authority.
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    const std::optional<NodeSample>& s = join_.node(i);
    if (!s) {
      fail("no feedback to hold", now);
      return;
    }
    hold_urad_[i] = s->position_urad;
  }
  holding_ = true;
  stage_ = ArmStage::kHold;
}

// ---- deactivate (C9) ----

void VdriveSession::start_deactivate(MonoTime now) {
  if (phase_ == SessionPhase::kConfigured || phase_ == SessionPhase::kConfiguring) {
    succeed();  // Nothing was started.
    return;
  }
  armed_id_.reset();
  stop_streaming();
  phase_ = SessionPhase::kDeactivating;
  if (!inhibited_) {
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      rpdo3(i, controlword::kSwitchOn);  // Disable Operation; Operational and SYNC stay.
      const std::optional<NodeSample>& s = join_.node(i);
      if (s && decode_statusword(s->statusword) == DriveState::kOperationEnabled) {
        begin_stop(i, kAcceptDisable, now);
      } else if (nodes_[i].stop.status == StopStatus::kNone) {
        nodes_[i].stop.status = StopStatus::kNotEnabled;
      }
    }
  }
  deactivate_progress(now);
}

void VdriveSession::deactivate_progress(MonoTime now) {
  if (!op_is(OpKind::kDeactivate) || !stops_resolved()) {
    return;
  }
  for (const NodeState& n : nodes_) {
    if (n.stop.status == StopStatus::kUnconfirmed) {
      fail("a stop was not confirmed; the nodes stay Operational", now);
      return;
    }
  }
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    command_nmt(i, cc::NmtCommand::kEnterPreOperational, now);
  }
  phase_ = SessionPhase::kConfigured;
  succeed();
}

// ---- command stream (C6) ----

void VdriveSession::offer(const hal_contract::ApprovedCommand& command) {
  if (phase_ != SessionPhase::kArmed || inhibited_ || !armed_id_ || !(command.id == *armed_id_)) {
    ++counts_.discarded;
    return;
  }
  if (box_) {
    ++counters_.superseded;  // Overwritten before a slot: never sent, never acknowledged.
  }
  box_ = command;
}

bool VdriveSession::open_at_slot(MonoTime valid_until, MonoTime slot) const {
  // The drive actuates the RPDOs of this slot at the next SYNC and checks them again at its next
  // W tick; 1 ms covers delivery.
  return floor_us_ns(valid_until) >
         (slot + config_.bus_period + kDriveTick + kDeliveryAllowance).ns;
}

bool VdriveSession::can_apply(MonoTime valid_until, MonoTime now) const {
  MonoTime next_slot = now;
  if (last_sync_) {
    next_slot = std::max(now, *last_sync_ + config_.bus_period);
  }
  return open_at_slot(valid_until, next_slot);
}

void VdriveSession::stop_streaming() {
  box_.reset();
  current_.reset();
  holding_ = false;
}

void VdriveSession::remember(const Stream& s) {
  seqs_[seqs_next_] = SeqEntry{true, s.seq, s.approved, s.source_seq};
  seqs_next_ = (seqs_next_ + 1) % kSeqRing;
}

const VdriveSession::SeqEntry* VdriveSession::lookup(std::uint16_t seq) const {
  for (std::size_t k = 0; k < kSeqRing; ++k) {
    const SeqEntry& e = seqs_[(seqs_next_ + kSeqRing - 1 - k) % kSeqRing];
    if (e.used && e.seq == seq) {
      return &e;
    }
  }
  return nullptr;
}

void VdriveSession::send_stream(Stream& s) {
  const std::uint32_t until = wire_us(s.valid_until);
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    const NodeState& n = nodes_[i];
    extra_.push_back(
        cc::pack_rpdo1(config_.nodes[i], od::Rpdo1{s.target_urad[i], s.seq, n.stream_tag}));
    extra_.push_back(cc::pack_rpdo2(config_.nodes[i], od::Rpdo2{until, s.seq, n.stream_tag}));
  }
}

void VdriveSession::slot(MonoTime now) {
  if (inhibited_) {
    return;
  }
  const bool arming =
      op_is(OpKind::kArm) && (stage_ == ArmStage::kHold || stage_ == ArmStage::kConfirm);
  if (phase_ != SessionPhase::kArmed && !arming) {
    return;
  }
  if (box_ && !(phase_ == SessionPhase::kArmed && armed_id_ && box_->id == *armed_id_)) {
    box_.reset();  // Defence in depth: never dispatch under an identity that is not armed.
  }
  if (box_) {
    Stream s;
    s.approved = true;
    s.source_seq = box_->source_seq;
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      s.target_urad[i] = to_urad(box_->position[i]);
    }
    s.valid_until = box_->valid_until;
    current_ = s;
    box_.reset();
    holding_ = false;  // The hold ends with the first approved command.
  }
  if (current_) {
    if (!open_at_slot(current_->valid_until, now)) {
      if (!current_->dispatched) {
        ++counters_.expired_before_dispatch;
      }
      current_.reset();  // Never re-sent with a new deadline: that needs a new approval.
      return;
    }
    if (!current_->dispatched) {
      current_->seq = next_seq_++;  // C20: the wire seq is allocated at dispatch.
      current_->dispatched = true;
      remember(*current_);
    }
    send_stream(*current_);  // A repeat keeps seq, payload and deadline.
    return;
  }
  if (!holding_) {
    return;
  }
  Stream h;
  h.target_urad = hold_urad_;
  h.valid_until = std::min(hold_until_, now + config_.lease);
  if (!open_at_slot(h.valid_until, now)) {
    holding_ = false;
    if (op_is(OpKind::kArm)) {
      fail("the hold ends before the enable is confirmed", now);
    }
    return;
  }
  h.seq = next_seq_++;
  h.dispatched = true;
  remember(h);
  send_stream(h);
  if (op_is(OpKind::kArm) && stage_ == ArmStage::kHold && ++hold_updates_ == 2) {
    // Phase 5: after the second stream update, enable all.
    for (std::size_t i = 0; i < kSessionNodes; ++i) {
      rpdo3(i, controlword::kEnableOperation);
      nodes_[i].enable_sent = true;
    }
    enable_at_ = now;
    stage_ = ArmStage::kConfirm;
  }
}

// ---- inhibit and stop confirmation (C17, C18, C10) ----

void VdriveSession::latch(std::optional<Fault> fault, MonoTime now, bool cancelled_arm) {
  if (phase_ == SessionPhase::kIdle) {
    return;
  }
  ++counts_.unexpected;
  if (fault) {
    faults_.set(*fault);
  }
  if (inhibited_) {
    return;
  }
  inhibited_ = true;
  faults_.set(Fault::kInhibited);
  if (op_) {
    cancelled_arm = end_failed("an unexpected event latched the inhibit") || cancelled_arm;
  }
  stop_streaming();
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    NodeState& n = nodes_[i];
    if (cancelled_arm && n.enable_sent) {
      // C10 fence: a delayed 0x0F cannot be recalled, so the node leaves Operational. A node in
      // Operation Enabled starts its own latched quick stop; in Pre-operational RPDO3 is ignored.
      n.fenced = true;
      command_nmt(i, cc::NmtCommand::kEnterPreOperational, now);
      begin_stop(i, kAcceptInhibit, now);
      n.stop.mode = StopMode::kSdoPoll;
      n.stop.next_round = now;
      continue;
    }
    const std::optional<NodeSample>& s = join_.node(i);
    const std::optional<DriveState> state =
        s ? std::optional<DriveState>{decode_statusword(s->statusword)} : std::nullopt;
    if (n.stop.status == StopStatus::kStopping) {
      n.stop.accept = static_cast<std::uint8_t>(n.stop.accept | kAcceptInhibit);
      if (state == DriveState::kOperationEnabled) {
        rpdo3(i, controlword::kQuickStop);
      }
      continue;
    }
    if (n.stop.status == StopStatus::kConfirmed || n.stop.status == StopStatus::kUnconfirmed) {
      continue;
    }
    if (state == DriveState::kOperationEnabled) {
      rpdo3(i, controlword::kQuickStop);
      begin_stop(i, kAcceptInhibit, now);
    } else if (state == DriveState::kQuickStopActive || state == DriveState::kFaultReactionActive ||
               state == DriveState::kFault) {
      begin_stop(i, kAcceptInhibit, now);  // Already stopping on its own: nothing is sent.
    } else {
      n.stop.status = StopStatus::kNotEnabled;  // Never reported as a powered hold.
    }
  }
}

void VdriveSession::inhibit(MonoTime now) { latch(std::nullopt, now); }

void VdriveSession::begin_stop(std::size_t i, std::uint8_t accept, MonoTime now) {
  NodeState& n = nodes_[i];
  const std::uint64_t gen = n.stop.gen + 1;  // Answers for the old track are ignored.
  n.stop = StopTrack{};
  n.stop.gen = gen;
  n.stop.status = StopStatus::kStopping;
  n.stop.accept = accept;
  n.stop.started = now;
  if (n.torque_loss) {
    resolve(i, StopStatus::kUnconfirmed);
  }
}

void VdriveSession::stop_on_sample(std::size_t i, const NodeSample& s) {
  NodeState& n = nodes_[i];
  StopTrack& t = n.stop;
  if (t.status != StopStatus::kStopping || t.mode != StopMode::kSamples) {
    return;
  }
  if (n.torque_loss) {
    resolve(i, StopStatus::kUnconfirmed);
    return;
  }
  if (s.sampled_at < t.started) {
    return;  // Not fresh: sampled before the stop began.
  }
  const DriveState state = decode_statusword(s.statusword);
  if (!at_rest(s.velocity_urad_s) || (t.accept & bit(state)) == 0U) {
    t.rest = 0;
    return;
  }
  if (t.last_seq && *t.last_seq == s.sample_seq) {
    return;  // C18: distinct samples only.
  }
  t.last_seq = s.sample_seq;
  if (++t.rest >= kRestSamples && !t.flags_pending) {
    request_flags(i);
  }
}

void VdriveSession::request_flags(std::size_t i) {
  NodeState& n = nodes_[i];
  if (n.profile != NodeProfile::kVirtual) {
    resolve(i, StopStatus::kConfirmed);
    return;
  }
  n.stop.flags_pending = true;
  const std::uint64_t gen = n.stop.gen;
  // C18: the drive's own evidence decides first.
  if (!master_.sdo_upload(
          config_.nodes[i], kIdxFeedbackExt, kSubDriveFlags, [this, i, gen](const SdoResult& r) {
            NodeState& node = nodes_[i];
            if (node.stop.gen != gen || node.stop.status != StopStatus::kStopping) {
              return;
            }
            node.stop.flags_pending = false;
            const bool against = r.status != SdoStatus::kOk ||
                                 (r.value & (kFlagDriveUnavailable | kFlagStopUnconfirmed)) != 0U ||
                                 node.torque_loss;
            resolve(i, against ? StopStatus::kUnconfirmed : StopStatus::kConfirmed);
          })) {
    resolve(i, StopStatus::kUnconfirmed);
  }
}

void VdriveSession::poll(std::size_t i, MonoTime now) {
  NodeState& n = nodes_[i];
  StopTrack& t = n.stop;
  if (t.round_out || now < t.next_round) {
    return;
  }
  t.round_out = true;
  t.round_left = 3;
  t.poll_failed = false;
  t.next_round = now + config_.bus_period;
  const std::uint64_t gen = t.gen;
  const auto done = [this, i, gen](int which) {
    return [this, i, gen, which](const SdoResult& r) {
      StopTrack& track = nodes_[i].stop;
      if (track.gen != gen || track.status != StopStatus::kStopping) {
        return;
      }
      if (r.status != SdoStatus::kOk) {
        track.poll_failed = true;
      } else if (which == 0) {
        track.poll_statusword = static_cast<std::uint16_t>(r.value);
      } else if (which == 1) {
        track.poll_velocity = static_cast<std::int32_t>(r.value);
      } else {
        track.poll_flags = static_cast<std::uint16_t>(r.value);
      }
      if (--track.round_left == 0) {
        poll_round_done(i);
      }
    };
  };
  if (!master_.sdo_upload(config_.nodes[i], kIdxStatusword, 0, done(0)) ||
      !master_.sdo_upload(config_.nodes[i], kIdxVelocity, 0, done(1)) ||
      !master_.sdo_upload(config_.nodes[i], kIdxFeedbackExt, kSubDriveFlags, done(2))) {
    resolve(i, StopStatus::kUnconfirmed);
  }
}

void VdriveSession::poll_round_done(std::size_t i) {
  NodeState& n = nodes_[i];
  StopTrack& t = n.stop;
  t.round_out = false;
  ++t.rounds;
  if (t.poll_failed || n.torque_loss ||
      (t.poll_flags & (kFlagDriveUnavailable | kFlagStopUnconfirmed)) != 0U) {
    resolve(i, StopStatus::kUnconfirmed);
    return;
  }
  const DriveState state = decode_statusword(t.poll_statusword);
  if (state == DriveState::kNotReadyToSwitchOn || state == DriveState::kSwitchOnDisabled ||
      state == DriveState::kReadyToSwitchOn || state == DriveState::kSwitchedOn) {
    // Fenced while not enabled: in Pre-operational no 0x0F can reach it any more (C10).
    resolve(i, StopStatus::kNotEnabled);
    return;
  }
  if (at_rest(t.poll_velocity) && (t.accept & bit(state)) != 0U) {
    if (++t.rest >= kRestSamples) {
      resolve(i, StopStatus::kConfirmed);
    }
  } else {
    t.rest = 0;
  }
}

void VdriveSession::resolve(std::size_t i, StopStatus status) {
  StopTrack& t = nodes_[i].stop;
  if (t.status != StopStatus::kStopping) {
    return;
  }
  t.status = status;
  ++t.gen;  // Answers still in flight for this stop are ignored.
  if (status == StopStatus::kUnconfirmed) {
    faults_.set(Fault::kStopUnconfirmed);
  }
  arm_progress(now_);
  deactivate_progress(now_);
}

bool VdriveSession::stops_resolved() const {
  return std::none_of(nodes_.begin(), nodes_.end(),
                      [](const NodeState& n) { return n.stop.status == StopStatus::kStopping; });
}

// ---- views ----

hal_contract::Feedback VdriveSession::feedback() const {
  hal_contract::Feedback f = join_.latest().value_or(hal_contract::Feedback{});
  if (phase_ == SessionPhase::kArmed) {
    std::optional<std::uint64_t> group;
    bool all = true;
    for (std::size_t i = 0; i < kSessionNodes && all; ++i) {
      const std::optional<NodeSample>& s = join_.node(i);
      const SeqEntry* e = s ? lookup(s->applied_seq) : nullptr;
      if (!s || s->applied_tag != nodes_[i].stream_tag || e == nullptr || !e->approved ||
          (group && *group != e->source_seq)) {
        all = false;
      } else {
        group = e->source_seq;
      }
    }
    if (all) {
      f.group_applied_source_seq = group;
    }
  }
  f.faults = faults_;
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    f.stop[i] = nodes_[i].stop.status;
  }
  f.counters = counters_;
  return f;
}

runtime_core::DeviceCapabilities VdriveSession::capabilities() const {
  const bool virtual_all =
      !profile_dropped_ && std::all_of(nodes_.begin(), nodes_.end(), [](const NodeState& n) {
        return n.profile == NodeProfile::kVirtual;
      });
  return virtual_all ? runtime_core::virtual_canopen_profile()
                     : runtime_core::generic_cia402_profile();
}

NodeView VdriveSession::node(std::size_t index) const {
  const NodeState& n = nodes_.at(index);
  NodeView v;
  v.node = config_.nodes.at(index).value();
  v.profile = n.profile;
  v.probe = n.probe;
  v.boot_id = n.boot_id;
  v.committed_attempt = n.committed_attempt;
  v.pending_tag = n.pending_tag;
  v.armed_tag = n.armed_tag;
  v.stop = n.stop.status;
  v.fenced = n.fenced;
  v.commit_retries = n.retries;
  return v;
}

}  // namespace canopen_master
