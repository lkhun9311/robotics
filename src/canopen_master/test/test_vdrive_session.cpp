// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <vector>

#include "bus_loop.hpp"
#include "canopen_codec/codec.hpp"
#include "canopen_master/feedback_join.hpp"
#include "canopen_master/vdrive_session.hpp"
#include "drive_core/commit.hpp"
#include "drive_core/drive.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

// The vdrive session against six in-memory drive_core::Drive through the test bus loop, in logic
// time. Expectations are read from the drives' own state and events and from the frames on the
// wire, decoded here; the session's own view is checked only where it is the subject.
namespace {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;
using canopen_master::FeedbackJoin;
using canopen_master::NodeProfile;
using canopen_master::NodeSample;
using canopen_master::SessionPhase;
using canopen_master::testing::BusLoop;
using canopen_master::testing::DriveRecord;
using canopen_master::testing::Fate;
using canopen_master::testing::kT0;
using canopen_master::testing::kW;
using canopen_master::testing::node_id;
using canopen_master::testing::WireFrame;
using drive_core::Cia402State;
using drive_core::DriveEventKind;
using hal_contract::ApprovedCommand;
using hal_contract::AuthorityId;
using hal_contract::Fault;
using hal_contract::Joints;
using hal_contract::OpState;
using hal_contract::StopStatus;
using runtime_core::Duration;
using runtime_core::microseconds;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

constexpr std::size_t kNodes = canopen_master::kSessionNodes;
constexpr Duration kTBus = canopen_master::kBusPeriod;

const AuthorityId kA{{7}, {11}, {100}};
const AuthorityId kB{{7}, {11}, {101}};
const AuthorityId kC{{7}, {12}, {102}};

// ---- wire decoding (independent of the session) ----

std::optional<std::size_t> node_of(const cc::Frame& f, std::uint16_t base) {
  if (f.cob_id > base && f.cob_id <= base + kNodes) {
    return static_cast<std::size_t>(f.cob_id - base - 1U);
  }
  return std::nullopt;
}

bool is_controlword(const WireFrame& w, std::uint16_t cw, std::optional<std::size_t> node = {}) {
  const std::optional<std::size_t> n = node_of(w.frame, 0x400);
  return w.from_master && n && (!node || *n == *node) && w.frame.dlc == 2 &&
         (w.frame.data[0] | (w.frame.data[1] << 8U)) == cw;
}

bool is_command_pdo(const WireFrame& w) {
  return w.from_master && (node_of(w.frame, 0x200) || node_of(w.frame, 0x300));
}

struct Rpdo1At {
  std::size_t node;
  od::Rpdo1 pdo;
};
std::optional<Rpdo1At> rpdo1(const WireFrame& w) {
  if (!w.from_master) return std::nullopt;
  if (const auto n = node_of(w.frame, 0x200)) {
    if (const auto p = cc::unpack_rpdo1(node_id(*n), w.frame)) return Rpdo1At{*n, *p};
  }
  return std::nullopt;
}
struct Rpdo2At {
  std::size_t node;
  od::Rpdo2 pdo;
};
std::optional<Rpdo2At> rpdo2(const WireFrame& w) {
  if (!w.from_master) return std::nullopt;
  if (const auto n = node_of(w.frame, 0x300)) {
    if (const auto p = cc::unpack_rpdo2(node_id(*n), w.frame)) return Rpdo2At{*n, *p};
  }
  return std::nullopt;
}

struct SdoAt {
  std::size_t node;
  cc::Sdo sdo;
};
std::optional<SdoAt> sdo_request(const WireFrame& w) {
  if (!w.from_master) return std::nullopt;
  if (const auto n = node_of(w.frame, 0x600)) {
    if (const auto s = cc::decode_sdo(node_id(*n), w.frame)) return SdoAt{*n, *s};
  }
  return std::nullopt;
}
std::optional<SdoAt> sdo_response(const WireFrame& w) {
  if (w.from_master) return std::nullopt;
  if (const auto n = node_of(w.frame, 0x580)) {
    if (const auto s = cc::decode_sdo(node_id(*n), w.frame)) return SdoAt{*n, *s};
  }
  return std::nullopt;
}

bool is_nmt(const WireFrame& w, cc::NmtCommand command, std::optional<std::size_t> node = {}) {
  if (!w.from_master || w.frame.cob_id != 0x000) return false;
  const auto m = cc::decode_nmt(w.frame);
  return m && m->command == command && m->target &&
         (!node || m->target->value() == node_id(*node).value());
}

bool is_download(const SdoAt& s, std::uint16_t index, std::optional<std::uint8_t> sub = {}) {
  return s.sdo.type == cc::SdoType::kDownloadRequest && s.sdo.address.index == index &&
         (!sub || s.sdo.address.sub == *sub);
}

std::uint32_t wire_us(MonoTime t) {
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(t.ns / 1000));
}

// ---- driving the session ----

void bring_up(BusLoop& l) {
  l.power_on();
  l.run(milliseconds(150));  // Past the boot quarantine, so a commit is never refused for it.
  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  ASSERT_EQ(l.activate(), OpState::kSucceeded);
  ASSERT_EQ(l.session.phase(), SessionPhase::kActive);
}

void bring_up_and_arm(BusLoop& l, const AuthorityId& id = kA, Duration hold = milliseconds(800)) {
  bring_up(l);
  ASSERT_EQ(l.arm(id, hold), OpState::kSucceeded) << "INCONCLUSIVE: arm failed";
}

// Approved commands, one W before each SYNC slot (a controller writing just before the bus
// cycle), so each is dispatched at that slot.
struct Ramp {
  AuthorityId id;
  Joints position{};
  Joints step{};
  std::uint64_t seq{0};
  Duration life{milliseconds(48)};
  std::optional<ApprovedCommand> last;

  void offer(BusLoop& l) {
    for (std::size_t i = 0; i < kNodes; ++i) position[i] += step[i];
    const ApprovedCommand c{id, ++seq, position, l.now, l.now + life};
    l.session.offer(c);
    last = c;
  }
};

Ramp ramp_from_here(BusLoop& l, const AuthorityId& id, double velocity) {
  Ramp r;
  r.id = id;
  for (std::size_t i = 0; i < kNodes; ++i) {
    r.position[i] = static_cast<double>(l.drives[i]->joint().position_nrad) * 1e-9;
    r.step[i] = velocity * 0.010;
  }
  return r;
}

void stream(BusLoop& l, Ramp& r, Duration d) {
  const MonoTime end = l.now + d;
  std::optional<MonoTime> last_sync;
  while (l.now < end) {
    l.step();
    if (l.sync_this_step) last_sync = l.now;
    if (last_sync && l.now == *last_sync + kTBus - kW) r.offer(l);
  }
}

std::optional<MonoTime> first_quick_stop(const BusLoop& l, std::size_t node) {
  for (const DriveRecord& r : l.drive_events) {
    if (r.node == node && r.event.kind == DriveEventKind::kQuickStop) return r.event.at;
  }
  return std::nullopt;
}

// ============================ bring-up ============================

TEST(Session, ConfigureProbesIdentityAndTag) {
  BusLoop l;
  l.power_on();
  l.step();
  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  const canopen_master::Identity eds = canopen_master::eds_identity();
  std::array<std::uint32_t, kNodes> boots{};
  for (std::size_t i = 0; i < kNodes; ++i) {
    const canopen_master::NodeView v = l.session.node(i);
    EXPECT_EQ(v.profile, NodeProfile::kVirtual) << i;
    ASSERT_TRUE(v.probe.identity[0] && v.probe.identity[1] && v.probe.identity[2]);
    EXPECT_EQ(*v.probe.identity[0], eds.vendor);
    EXPECT_EQ(*v.probe.identity[1], eds.product);
    EXPECT_EQ(*v.probe.identity[2], eds.revision);
    EXPECT_TRUE(v.probe.identity[3].has_value());
    EXPECT_EQ(v.probe.boot_id, l.drives[i]->issuer().boot_id());
    EXPECT_EQ(v.boot_id, l.drives[i]->issuer().boot_id());
    EXPECT_EQ(v.probe.command_timeout_ms, 30U);
    EXPECT_EQ(v.probe.mode_display, 8U);
    EXPECT_EQ(v.probe.committed_attempt, 0U);
    EXPECT_EQ(v.probe.exec_tag, 0U);  // No tag before a commit.
    boots[i] = *v.boot_id;
    // On the wire: the ten probe uploads, then Reset Communication, then 0x2013 and 0x2016.
    const std::array<std::pair<std::uint16_t, std::uint8_t>, 10> probe{{{0x1000, 0},
                                                                        {0x1018, 1},
                                                                        {0x1018, 2},
                                                                        {0x1018, 3},
                                                                        {0x1018, 4},
                                                                        {0x2013, 0},
                                                                        {0x2014, 0},
                                                                        {0x2016, 0},
                                                                        {0x6061, 0},
                                                                        {0x2012, 0}}};
    std::uint64_t reset_at = 0;
    for (const WireFrame& w : l.wire) {
      if (is_nmt(w, cc::NmtCommand::kResetCommunication, i)) reset_at = w.order;
    }
    ASSERT_NE(reset_at, 0U) << "no Reset Communication to node " << i;
    std::vector<std::pair<std::uint16_t, std::uint8_t>> before, after;
    for (const WireFrame& w : l.wire) {
      const auto s = sdo_request(w);
      if (!s || s->node != i) continue;
      EXPECT_EQ(s->sdo.type, cc::SdoType::kUploadRequest);
      (w.order < reset_at ? before : after).emplace_back(s->sdo.address.index, s->sdo.address.sub);
    }
    EXPECT_EQ(before, std::vector(probe.begin(), probe.end())) << "node " << i;
    const std::vector<std::pair<std::uint16_t, std::uint8_t>> reread{{0x2013, 0}, {0x2016, 0}};
    EXPECT_EQ(after, reread) << "node " << i;
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      EXPECT_NE(boots[i], boots[j]);
      EXPECT_NE(drive_core::exec_tag_start(boots[i]), drive_core::exec_tag_start(boots[j]));
    }
  }
  const runtime_core::DeviceCapabilities caps = l.session.capabilities();
  EXPECT_EQ(caps.device_side_expiry, runtime_core::Support::kSupported);
  EXPECT_EQ(caps.execution_generation, runtime_core::Support::kSupported);
  EXPECT_FALSE(l.session.profile_confirmed());  // Provisional until the first arm (C2).
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, GenericNodeConfiguresWithoutExtensionReads) {
  BusLoop l;
  // Node 4 has no 0x2012: it answers the probe's upload with abort 0x06020000 (no object).
  l.intercept[3] = [](const cc::Frame& f) -> std::optional<cc::Frame> {
    const auto s = cc::decode_sdo(node_id(3), f);
    if (f.cob_id != 0x604 || !s || s->type != cc::SdoType::kUploadRequest ||
        s->address.index != 0x2012) {
      return std::nullopt;
    }
    cc::Sdo abort{};
    abort.type = cc::SdoType::kAbortToClient;
    abort.address = s->address;
    abort.abort_code = 0x0602'0000;
    return cc::encode_sdo(node_id(3), abort);
  };
  l.power_on();
  l.step();
  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  EXPECT_EQ(l.session.node(3).profile, NodeProfile::kGeneric);
  for (std::size_t i : {0U, 1U, 2U, 4U, 5U}) {
    EXPECT_EQ(l.session.node(i).profile, NodeProfile::kVirtual) << i;
  }
  const runtime_core::DeviceCapabilities caps = l.session.capabilities();
  const runtime_core::DeviceCapabilities generic = runtime_core::generic_cia402_profile();
  EXPECT_EQ(caps.device_side_expiry, generic.device_side_expiry);
  EXPECT_EQ(caps.execution_generation, generic.execution_generation);
  EXPECT_EQ(caps.apply_ack, generic.apply_ack);
  EXPECT_EQ(caps.local_stop, generic.local_stop);
  // Node 4: the probe's 0x2012 upload is its last SDO request; no reset, nothing afterwards.
  std::uint64_t probe_tag_at = 0;
  for (const WireFrame& w : l.wire) {
    const auto s = sdo_request(w);
    if (s && s->node == 3 && s->sdo.address.index == 0x2012) probe_tag_at = w.order;
  }
  ASSERT_NE(probe_tag_at, 0U) << "INCONCLUSIVE: the probe never read 0x2012 of node 4";
  for (const WireFrame& w : l.wire) {
    EXPECT_FALSE(is_nmt(w, cc::NmtCommand::kResetCommunication, 3));
    const auto s = sdo_request(w);
    if (s && s->node == 3) {
      EXPECT_LE(w.order, probe_tag_at) << "SDO after the probe";
    }
  }
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, ActivateReachesSwitchedOnWithoutEnable) {
  BusLoop l;
  bring_up(l);
  l.run(milliseconds(100));
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kSwitchedOn) << i;
    EXPECT_EQ(l.drives[i]->nmt().state(), cc::NmtState::kOperational) << i;
  }
  for (const WireFrame& w : l.wire) {
    EXPECT_FALSE(is_controlword(w, 0x0F)) << "an enable during activation";
    EXPECT_FALSE(is_command_pdo(w)) << "a command before arming";
  }
  EXPECT_FALSE(l.session.inhibited());
  EXPECT_EQ(l.session.counts().unexpected, 0U);
}

// ============================ arm ============================

TEST(Session, ArmCommitsThenEnablesAfterSecondUpdate) {
  BusLoop l;
  bring_up_and_arm(l);
  std::uint64_t last_commit = 0;
  std::uint64_t first_command = 0;
  std::uint64_t first_enable = 0;
  std::vector<MonoTime> update_slots;  // Slots with RPDOs before the first enable.
  for (const WireFrame& w : l.wire) {
    if (const auto s = sdo_request(w); s && is_download(*s, 0x2011)) last_commit = w.order;
    if (is_command_pdo(w) && first_command == 0) first_command = w.order;
    if (is_controlword(w, 0x0F) && first_enable == 0) first_enable = w.order;
    if (rpdo1(w) && first_enable == 0 && (update_slots.empty() || update_slots.back() != w.at)) {
      update_slots.push_back(w.at);
    }
  }
  ASSERT_NE(last_commit, 0U);
  ASSERT_NE(first_enable, 0U);
  EXPECT_LT(last_commit, first_command) << "a command before every commit was written";
  EXPECT_EQ(update_slots.size(), 2U) << "enable after exactly two stream updates";
  std::size_t enables = 0;
  for (const WireFrame& w : l.wire) {
    if (is_controlword(w, 0x0F)) {
      ++enables;
      EXPECT_EQ(w.at, update_slots.back()) << "enable in the slot of the second update";
    }
  }
  EXPECT_EQ(enables, kNodes);
  EXPECT_EQ(l.session.armed_id(), kA);
  EXPECT_TRUE(l.session.profile_confirmed());
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kOperationEnabled) << i;
    EXPECT_EQ(l.session.node(i).armed_tag, l.drives[i]->issuer().current()) << i;
    EXPECT_EQ(l.drives[i]->issuer().generation(), kA.generation.value);
  }
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, FirstAttemptAboveObservedCommitted) {
  BusLoop l;
  l.power_on();
  l.run(milliseconds(150));
  // Before this session ever configures, another master committed attempt 7 on node 2.
  const std::size_t n = 1;
  const auto sdo_write = [&](std::uint16_t index, std::uint8_t sub, std::uint32_t value) {
    cc::Sdo s{};
    s.type = cc::SdoType::kDownloadRequest;
    s.address = {index, sub};
    s.size = 4;
    s.value = value;
    const auto r = l.to_drive(n, *cc::encode_sdo(node_id(n), s));
    ASSERT_EQ(r.size(), 1U);
    const auto reply = cc::decode_sdo(node_id(n), r[0]);
    ASSERT_TRUE(reply && reply->type == cc::SdoType::kDownloadResponse) << std::hex << index;
  };
  l.to_drive(n, cc::encode_nmt({cc::NmtCommand::kStart, node_id(n)}));
  l.to_drive(n, cc::pack_rpdo3(node_id(n), {0x06}));
  l.to_drive(n, cc::pack_rpdo3(node_id(n), {0x07}));
  const drive_core::CommitRequest seven{7, 5, 0, 3};
  sdo_write(0x2010, 4, 7);
  sdo_write(0x2010, 1, 5);
  sdo_write(0x2010, 2, 0);
  sdo_write(0x2010, 3, 3);
  sdo_write(0x2011, 0, seven.crc());
  ASSERT_EQ(l.drives[n]->committed_attempt(), 7U) << "INCONCLUSIVE: setup commit failed";

  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  EXPECT_EQ(l.session.node(n).committed_attempt, 7U);
  ASSERT_EQ(l.activate(), OpState::kSucceeded);
  ASSERT_EQ(l.arm(kA, milliseconds(500)), OpState::kSucceeded);
  EXPECT_EQ(l.drives[n]->committed_attempt(), 8U);
  for (const WireFrame& w : l.wire) {
    const auto s = sdo_request(w);
    if (s && is_download(*s, 0x2010, 4)) {
      EXPECT_EQ(s->sdo.value, s->node == n ? 8U : 1U) << "node " << s->node;
    }
  }
}

TEST(Session, ColdStartArmWaitsOutQuarantine) {
  BusLoop l;
  const MonoTime boot = l.now;
  l.power_on();
  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  ASSERT_EQ(l.activate(), OpState::kSucceeded);
  ASSERT_LE(l.now, boot + milliseconds(20)) << "INCONCLUSIVE: bring-up slower than 20 ms";
  l.run(boot + milliseconds(20) - l.now);
  ASSERT_EQ(l.arm(kA, milliseconds(800)), OpState::kSucceeded);
  EXPECT_EQ(l.session.counts().quarantine_retries, kNodes);
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.session.node(i).commit_retries, 1) << i;
    EXPECT_EQ(l.drives[i]->committed_attempt(), 1U);
  }
  // Per node: the first 0x2011 inside the quarantine is refused, the retry after it succeeds.
  std::array<std::vector<MonoTime>, kNodes> commits;
  for (const WireFrame& w : l.wire) {
    if (const auto s = sdo_request(w); s && is_download(*s, 0x2011))
      commits[s->node].push_back(w.at);
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    ASSERT_EQ(commits[i].size(), 2U) << i;
    EXPECT_LT(commits[i][0], boot + milliseconds(100));
    EXPECT_GE(commits[i][1], boot + milliseconds(100) + kTBus);
  }
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, AttemptIdStrictlyIncreasesAcrossArms) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  ASSERT_EQ(l.arm(kB, milliseconds(800)), OpState::kSucceeded);
  ASSERT_EQ(l.arm(kC, milliseconds(800)), OpState::kSucceeded);
  std::array<std::vector<std::uint32_t>, kNodes> attempts;
  for (const WireFrame& w : l.wire) {
    if (const auto s = sdo_request(w); s && is_download(*s, 0x2010, 4)) {
      attempts[s->node].push_back(s->sdo.value);
    }
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(attempts[i], (std::vector<std::uint32_t>{1, 2, 3})) << i;
    EXPECT_EQ(l.drives[i]->committed_attempt(), 3U);
  }
  EXPECT_EQ(l.session.armed_id(), kC);
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, GenerationChangeDisablesBeforeCommit) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 1.0);
  stream(l, r, milliseconds(100));
  ASSERT_EQ(std::abs(l.drives[0]->joint().velocity_nrad_s), drive_core::kVMaxNradS)
      << "INCONCLUSIVE: not moving";
  // Every staging or commit write reaching a drive while any drive is Operation Enabled.
  int violations = 0;
  int commit_writes = 0;
  l.filter = [&](const WireFrame& w) {
    if (const auto s = sdo_request(w); s && (is_download(*s, 0x2010) || is_download(*s, 0x2011))) {
      ++commit_writes;
      for (const auto& d : l.drives) {
        if (d->cia402().state() == Cia402State::kOperationEnabled) ++violations;
      }
    }
    return Fate::kDeliver;
  };
  ASSERT_EQ(l.arm(kB, milliseconds(800)), OpState::kSucceeded);
  EXPECT_EQ(commit_writes, static_cast<int>(5 * kNodes));
  EXPECT_EQ(violations, 0);
  for (const auto& d : l.drives) EXPECT_EQ(d->issuer().generation(), kB.generation.value);
  EXPECT_FALSE(l.session.inhibited());
}

TEST(Session, ArmPhasesAreGroupWide) {
  canopen_master::MasterConfig mc = canopen_master::testing::default_master_config();
  mc.sdo_timeout = milliseconds(5000);  // The op deadline, not the SDO timeout, ends the arm.
  BusLoop l(mc);
  bring_up(l);
  bool arming = false;
  l.filter = [&](const WireFrame& w) {
    // Node 6's commit answers are held past the op deadline.
    return arming && sdo_response(w) && sdo_response(w)->node == 5 ? Fate::kHold : Fate::kDeliver;
  };
  const hal_contract::OpTicket t = l.ticket(hal_contract::kArmTimeout);
  arming = true;
  l.session.arm(t, {kA, l.now + milliseconds(800), t.op_id});
  EXPECT_EQ(l.wait(t.op_id, milliseconds(1200)), OpState::kFailed);
  ASSERT_FALSE(l.held.empty()) << "INCONCLUSIVE: nothing of node 6 was held";
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(l.drives[i]->committed_attempt(), 1U) << "INCONCLUSIVE: node " << i;
  }
  arming = false;
  const std::uint64_t failed_at = l.wire.back().order;
  for (const WireFrame& w : l.held) l.deliver_from_drive(w.frame);
  l.run(milliseconds(200));
  for (const WireFrame& w : l.wire) {
    EXPECT_FALSE(is_controlword(w, 0x0F));
    EXPECT_FALSE(is_command_pdo(w));
    const auto s = sdo_request(w);
    if (w.order > failed_at && s) {
      EXPECT_FALSE(is_download(*s, 0x2010) || is_download(*s, 0x2011)) << "node " << s->node;
    }
  }
  for (const auto& d : l.drives) EXPECT_NE(d->cia402().state(), Cia402State::kOperationEnabled);
  EXPECT_TRUE(l.session.inhibited());
  EXPECT_TRUE(l.session.feedback().faults.has(Fault::kArmFailed));
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.session.feedback().stop[i], StopStatus::kNotEnabled) << i;
  }
}

TEST(Session, TimedOutArmNeverEnablesLater) {
  canopen_master::MasterConfig mc = canopen_master::testing::default_master_config();
  mc.sdo_timeout = milliseconds(5000);  // The held answers are still awaited when they arrive.
  BusLoop l(mc);
  bring_up(l);
  bool arming = false;
  l.filter = [&](const WireFrame& w) {
    return arming && sdo_response(w) ? Fate::kHold : Fate::kDeliver;
  };
  const hal_contract::OpTicket t = l.ticket(hal_contract::kArmTimeout);
  arming = true;
  l.session.arm(t, {kA, l.now + milliseconds(800), t.op_id});
  l.step();  // The arm starts; its first SDO requests leave, their answers are held.
  ASSERT_EQ(l.session.op_state(t.op_id), OpState::kPending);
  // The worker sleeps past the deadline; nobody waits for the op.
  l.advance(milliseconds(1100));
  EXPECT_EQ(l.session.op_state(t.op_id), OpState::kFailed);
  ASSERT_FALSE(l.held.empty()) << "INCONCLUSIVE: no SDO answer was held";
  const std::uint64_t failed_at = l.wire.back().order;
  arming = false;
  for (const WireFrame& w : l.held) l.deliver_from_drive(w.frame);
  l.run(milliseconds(300));
  for (const WireFrame& w : l.wire) {
    EXPECT_FALSE(is_controlword(w, 0x0F));
    EXPECT_FALSE(is_command_pdo(w));
    // The timed-out op sends nothing more: no staging or commit request after it failed.
    const auto s = sdo_request(w);
    if (w.order > failed_at && s) {
      EXPECT_FALSE(is_download(*s, 0x2010) || is_download(*s, 0x2011)) << "node " << s->node;
    }
  }
  for (const auto& d : l.drives) {
    EXPECT_NE(d->cia402().state(), Cia402State::kOperationEnabled);
    EXPECT_EQ(d->committed_attempt(), 0U) << "a commit completed after the op failed";
  }
}

TEST(Session, DelayedEnableAfterCancelDoesNotReenable) {
  BusLoop l;
  bring_up(l);
  // Node 1's enable is captured: it never enables, so the arm is never confirmed and fails.
  l.filter = [&](const WireFrame& w) {
    return is_controlword(w, 0x0F, 0) ? Fate::kHold : Fate::kDeliver;
  };
  EXPECT_EQ(l.arm(kA, milliseconds(800)), OpState::kFailed);
  ASSERT_EQ(l.held.size(), 1U) << "INCONCLUSIVE: the enable was not captured";
  ASSERT_EQ(l.drives[1]->nmt().state(), cc::NmtState::kPreOperational) << "no fence";
  // The captured enable arrives after the fence, and a duplicate reaches every other node.
  for (std::size_t i = 0; i < kNodes; ++i) {
    l.deliver_from_master(cc::pack_rpdo3(node_id(i), {0x0F}));
  }
  l.run(milliseconds(100));
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_NE(l.drives[i]->cia402().state(), Cia402State::kOperationEnabled) << i;
    EXPECT_EQ(l.drives[i]->nmt().state(), cc::NmtState::kPreOperational) << i;
    EXPECT_TRUE(l.session.node(i).fenced) << i;
  }
}

TEST(Session, PartialEnableStopsPerState) {
  BusLoop l;
  bring_up(l);
  // Nodes 4-6 never get their enable: the arm is cancelled with 3 of 6 enabled.
  l.filter = [&](const WireFrame& w) {
    const auto n = node_of(w.frame, 0x400);
    return is_controlword(w, 0x0F) && n && *n >= 3 ? Fate::kDrop : Fate::kDeliver;
  };
  EXPECT_EQ(l.arm(kA, milliseconds(800)), OpState::kFailed);
  for (std::size_t i = 0; i < 3; ++i) {
    ASSERT_TRUE(first_quick_stop(l, i)) << "INCONCLUSIVE: node " << i << " never enabled";
  }
  l.run(milliseconds(300));
  for (std::size_t i = 0; i < kNodes; ++i) {
    const StopStatus s = l.session.feedback().stop[i];
    if (i < 3) {
      EXPECT_EQ(s, StopStatus::kConfirmed) << i;
      EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kQuickStopActive) << i;
    } else {
      EXPECT_EQ(s, StopStatus::kNotEnabled) << i;
      EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kSwitchedOn) << i;
    }
  }
}

TEST(Session, OldCommandQueuedBeforeRearmIsDiscarded) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.2);
  stream(l, r, milliseconds(60));
  // A command queued in the box, then a re-arm before the next slot.
  Joints odd{};
  odd.fill(0.123456);
  const hal_contract::CommandCounters before = l.session.feedback().counters;
  l.session.offer({kA, r.seq + 1, odd, l.now, l.now + milliseconds(48)});
  ASSERT_EQ(l.arm(kB, milliseconds(800)), OpState::kSucceeded);
  l.run(milliseconds(50));
  // Discarded by the arm: neither dispatched, nor superseded, nor expired in the box later.
  EXPECT_EQ(l.session.feedback().counters.expired_before_dispatch, before.expired_before_dispatch);
  EXPECT_EQ(l.session.feedback().counters.superseded, before.superseded);
  for (const WireFrame& w : l.wire) {
    if (const auto p = rpdo1(w)) {
      EXPECT_NE(p->pdo.target_position, 123456) << "old command sent";
    }
  }
}

TEST(Session, FullLifecycleRaisesNoInhibit) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  l.run(milliseconds(50));
  ASSERT_EQ(l.deactivate(), OpState::kSucceeded);
  ASSERT_EQ(l.activate(), OpState::kSucceeded);
  ASSERT_EQ(l.arm(kB, milliseconds(800)), OpState::kSucceeded);
  l.run(milliseconds(50));
  EXPECT_FALSE(l.session.inhibited());
  EXPECT_EQ(l.session.counts().unexpected, 0U);
  EXPECT_FALSE(l.session.feedback().faults.any());
}

TEST(Session, NeverWritesConstObjects) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.3);
  stream(l, r, milliseconds(50));
  ASSERT_EQ(l.arm(kB, milliseconds(800)), OpState::kSucceeded);
  ASSERT_EQ(l.deactivate(), OpState::kSucceeded);
  ASSERT_EQ(l.configure(), OpState::kSucceeded);
  std::size_t downloads = 0;
  for (const WireFrame& w : l.wire) {
    const auto s = sdo_request(w);
    if (!s || s->sdo.type != cc::SdoType::kDownloadRequest) continue;
    ++downloads;
    const std::uint16_t x = s->sdo.address.index;
    const bool pdo_object = (x >= 0x1400 && x <= 0x1402) || (x >= 0x1600 && x <= 0x1602) ||
                            (x >= 0x1800 && x <= 0x1802) || (x >= 0x1A00 && x <= 0x1A02);
    EXPECT_FALSE(x == 0x1017 || pdo_object) << std::hex << x;
  }
  EXPECT_GT(downloads, 0U) << "INCONCLUSIVE: no SDO download at all";
}

// ============================ inhibit and stops ============================

TEST(Session, ForeignAppliedTagInhibits) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  ASSERT_FALSE(l.session.inhibited()) << "INCONCLUSIVE";
  // Node 2 reports a sample whose applied_tag is neither its pending nor its armed tag.
  const std::uint16_t armed = l.drives[1]->issuer().current();
  const std::uint16_t seq = static_cast<std::uint16_t>(l.session.join().node(1)->sample_seq + 1);
  l.deliver_from_drive(cc::pack_tpdo1(node_id(1), {0x1237, 0, seq}));
  l.deliver_from_drive(cc::pack_tpdo2(node_id(1), {0, seq, 0}));
  l.deliver_from_drive(
      cc::pack_tpdo3(node_id(1), {wire_us(l.now), seq, static_cast<std::uint16_t>(armed + 1)}));
  EXPECT_TRUE(l.session.inhibited());
  l.run(milliseconds(250));
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.session.feedback().stop[i], StopStatus::kConfirmed) << i;
  }
}

TEST(Session, OneNodeStopInhibitsGroup) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.5);
  stream(l, r, milliseconds(100));
  l.filter = [](const WireFrame& w) {
    return w.from_master && w.frame.cob_id == 0x303 ? Fate::kDrop : Fate::kDeliver;
  };
  stream(l, r, milliseconds(400));
  const std::optional<MonoTime> stop3 = first_quick_stop(l, 2);
  ASSERT_TRUE(stop3) << "INCONCLUSIVE: node 3 never stopped";
  for (const DriveRecord& e : l.drive_events) {
    if (e.node == 2 && e.event.kind == DriveEventKind::kQuickStop) {
      EXPECT_EQ(e.event.reason, drive_core::QuickStopReason::kTimeout);  // Its own local stop.
      break;
    }
  }
  std::uint64_t last_quick_stop = 0;
  for (std::size_t i : {0U, 1U, 3U, 4U, 5U}) {
    std::optional<WireFrame> qs;
    for (const WireFrame& w : l.wire) {
      if (is_controlword(w, 0x02, i)) {
        qs = w;
        break;
      }
    }
    ASSERT_TRUE(qs) << "no Quick Stop to node " << i;
    EXPECT_LE(qs->at - *stop3, Duration{2 * kTBus.ns}) << i;
    last_quick_stop = std::max(last_quick_stop, qs->order);
  }
  for (const DriveRecord& e : l.drive_events) {
    if (e.order > last_quick_stop) {
      EXPECT_NE(e.event.kind, DriveEventKind::kActuated) << e.node;
    }
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.session.feedback().stop[i], StopStatus::kConfirmed) << i;
  }
  EXPECT_TRUE(l.session.inhibited());
  EXPECT_TRUE(l.session.feedback().faults.has(Fault::kPolicyStop));
}

TEST(Session, FollowingErrorOnMovingAxisInhibitsGroup) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 1.0);
  stream(l, r, milliseconds(100));
  ASSERT_EQ(std::abs(l.drives[1]->joint().velocity_nrad_s), drive_core::kVMaxNradS)
      << "INCONCLUSIVE: not moving";
  l.inject_fault(1, drive_core::kErrorFollowing);
  ASSERT_EQ(l.drives[1]->cia402().state(), Cia402State::kFaultReactionActive);
  stream(l, r, milliseconds(400));
  // The first observation: the EMCY, or a TPDO1 reporting Fault Reaction Active.
  std::optional<MonoTime> seen;
  for (const WireFrame& w : l.wire) {
    if (!w.from_master && (w.frame.cob_id == 0x082 || w.frame.cob_id == 0x182)) {
      if (w.frame.cob_id == 0x082 ||
          (w.frame.data[0] & 0x4FU) == 0x0FU) {  // statusword Fault Reaction Active
        seen = w.at;
        break;
      }
    }
  }
  ASSERT_TRUE(seen) << "INCONCLUSIVE: the fault was never observable";
  for (std::size_t i : {0U, 2U, 3U, 4U, 5U}) {
    std::optional<MonoTime> qs;
    for (const WireFrame& w : l.wire) {
      if (is_controlword(w, 0x02, i)) {
        qs = w.at;
        break;
      }
    }
    ASSERT_TRUE(qs) << i;
    EXPECT_LE(*qs - *seen, Duration{2 * kTBus.ns}) << i;
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(l.session.feedback().stop[i], StopStatus::kConfirmed) << i;
  }
  EXPECT_EQ(l.drives[1]->cia402().state(), Cia402State::kFault);
  EXPECT_TRUE(l.session.feedback().faults.has(Fault::kDriveFault));
}

TEST(Session, LowSpeedTorqueLossIsUnconfirmed) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  // Node 4 creeps: 10 urad per command, i.e. 0.005 rad/s for the W after each actuation.
  Ramp r = ramp_from_here(l, kA, 0.0);
  r.step[3] = 10e-6;
  bool injected = false;
  const MonoTime end = l.now + milliseconds(300);
  while (l.now < end && !injected) {
    l.step();
    if (l.drives[3]->joint().velocity_nrad_s == 5'000'000) {
      l.inject_torque_loss(3);
      injected = true;
    } else if (l.sync_this_step) {
      r.offer(l);
    }
  }
  ASSERT_TRUE(injected) << "INCONCLUSIVE: node 4 never moved at 0.005 rad/s";
  bool ever_confirmed = false;
  for (int k = 0; k < 200; ++k) {
    l.step();
    ever_confirmed = ever_confirmed || l.session.feedback().stop[3] == StopStatus::kConfirmed;
  }
  EXPECT_LT(std::abs(l.drives[3]->joint().velocity_nrad_s), 10'000'000) << "below rest speed";
  EXPECT_FALSE(ever_confirmed);
  EXPECT_EQ(l.session.feedback().stop[3], StopStatus::kUnconfirmed);
  EXPECT_TRUE(l.session.feedback().faults.has(Fault::kStopUnconfirmed));
}

TEST(Session, StopConfirmationWithinDerivedDeadline) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 1.0);
  stream(l, r, milliseconds(200));
  for (const auto& d : l.drives) {
    ASSERT_EQ(std::abs(d->joint().velocity_nrad_s), drive_core::kVMaxNradS) << "INCONCLUSIVE";
  }
  // Worst SYNC phase: the inhibit comes right after a SYNC was sampled.
  l.to_next_sync();
  r.offer(l);
  const MonoTime inhibit_at = l.now;
  l.session.inhibit(l.now);
  std::array<std::optional<MonoTime>, kNodes> confirmed_at{};
  for (int k = 0; k < 150; ++k) {
    l.step();
    for (std::size_t i = 0; i < kNodes; ++i) {
      if (!confirmed_at[i] && l.session.feedback().stop[i] == StopStatus::kConfirmed) {
        confirmed_at[i] = l.now;
      }
    }
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    ASSERT_TRUE(confirmed_at[i]) << i;
    EXPECT_LE(*confirmed_at[i] - inhibit_at, canopen_master::kStopConfirmDeadline) << i;
  }
}

TEST(Session, DeactivateConfirmsStopBeforePreop) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.5);
  stream(l, r, milliseconds(100));
  std::optional<std::array<StopStatus, kNodes>> at_preop;
  std::uint64_t preop_order = 0;
  l.filter = [&](const WireFrame& w) {
    if (!at_preop && is_nmt(w, cc::NmtCommand::kEnterPreOperational)) {
      at_preop = l.session.feedback().stop;
      preop_order = w.order;
    }
    return Fate::kDeliver;
  };
  const std::uint64_t start_order = l.wire.back().order;
  ASSERT_EQ(l.deactivate(), OpState::kSucceeded);
  l.run(milliseconds(50));
  ASSERT_TRUE(at_preop) << "no Enter Pre-operational";
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ((*at_preop)[i], StopStatus::kConfirmed) << i;
    EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kSwitchedOn) << i;
    EXPECT_EQ(l.drives[i]->nmt().state(), cc::NmtState::kPreOperational) << i;
  }
  // TPDO1 kept flowing from every node until then: braking from 0.5 rad/s plus 5 rest samples.
  std::array<int, kNodes> tpdos{};
  for (const WireFrame& w : l.wire) {
    const auto n = node_of(w.frame, 0x180);
    if (!w.from_master && n && w.order > start_order) {
      EXPECT_LT(w.order, preop_order) << "a TPDO after Pre-operational";
      ++tpdos[*n];
    }
  }
  for (std::size_t i = 0; i < kNodes; ++i) EXPECT_GE(tpdos[i], 5 + 5) << i;
  EXPECT_FALSE(l.session.inhibited());
}

// ============================ command stream ============================

TEST(Session, DispatchMarginBoundary) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  const Duration margin =
      canopen_master::testing::plus(canopen_master::testing::plus(kTBus, kW), milliseconds(1));
  std::uint64_t seq = 0;
  // Offers right after a SYNC: dispatched at the next slot, whose next SYNC is two T_bus away.
  const auto offer = [&](double target, MonoTime until) {
    Joints p{};
    p.fill(target);
    l.session.offer({kA, ++seq, p, l.now, until});
  };
  const auto sent_at = [&](MonoTime slot, std::int32_t target) {
    for (const WireFrame& w : l.wire) {
      if (const auto r1 = rpdo1(w); r1 && w.at == slot && r1->pdo.target_position == target) {
        return true;
      }
    }
    return false;
  };
  // A sent case command closes soon after its actuation and the drive stops, so the cases that
  // must not be sent come first, each followed by a long-lived command that keeps it running.
  struct Case {
    double target;
    Duration beyond;  // Past next SYNC + W + 1 ms.
    bool sent;
  };
  const std::array<Case, 4> cases{{{0.002, Duration{0}, false},
                                   {0.003, microseconds(-1), false},
                                   {0.004, Duration{999}, false},  // Floors to the margin.
                                   {0.001, microseconds(1), true}}};
  double keep = 0.0;
  for (const Case& c : cases) {
    l.to_next_sync();
    const MonoTime slot = l.now + kTBus;
    const MonoTime until = slot + margin + c.beyond;
    EXPECT_EQ(l.session.can_apply(until, l.now), c.sent) << c.target;
    offer(c.target, until);
    l.to_next_sync();
    ASSERT_EQ(l.now, slot);
    EXPECT_EQ(sent_at(slot, static_cast<std::int32_t>(std::lround(c.target * 1e6))), c.sent)
        << c.target;
    if (!c.sent) {
      keep += 1e-4;
      offer(keep, l.now + milliseconds(48));
    }
  }
  EXPECT_EQ(l.session.feedback().counters.expired_before_dispatch, 3U);
  EXPECT_FALSE(l.session.inhibited()) << "INCONCLUSIVE: the keep-alive commands did not hold";
}

TEST(Session, CommandValidityNeverExceedsApproval) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.3);
  std::map<std::int32_t, MonoTime> until_of_target;  // node 0 target -> approval valid_until
  const MonoTime end = l.now + milliseconds(300);
  int k = 0;
  const std::uint64_t start_order = l.wire.back().order;
  std::optional<MonoTime> last_sync;
  while (l.now < end) {
    l.step();
    if (l.sync_this_step) last_sync = l.now;
    if (!last_sync || l.now != *last_sync + kTBus - kW) continue;
    // 24..48 ms: each still open when the next one is actuated, so the drives keep running.
    r.life = milliseconds(24 + (k++ * 7) % 25);
    r.offer(l);
    until_of_target[static_cast<std::int32_t>(std::lround(r.position[0] * 1e6))] =
        r.last->valid_until;
  }
  std::map<std::uint16_t, std::int32_t> target_of_seq;
  std::size_t checked = 0;
  for (const WireFrame& w : l.wire) {
    if (w.order <= start_order) continue;
    if (const auto a = rpdo1(w); a && a->node == 0)
      target_of_seq[a->pdo.command_seq] = a->pdo.target_position;
    if (const auto b = rpdo2(w); b && b->node == 0) {
      const auto t = target_of_seq.find(b->pdo.command_seq);
      ASSERT_NE(t, target_of_seq.end());
      const auto u = until_of_target.find(t->second);
      if (u == until_of_target.end()) continue;  // A hold command.
      EXPECT_LE(b->pdo.valid_until_us, wire_us(u->second));
      ++checked;
    }
  }
  EXPECT_GT(checked, 10U) << "INCONCLUSIVE: few approved commands dispatched";
}

TEST(Session, NoNewDeadlineWithoutNewApproval) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.3);
  stream(l, r, milliseconds(100));
  const ApprovedCommand last = *r.last;
  const std::uint64_t stop_order = l.wire.back().order;
  l.run(milliseconds(100));  // Approvals stop.
  std::optional<od::Rpdo2> first;
  std::optional<od::Rpdo1> first1;
  std::size_t repeats = 0;
  for (const WireFrame& w : l.wire) {
    if (w.order <= stop_order) continue;
    if (const auto a = rpdo1(w); a && a->node == 0) {
      if (!first1) first1 = a->pdo;
      EXPECT_EQ(a->pdo.command_seq, first1->command_seq);
      EXPECT_EQ(a->pdo.target_position, first1->target_position);
    }
    if (const auto b = rpdo2(w); b && b->node == 0) {
      if (!first) first = b->pdo;
      EXPECT_EQ(b->pdo.command_seq, first->command_seq) << "a new seq without an approval";
      EXPECT_EQ(b->pdo.valid_until_us, first->valid_until_us) << "a new deadline";
      // None once less than T_bus remains.
      EXPECT_GT(static_cast<std::int64_t>(b->pdo.valid_until_us) - wire_us(w.at), kTBus.ns / 1000);
      ++repeats;
    }
  }
  ASSERT_TRUE(first1) << "INCONCLUSIVE: nothing after the last approval";
  EXPECT_EQ(first1->target_position,
            static_cast<std::int32_t>(std::lround(last.position[0] * 1e6)));
  EXPECT_EQ(first->valid_until_us, wire_us(last.valid_until));
  EXPECT_GE(repeats, 2U) << "INCONCLUSIVE: no repeat";
}

TEST(Session, DriveStopsAfterApprovalsStop) {
  BusLoop l;
  bring_up_and_arm(l, kA);
  Ramp r = ramp_from_here(l, kA, 0.3);
  stream(l, r, milliseconds(100));
  const MonoTime stopped = l.now;
  l.run(milliseconds(200));
  for (std::size_t i = 0; i < kNodes; ++i) {
    std::optional<drive_core::DriveEvent> qs;
    for (const DriveRecord& e : l.drive_events) {
      if (e.node == i && e.event.kind == DriveEventKind::kQuickStop && e.event.at > stopped) {
        qs = e.event;
        break;
      }
    }
    ASSERT_TRUE(qs) << "node " << i << " kept going";
    EXPECT_TRUE(qs->reason == drive_core::QuickStopReason::kExpiry ||
                qs->reason == drive_core::QuickStopReason::kTimeout)
        << i;
    EXPECT_EQ(l.drives[i]->cia402().state(), Cia402State::kQuickStopActive) << i;
  }
}

// ============================ feedback join ============================

constexpr Duration kSkew{12'000'000};  // T_bus + W

void feed(FeedbackJoin& j, std::size_t node, std::uint16_t seq, MonoTime sampled, MonoTime rx,
          std::int32_t position = 0) {
  j.on_tpdo(node, od::Tpdo1{0x0237, position, seq}, rx);
  j.on_tpdo(node, od::Tpdo2{0, seq, 0}, rx);
  j.on_tpdo(node, od::Tpdo3{wire_us(sampled), seq, 0}, rx);
}

TEST(FeedbackJoin, IncompleteSampleIsNotPublished) {
  FeedbackJoin j(kSkew);
  const MonoTime t = kT0;
  for (std::size_t n = 1; n < kNodes; ++n) feed(j, n, 1, t, t);
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo1{0x0237, 5, 1}, t));
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo2{0, 1, 0}, t));
  EXPECT_FALSE(j.node(0));
  EXPECT_FALSE(j.latest()) << "a node with two of three TPDOs";
  EXPECT_TRUE(j.on_tpdo(0, od::Tpdo3{wire_us(t), 1, 0}, t));
  EXPECT_TRUE(j.latest());
}

TEST(FeedbackJoin, StaggeredStartsJoinPerNode) {
  FeedbackJoin j(kSkew);
  const MonoTime t = kT0;
  // Each node started on a different SYNC: its sample_seq differs from the others'.
  for (std::size_t n = 0; n < kNodes; ++n) {
    feed(j, n, static_cast<std::uint16_t>(100 + 37 * n), t, t, static_cast<std::int32_t>(n * 1000));
  }
  const auto f = j.latest();
  ASSERT_TRUE(f);
  for (std::size_t n = 0; n < kNodes; ++n) {
    EXPECT_DOUBLE_EQ(f->position[n], static_cast<double>(n) * 1e-3);
    EXPECT_EQ(j.node(n)->sample_seq, 100 + 37 * n);
  }
}

TEST(FeedbackJoin, SkewedSnapshotIsStale) {
  FeedbackJoin j(kSkew);
  const MonoTime t = kT0 + milliseconds(100);
  for (std::size_t n = 0; n < kNodes; ++n) feed(j, n, 1, t, t);
  ASSERT_FALSE(j.latest()->skewed);
  feed(j, 4, 2, t - kSkew, t);  // Exactly T_bus + W apart: not skewed.
  ASSERT_TRUE(j.latest());
  EXPECT_FALSE(j.latest()->skewed);
  feed(j, 4, 3, t - kSkew - microseconds(1), t);
  ASSERT_TRUE(j.latest());
  EXPECT_TRUE(j.latest()->skewed);
}

TEST(FeedbackJoin, BootUpResetsNodeJoin) {
  FeedbackJoin j(kSkew);
  const MonoTime t = kT0;
  for (std::size_t n = 0; n < kNodes; ++n) feed(j, n, 50, t, t);
  j.on_tpdo(2, od::Tpdo1{0x0237, 1, 51}, t);  // A partial of the old boot.
  ASSERT_TRUE(j.latest());
  j.on_boot(2);
  EXPECT_FALSE(j.node(2));
  EXPECT_FALSE(j.latest());
  // The partial does not complete with the new boot's TPDOs; seqs start over after the boot.
  EXPECT_FALSE(j.on_tpdo(2, od::Tpdo2{0, 51, 0}, t));
  EXPECT_FALSE(j.on_tpdo(2, od::Tpdo3{wire_us(t), 51, 0}, t)) << "joined across a boot-up";
  feed(j, 2, 1, t, t);
  ASSERT_TRUE(j.node(2));
  EXPECT_EQ(j.node(2)->sample_seq, 1);
  EXPECT_TRUE(j.latest());
}

TEST(FeedbackJoin, ReorderedTpdosJoin) {
  FeedbackJoin j(kSkew);
  const MonoTime t = kT0;
  // Node 0: seq 8 and 9 interleaved and reversed.
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo3{wire_us(t + milliseconds(10)), 9, 4}, t));
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo2{-20, 8, 3}, t));
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo1{0x0237, 900, 9}, t));
  EXPECT_FALSE(j.on_tpdo(0, od::Tpdo3{wire_us(t), 8, 4}, t));
  const auto s8 = j.on_tpdo(0, od::Tpdo1{0x0237, 800, 8}, t);
  ASSERT_TRUE(s8);
  EXPECT_EQ(s8->position_urad, 800);
  EXPECT_EQ(s8->velocity_urad_s, -20);
  EXPECT_EQ(s8->applied_seq, 3);
  const auto s9 = j.on_tpdo(0, od::Tpdo2{-10, 9, 5}, t + milliseconds(10));
  ASSERT_TRUE(s9);
  EXPECT_EQ(s9->position_urad, 900);
  EXPECT_EQ(s9->velocity_urad_s, -10);
  EXPECT_EQ(s9->applied_tag, 4);
  EXPECT_EQ(s9->sampled_at, t + milliseconds(10));
  EXPECT_EQ(j.node(0)->sample_seq, 9);
}

TEST(FeedbackJoin, SampleTimeUnwrapsAcrossWrap) {
  // A host time just past a 2^32 us boundary; the sample was taken just before it.
  const std::int64_t wrap_us = std::int64_t{1} << 33;  // 2 x 2^32 us
  const MonoTime rx{(wrap_us + 300) * 1000};
  const MonoTime sampled{(wrap_us - 700) * 1000};
  EXPECT_EQ(canopen_master::unwrap_us(static_cast<std::uint32_t>(wrap_us - 700), rx), sampled);
  EXPECT_GT(wire_us(sampled), wire_us(rx)) << "INCONCLUSIVE: the stamps did not wrap";
  FeedbackJoin j(kSkew);
  for (std::size_t n = 0; n < kNodes; ++n) feed(j, n, 1, sampled, rx);
  ASSERT_TRUE(j.latest());
  EXPECT_EQ(j.latest()->sampled_at, sampled);
  // And a stamp slightly ahead of the reference is not taken as 71 minutes old.
  EXPECT_EQ(canopen_master::unwrap_us(wire_us(rx + microseconds(5)), rx), rx + microseconds(5));
}

TEST(FeedbackJoin, OldestSourceTimeIsSnapshotTime) {
  FeedbackJoin j(kSkew);
  const MonoTime rx = kT0 + milliseconds(500);
  for (std::size_t n = 0; n < kNodes; ++n) {
    feed(j, n, 1, rx - milliseconds(static_cast<std::int64_t>(n == 3 ? 9 : 2)), rx);
  }
  const auto f = j.latest();
  ASSERT_TRUE(f);
  EXPECT_EQ(f->sampled_at, rx - milliseconds(9)) << "the oldest source time, never rx";
  EXPECT_EQ(f->node_sampled_at[3], rx - milliseconds(9));
  EXPECT_EQ(f->node_sampled_at[0], rx - milliseconds(2));
  EXPECT_FALSE(f->skewed);
}

}  // namespace
