// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "canopen_backend/canopen_backend.hpp"
#include "canopen_codec/codec.hpp"
#include "canopen_master/testing/drive_bus.hpp"
#include "canopen_master/vdrive_session.hpp"
#include "drive_core/cia402.hpp"
#include "drive_core/drive.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "rig.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/time.hpp"

// CanopenBackend in logic time against six in-memory drive_core::Drive (the shared DriveBus).
// The probe and its trust boundary (C2), confirmation by behaviour, the heartbeat fault and the
// configure-while-armed refusal. Expectations come from the drives and the wire, and from the
// backend's own reports where those are the subject.
namespace {

namespace cc = canopen_codec;
namespace cm = canopen_master;
namespace cmt = canopen_master::testing;
namespace hc = hal_contract;
namespace rc = runtime_core;
using canopen_backend::CanopenBackend;
using canopen_backend::CanopenConfig;
using canopen_backend::testing::kNodes;
using canopen_backend::testing::kW;
using canopen_backend::testing::Rig;
using canopen_backend::testing::WireFrame;
using hc::OpState;
using rc::milliseconds;

const hc::AuthorityId kA{{3}, {5}, {7}};

void expect_profile(const rc::DeviceCapabilities& got, const rc::DeviceCapabilities& want) {
  EXPECT_EQ(got.device_side_expiry, want.device_side_expiry);
  EXPECT_EQ(got.execution_generation, want.execution_generation);
  EXPECT_EQ(got.local_stop, want.local_stop);
  EXPECT_EQ(got.apply_ack, want.apply_ack);
  EXPECT_EQ(got.source_timestamp, want.source_timestamp);
}

// The six drives powered on, the backend has seen their boot-ups.
std::unique_ptr<CanopenBackend> start(Rig& rig, CanopenConfig config = Rig::default_config()) {
  std::unique_ptr<CanopenBackend> b = rig.make_backend(std::move(config));
  rig.power_on();
  rig.advance(kW);
  return b;
}

bool is_nmt(const WireFrame& w, std::optional<cc::NmtCommand> command = std::nullopt) {
  const std::optional<cc::NmtMessage> m = w.from_master ? cc::decode_nmt(w.frame) : std::nullopt;
  return m && (!command || m->command == *command);
}

// Two approvals per period at a constant target; returns the last valid_until.
rc::MonoTime stream(Rig& rig, CanopenBackend& b, const hc::AuthorityId& id, std::uint64_t& seq,
                    double target, int periods, rc::Duration validity) {
  rc::MonoTime until{};
  for (int k = 0; k < periods; ++k) {
    hc::Joints p{};
    p.fill(target);
    until = rig.now + validity;
    b.submit(hc::ApprovedCommand{id, ++seq, p, rig.now, until}, rig.now);
    rig.advance(milliseconds(10));
  }
  return until;
}

TEST(CanopenBackend, ProbeGivesVirtualProfile) {
  Rig rig;
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  expect_profile(b->capabilities(), rc::virtual_canopen_profile());
  // Provisional (C2): trusted for the identity, not yet confirmed by behaviour.
  EXPECT_FALSE(b->session().profile_confirmed());
  const cm::Identity eds = cm::eds_identity();
  for (std::size_t i = 0; i < kNodes; ++i) {
    const cm::NodeView n = b->session().node(i);
    EXPECT_EQ(n.profile, cm::NodeProfile::kVirtual) << i;
    EXPECT_EQ(n.probe.identity[0], eds.vendor) << i;
    EXPECT_EQ(n.probe.identity[1], eds.product) << i;
    EXPECT_EQ(n.probe.identity[2], eds.revision) << i;
    EXPECT_EQ(n.probe.mode_display, 8U) << i;
  }
  // The first arm confirms it.
  ASSERT_EQ(rig.activate(), OpState::kSucceeded);
  ASSERT_EQ(rig.arm(kA, milliseconds(500)), OpState::kSucceeded);
  EXPECT_TRUE(b->session().profile_confirmed());
  expect_profile(b->capabilities(), rc::virtual_canopen_profile());
  EXPECT_FALSE(b->inhibited());
}

TEST(CanopenBackend, NodeWithoutTagGivesGenericProfile) {
  Rig rig;
  rig.bus.intercept[3] = cmt::without_object(3, 0x2012);
  std::unique_ptr<CanopenBackend> owned = rig.make_backend();
  CanopenBackend* b = owned.get();
  rc::Requirements needs{};
  needs.device_side_expiry = true;
  rc::JointLimits<hc::kJoints> limits{};
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    limits.pos_min[i] = -3.0;
    limits.pos_max[i] = 3.0;
    limits.vel_max[i] = 1.0;
    limits.acc_max[i] = 100.0;
  }
  hc::Hal hal(std::move(owned), needs, rc::GuardConfig{milliseconds(10), milliseconds(20)}, limits);
  rig.power_on();
  rig.advance(kW);
  const hc::OpHandle c = hal.configure(rig.now);
  rig.advance_until([&] { return hal.op_state(c) != OpState::kPending; }, hc::kConfigureTimeout);
  ASSERT_EQ(hal.op_state(c), OpState::kSucceeded) << "INCONCLUSIVE: configure failed";
  EXPECT_EQ(b->session().node(3).profile, cm::NodeProfile::kGeneric);
  expect_profile(b->capabilities(), rc::generic_cia402_profile());

  const hc::OpHandle a = hal.activate(rig.now);
  EXPECT_EQ(hal.op_state(a), OpState::kFailed);
  const std::optional<hc::OpResult> r = hal.wait(a, {});
  ASSERT_TRUE(r.has_value());
  bool expiry_missing = false;
  for (const rc::Missing& m : r->missing) {
    if (m.capability == rc::Capability::kDeviceSideExpiry) expiry_missing = true;
  }
  EXPECT_TRUE(expiry_missing);
  rig.advance(milliseconds(200));
  for (const WireFrame& w : rig.wire) {
    EXPECT_FALSE(is_nmt(w, cc::NmtCommand::kStart)) << "an NMT Start after a refused activation";
  }
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_NE(rig.bus.drives[i]->nmt().state(), cc::NmtState::kOperational) << i;
  }
}

TEST(CanopenBackend, ProfileParameterCannotRaiseProbe) {
  {  // A generic probe stays generic under a virtual limit.
    Rig rig;
    rig.bus.intercept[1] = cmt::without_object(1, 0x2012);
    CanopenConfig c = Rig::default_config();
    c.profile_limit = rc::virtual_canopen_profile();
    std::unique_ptr<CanopenBackend> b = start(rig, c);
    ASSERT_EQ(rig.configure(), OpState::kSucceeded);
    expect_profile(b->capabilities(), rc::generic_cia402_profile());
  }
  {  // A limit stronger than every probe value raises nothing either.
    Rig rig;
    CanopenConfig c = Rig::default_config();
    c.profile_limit = rc::DeviceCapabilities{rc::Support::kSupported, rc::Support::kSupported,
                                             rc::Support::kSupported, rc::Support::kSupported,
                                             rc::Support::kSupported};
    std::unique_ptr<CanopenBackend> b = start(rig, c);
    ASSERT_EQ(rig.configure(), OpState::kSucceeded);
    expect_profile(b->capabilities(), rc::virtual_canopen_profile());
  }
  {  // A virtual probe is lowered by a generic limit.
    Rig rig;
    CanopenConfig c = Rig::default_config();
    c.profile_limit = rc::generic_cia402_profile();
    std::unique_ptr<CanopenBackend> b = start(rig, c);
    ASSERT_EQ(rig.configure(), OpState::kSucceeded);
    ASSERT_EQ(b->session().node(0).profile, cm::NodeProfile::kVirtual)
        << "INCONCLUSIVE: the probe itself was not virtual";
    expect_profile(b->capabilities(), rc::generic_cia402_profile());
  }
}

TEST(CanopenBackend, WrongModeGivesGenericProfile) {
  Rig rig;
  rig.bus.intercept[2] = cmt::reports(2, 0x6061, 0, 1, 1);  // profile position, not CSP (8)
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  EXPECT_EQ(b->session().node(2).probe.mode_display, 1U);
  EXPECT_EQ(b->session().node(2).profile, cm::NodeProfile::kGeneric);
  EXPECT_EQ(b->session().node(0).profile, cm::NodeProfile::kVirtual);
  expect_profile(b->capabilities(), rc::generic_cia402_profile());
}

TEST(CanopenBackend, IdentityMismatchGivesGeneric) {
  Rig rig;
  const cm::Identity eds = cm::eds_identity();
  rig.bus.intercept[4] = cmt::reports(4, 0x1018, 3, eds.revision + 1U);
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  EXPECT_EQ(b->session().node(4).probe.identity[2], eds.revision + 1U);
  EXPECT_EQ(b->session().node(4).profile, cm::NodeProfile::kGeneric);
  expect_profile(b->capabilities(), rc::generic_cia402_profile());
}

// C2 negative peer: answers the probe, reaches Switched On and commits, but its TPDO3 never
// reports the armed tag.
TEST(CanopenBackend, NoTpdoAckFailsConfirmation) {
  Rig rig;
  rig.rewrite = [](bool from_master, const cc::Frame& f) -> std::optional<cc::Frame> {
    return from_master ? std::nullopt : cmt::without_applied_tag(3, f);
  };
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  expect_profile(b->capabilities(), rc::virtual_canopen_profile());
  ASSERT_EQ(rig.activate(), OpState::kSucceeded) << "INCONCLUSIVE: never reached Switched On";
  const rc::MonoTime posted = rig.now;
  EXPECT_EQ(rig.arm(kA, milliseconds(500)), OpState::kFailed);
  ASSERT_NE(rig.bus.drives[3]->committed_attempt(), 0U) << "INCONCLUSIVE: node 4 never committed";
  EXPECT_LT((rig.now - posted).ns, milliseconds(200).ns) << "not failed by the 4 x T_bus window";
  EXPECT_TRUE(b->inhibited());
  EXPECT_TRUE(b->poll(rig.now).faults.has(hc::Fault::kArmFailed));
  EXPECT_FALSE(b->session().profile_confirmed());
  expect_profile(b->capabilities(), rc::generic_cia402_profile());
}

// C2: the virtual profile is trusted for the identity. A peer that claims our identity and
// acknowledges normally but ignores expiry is not detected: this documents the boundary rather
// than pretending to detect it. The same peer with another identity is generic.
TEST(CanopenBackend, IdentityIsTheTrustBoundary) {
  // Every node rewrites each received deadline to 45 ms after receipt (inside the 48 ms lease, past
  // its own 30 ms command timeout): it never expires a command by its deadline.
  const auto ignore_expiry = [](Rig& rig) {
    rig.rewrite = [&rig](bool from_master, const cc::Frame& f) -> std::optional<cc::Frame> {
      if (!from_master) return std::nullopt;
      for (std::size_t i = 0; i < kNodes; ++i) {
        if (std::optional<cc::od::Rpdo2> p = cc::unpack_rpdo2(cmt::node_id(i), f)) {
          p->valid_until_us = static_cast<std::uint32_t>(rig.now.ns / 1000 + 45'000);
          return cc::pack_rpdo2(cmt::node_id(i), *p);
        }
      }
      return std::nullopt;
    };
  };
  Rig rig;
  ignore_expiry(rig);
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  ASSERT_EQ(rig.activate(), OpState::kSucceeded);
  ASSERT_EQ(rig.arm(kA, milliseconds(500)), OpState::kSucceeded);
  EXPECT_TRUE(b->session().profile_confirmed()) << "the peer passes confirmation";
  expect_profile(b->capabilities(), rc::virtual_canopen_profile());

  std::uint64_t seq = 0;
  stream(rig, *b, kA, seq, 0.01, 5, milliseconds(48));
  const rc::MonoTime host_deadline = stream(rig, *b, kA, seq, 0.02, 1, milliseconds(28));
  rig.advance(milliseconds(100));
  // The host's deadline passed with every drive still enabled; they stopped only on the command
  // timeout, which this peer does not rewrite.
  std::size_t stops = 0;
  for (const cmt::DriveRecord& r : rig.bus.drive_events) {
    if (r.event.kind != drive_core::DriveEventKind::kQuickStop) continue;
    ++stops;
    EXPECT_EQ(r.event.reason, drive_core::QuickStopReason::kTimeout) << "node " << r.node;
    EXPECT_GT(r.event.at, host_deadline) << "node " << r.node;
  }
  EXPECT_EQ(stops, kNodes) << "INCONCLUSIVE: not every drive stopped on its own";
  expect_profile(b->capabilities(), rc::virtual_canopen_profile());

  // The same behaviour under another identity is not trusted.
  Rig other;
  ignore_expiry(other);
  other.bus.intercept[0] = cmt::reports(0, 0x1018, 1, cm::eds_identity().vendor + 1U);
  std::unique_ptr<CanopenBackend> ob = start(other);
  ASSERT_EQ(other.configure(), OpState::kSucceeded);
  expect_profile(ob->capabilities(), rc::generic_cia402_profile());
}

TEST(CanopenBackend, HeartbeatLossIsNodeLostFault) {
  Rig rig;
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  ASSERT_EQ(rig.activate(), OpState::kSucceeded);
  ASSERT_EQ(rig.arm(kA, milliseconds(800)), OpState::kSucceeded);
  ASSERT_FALSE(b->poll(rig.now).faults.any()) << "INCONCLUSIVE: a fault before the loss";
  const std::size_t mark = rig.wire.size();
  const rc::MonoTime muted = rig.now;
  rig.bus.heartbeat_muted[2] = true;
  rig.advance_until([&] { return b->poll(rig.now).faults.has(hc::Fault::kNodeLost); },
                    milliseconds(500));
  const hc::Feedback fb = b->poll(rig.now);
  ASSERT_TRUE(fb.faults.has(hc::Fault::kNodeLost));
  // The consumer time is 250 ms after the last heartbeat (100 ms period).
  EXPECT_GE((rig.now - muted).ns, milliseconds(150).ns);
  EXPECT_LE((rig.now - muted).ns, milliseconds(352).ns);
  EXPECT_TRUE(fb.faults.has(hc::Fault::kInhibited));
  EXPECT_TRUE(b->inhibited());
  rig.advance(milliseconds(300));
  // The master raises the fault; it never resets or stops the node (roadmap boot policy).
  for (std::size_t k = mark; k < rig.wire.size(); ++k) {
    EXPECT_FALSE(is_nmt(rig.wire[k])) << "an NMT command after the heartbeat loss";
  }
  EXPECT_EQ(rig.bus.drives[2]->nmt().state(), cc::NmtState::kOperational);
}

// Carried from Task 4: configure while armed (and not inhibited) is refused, since its Enter
// Pre-operational would stop enabled drives without tracking the stops.
TEST(CanopenBackend, RefusesConfigureWhileArmed) {
  Rig rig;
  std::unique_ptr<CanopenBackend> b = start(rig);
  ASSERT_EQ(rig.configure(), OpState::kSucceeded);
  ASSERT_EQ(rig.activate(), OpState::kSucceeded);
  ASSERT_EQ(rig.arm(kA, milliseconds(800)), OpState::kSucceeded);
  const std::size_t mark = rig.wire.size();
  const hc::OpTicket t = rig.ticket(hc::kConfigureTimeout);
  b->configure(t, rig.now);
  EXPECT_EQ(b->op_state(t.op_id), OpState::kFailed);
  const std::optional<hc::OpResult> r = b->wait(t.op_id, {});
  ASSERT_TRUE(r.has_value());
  EXPECT_NE(r->error.find("armed"), std::string::npos) << r->error;
  rig.advance(milliseconds(50));
  for (std::size_t k = mark; k < rig.wire.size(); ++k) {
    EXPECT_FALSE(is_nmt(rig.wire[k])) << "the refused configure sent NMT";
  }
  EXPECT_EQ(b->session().phase(), cm::SessionPhase::kArmed);
  EXPECT_FALSE(b->inhibited());
  for (std::size_t i = 0; i < kNodes; ++i) {
    EXPECT_EQ(rig.bus.drives[i]->cia402().state(), drive_core::Cia402State::kOperationEnabled) << i;
  }
  // After a deactivate, configure goes ahead.
  ASSERT_EQ(rig.deactivate(), OpState::kSucceeded);
  EXPECT_EQ(rig.configure(), OpState::kSucceeded);
}

}  // namespace
