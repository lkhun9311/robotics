// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <span>
#include <stdexcept>
#include <variant>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "canopen_master/events.hpp"
#include "canopen_master/heartbeat.hpp"
#include "canopen_master/master.hpp"
#include "canopen_master/sdo_client.hpp"
#include "runtime_core/time.hpp"

// The master core driven only through frames and logic time. Every expected frame is written
// here as CiA 301 bytes, never produced by canopen_codec's encoders (the code under test uses
// them). Ticks come before the frames of the same instant.
namespace {

namespace cc = canopen_codec;
using canopen_master::HeartbeatStatus;
using canopen_master::Master;
using canopen_master::MasterConfig;
using canopen_master::MasterEvent;
using canopen_master::MasterEventKind;
using canopen_master::SdoResult;
using canopen_master::SdoStatus;
using runtime_core::Duration;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

const cc::NodeId kNode2 = cc::NodeId::make(2).value();
const cc::NodeId kNode3 = cc::NodeId::make(3).value();
const cc::NodeId kNode5 = cc::NodeId::make(5).value();
constexpr MonoTime kT0{5'000'000'000};

cc::Frame frame(std::uint16_t cob, std::initializer_list<std::uint8_t> bytes) {
  cc::Frame f{cob, static_cast<std::uint8_t>(bytes.size()), {}};
  std::copy(bytes.begin(), bytes.end(), f.data.begin());
  return f;
}

MonoTime at(std::int64_t ms) { return kT0 + milliseconds(ms); }

// SYNC production off unless a test asks for it, so the other services' frames stand alone.
MasterConfig config(std::vector<cc::NodeId> nodes, Duration sync_period = Duration{}) {
  MasterConfig c;
  c.nodes = std::move(nodes);
  c.sync_period = sync_period;
  return c;
}

constexpr Duration kSyncPeriod{10'000'000};

std::vector<MasterEvent> of_kind(const std::vector<MasterEvent>& events, MasterEventKind kind) {
  std::vector<MasterEvent> out;
  std::copy_if(events.begin(), events.end(), std::back_inserter(out),
               [&](const MasterEvent& e) { return e.kind == kind; });
  return out;
}

bool has_cob(const std::vector<cc::Frame>& frames, std::uint16_t cob) {
  return std::any_of(frames.begin(), frames.end(),
                     [&](const cc::Frame& f) { return f.cob_id == cob; });
}

void append(std::vector<cc::Frame>& to, const std::vector<cc::Frame>& from) {
  to.insert(to.end(), from.begin(), from.end());
}

// ---- NMT ----

TEST(Master, NmtFramesExact) {
  Master m(config({kNode2, kNode3}));
  EXPECT_TRUE(m.nmt(cc::NmtCommand::kStart, kNode2));
  EXPECT_TRUE(m.nmt(cc::NmtCommand::kStop, kNode2));
  EXPECT_TRUE(m.nmt(cc::NmtCommand::kEnterPreOperational, kNode2));
  EXPECT_TRUE(m.nmt(cc::NmtCommand::kResetCommunication, kNode2));
  EXPECT_TRUE(m.nmt(cc::NmtCommand::kResetNode, kNode3));
  EXPECT_FALSE(m.nmt(cc::NmtCommand::kStart, kNode5));  // Not configured: never sent.

  // CiA 301 NMT node control: COB-ID 0x000, DLC 2, command specifier then node-ID.
  const std::vector<cc::Frame> expected{
      frame(0x000, {0x01, 0x02}), frame(0x000, {0x02, 0x02}), frame(0x000, {0x80, 0x02}),
      frame(0x000, {0x82, 0x02}), frame(0x000, {0x81, 0x03}),
  };
  EXPECT_EQ(m.tick(at(0)), expected);
  EXPECT_TRUE(m.tick(at(1)).empty());  // Sent once.
}

// ---- Heartbeat consumer (C7) ----

TEST(Master, HeartbeatLossAt250msAndSendsNothing) {
  Master m(config({kNode2}));
  std::vector<cc::Frame> out;
  append(out, m.tick(at(0)));
  // Node 2 keeps sending a TPDO and an unknown frame after its last heartbeat: neither is a
  // heartbeat, so neither may keep the consumer alive.
  // Loss at 260 ms; the loop runs to loss + 2 s inclusive.
  for (std::int64_t ms = 1; ms <= 2260; ++ms) {
    append(out, m.tick(at(ms)));
    if (ms == 10) {
      append(out, m.on_frame(frame(0x702, {0x05}), at(ms)));  // The last heartbeat.
    }
    if (ms % 50 == 0) {
      append(out, m.on_frame(frame(0x182, {1, 2, 3, 4, 5, 6, 7, 8}), at(ms)));
      append(out, m.on_frame(frame(0x282, {0}), at(ms)));
    }
    if (ms == 259) {
      EXPECT_TRUE(of_kind(m.drain_events(), MasterEventKind::kHeartbeatLost).empty());
      EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAlive);
    }
  }
  const auto events = m.drain_events();
  const auto lost = of_kind(events, MasterEventKind::kHeartbeatLost);
  ASSERT_EQ(lost.size(), 1U);
  EXPECT_EQ(lost[0].node, 2);
  EXPECT_EQ(lost[0].at, at(260));  // Last heartbeat at 10 ms + 250 ms; equality is expired.
  EXPECT_TRUE(of_kind(events, MasterEventKind::kBootTimeout).empty());
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kLost);
  EXPECT_FALSE(has_cob(out, 0x000));  // No NMT command in reaction, from start to loss + 2 s.
  EXPECT_EQ(m.nmt_state(kNode2), cc::NmtState::kOperational);

  m.on_frame(frame(0x702, {0x05}), at(2300));
  EXPECT_EQ(of_kind(m.drain_events(), MasterEventKind::kHeartbeatResumed).size(), 1U);
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAlive);
}

TEST(Master, BootTimeoutDistinctFromRunningTimeout) {
  Master m(config({kNode2, kNode3}));
  std::vector<cc::Frame> out;
  append(out, m.tick(at(0)));  // The consumers start here.
  for (std::int64_t ms = 1; ms <= 1100; ++ms) {
    append(out, m.tick(at(ms)));
    if (ms == 100) {
      // Node 3 boots and then falls silent; node 2 never sends anything.
      append(out, m.on_frame(frame(0x703, {0x00}), at(ms)));
    }
    if (ms == 999) {
      EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAwaitingBoot);
    }
  }
  const auto events = m.drain_events();
  const auto boot = of_kind(events, MasterEventKind::kBootTimeout);
  ASSERT_EQ(boot.size(), 1U);
  EXPECT_EQ(boot[0].node, 2);
  EXPECT_EQ(boot[0].at, at(1000));
  const auto lost = of_kind(events, MasterEventKind::kHeartbeatLost);
  ASSERT_EQ(lost.size(), 1U);
  EXPECT_EQ(lost[0].node, 3);
  EXPECT_EQ(lost[0].at, at(350));
  ASSERT_EQ(of_kind(events, MasterEventKind::kBoot).size(), 1U);
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kBootTimedOut);
  EXPECT_FALSE(has_cob(out, 0x000));

  // Our Reset Communication restarts node 2's boot window from when it is sent.
  ASSERT_TRUE(m.nmt(cc::NmtCommand::kResetCommunication, kNode2));
  EXPECT_EQ(m.tick(at(1200)), std::vector<cc::Frame>{frame(0x000, {0x82, 0x02})});
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAwaitingBoot);
  m.tick(at(2199));
  EXPECT_TRUE(of_kind(m.drain_events(), MasterEventKind::kBootTimeout).empty());
  m.tick(at(2200));
  const auto again = of_kind(m.drain_events(), MasterEventKind::kBootTimeout);
  ASSERT_EQ(again.size(), 1U);
  EXPECT_EQ(again[0].node, 2);

  // A first heartbeat after a boot timeout is reported as resumed.
  m.on_frame(frame(0x702, {0x7F}), at(2300));
  const auto resumed = m.drain_events();
  EXPECT_EQ(of_kind(resumed, MasterEventKind::kHeartbeatResumed).size(), 1U);
  EXPECT_EQ(of_kind(resumed, MasterEventKind::kState).size(), 1U);
}

TEST(Master, ConsumerTimePerNodeAndZeroDisables) {
  Master m(config({kNode2, kNode3}));
  m.tick(at(0));
  ASSERT_TRUE(m.set_heartbeat_consumer(kNode2, Duration{}));  // N3 rewrites 0x1017 meanwhile.
  ASSERT_TRUE(m.set_heartbeat_consumer(kNode3, milliseconds(500)));
  EXPECT_FALSE(m.set_heartbeat_consumer(kNode5, milliseconds(500)));
  m.on_frame(frame(0x702, {0x7F}), at(10));
  m.on_frame(frame(0x703, {0x7F}), at(10));
  for (std::int64_t ms = 11; ms <= 3000; ++ms) {
    m.tick(at(ms));
  }
  auto events = m.drain_events();
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kDisabled);
  for (const MasterEvent& e : events) {
    EXPECT_FALSE(e.node == 2 && (e.kind == MasterEventKind::kHeartbeatLost ||
                                 e.kind == MasterEventKind::kBootTimeout));
  }
  const auto lost = of_kind(events, MasterEventKind::kHeartbeatLost);
  ASSERT_EQ(lost.size(), 1U);
  EXPECT_EQ(lost[0].node, 3);
  EXPECT_EQ(lost[0].at, at(510));
  // The disabled consumer still tracks the state.
  EXPECT_EQ(m.nmt_state(kNode2), cc::NmtState::kPreOperational);

  // Re-enabled: the consumer starts at that moment and waits for a first heartbeat.
  ASSERT_TRUE(m.set_heartbeat_consumer(kNode2, milliseconds(250)));
  m.tick(at(4000));
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAwaitingBoot);
  m.tick(at(4999));
  EXPECT_TRUE(m.drain_events().empty());
  m.tick(at(5000));
  const auto boot = of_kind(m.drain_events(), MasterEventKind::kBootTimeout);
  ASSERT_EQ(boot.size(), 1U);
  EXPECT_EQ(boot[0].node, 2);
}

TEST(Master, PreResetHeartbeatIsNotTheFirstAfterReset) {
  // A heartbeat already received when on_frame is called predates a Reset Communication queued
  // before that call: it must not satisfy the boot window the reset opens (C7).
  Master m(config({kNode2}));
  m.tick(at(0));
  m.on_frame(frame(0x702, {0x05}), at(10));
  ASSERT_TRUE(m.nmt(cc::NmtCommand::kResetCommunication, kNode2));
  EXPECT_EQ(m.on_frame(frame(0x702, {0x05}), at(20)),
            std::vector<cc::Frame>{frame(0x000, {0x82, 0x02})});
  EXPECT_EQ(m.heartbeat_status(kNode2), HeartbeatStatus::kAwaitingBoot);
  EXPECT_EQ(m.nmt_state(kNode2), std::nullopt);
  m.drain_events();
  for (std::int64_t ms = 21; ms <= 1100; ++ms) {
    m.tick(at(ms));
  }
  const auto events = m.drain_events();
  EXPECT_TRUE(of_kind(events, MasterEventKind::kHeartbeatLost).empty());
  const auto boot = of_kind(events, MasterEventKind::kBootTimeout);
  ASSERT_EQ(boot.size(), 1U);
  EXPECT_EQ(boot[0].at, at(1020));
}

// ---- Expedited SDO client (C8) ----

struct Recorder {
  std::vector<SdoResult> results;
  canopen_master::SdoDone done() {
    return [this](const SdoResult& r) { results.push_back(r); };
  }
};

TEST(Master, SdoOneOutstandingPerNodeFifo) {
  Master m(config({kNode2, kNode3}));
  Recorder rec;
  const std::array<std::uint8_t, 2> hb{0xE8, 0x03};
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1018, 0x01, rec.done()));
  ASSERT_TRUE(m.sdo_download(kNode2, 0x1017, 0x00, hb, rec.done()));
  ASSERT_TRUE(m.sdo_upload(kNode3, 0x1000, 0x00, rec.done()));

  // One request per node on the wire.
  EXPECT_EQ(m.tick(at(0)), (std::vector<cc::Frame>{
                               frame(0x602, {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0}),
                               frame(0x603, {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0}),
                           }));
  EXPECT_TRUE(m.tick(at(1)).empty());

  // Expedited upload response, 4 bytes (0x43), value 0x00020192.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00}), at(5)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x18, 0x10, 0x01, 0, 0, 0, 0})});
  // 1-byte response (0x4F): only the indicated byte counts.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x4F, 0x18, 0x10, 0x01, 0x07, 0xAA, 0xBB, 0xCC}), at(6)),
            std::vector<cc::Frame>{frame(0x602, {0x2B, 0x17, 0x10, 0x00, 0xE8, 0x03, 0, 0})});
  EXPECT_TRUE(m.on_frame(frame(0x582, {0x60, 0x17, 0x10, 0x00, 0, 0, 0, 0}), at(7)).empty());
  EXPECT_TRUE(
      m.on_frame(frame(0x583, {0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00}), at(8)).empty());

  ASSERT_EQ(rec.results.size(), 4U);
  EXPECT_EQ(rec.results[0], (SdoResult{2, 0x1000, 0x00, SdoStatus::kOk, 4, 0x00020192, 0}));
  EXPECT_EQ(rec.results[1], (SdoResult{2, 0x1018, 0x01, SdoStatus::kOk, 1, 0x07, 0}));
  EXPECT_EQ(rec.results[2], (SdoResult{2, 0x1017, 0x00, SdoStatus::kOk, 0, 0, 0}));
  EXPECT_EQ(rec.results[3], (SdoResult{3, 0x1000, 0x00, SdoStatus::kOk, 4, 0x00020192, 0}));
  EXPECT_EQ(of_kind(m.drain_events(), MasterEventKind::kSdoOk).size(), 4U);

  // A response with nothing outstanding is counted and ignored.
  EXPECT_TRUE(m.on_frame(frame(0x582, {0x60, 0x17, 0x10, 0x00, 0, 0, 0, 0}), at(9)).empty());
  EXPECT_EQ(m.counts().sdo_unexpected, 1U);
  EXPECT_EQ(rec.results.size(), 4U);
}

TEST(Master, SdoTimeoutAt200ms) {
  Master m(config({kNode2}));
  Recorder rec;
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1001, 0x00, rec.done()));
  EXPECT_EQ(m.tick(at(0)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0})});
  EXPECT_TRUE(m.tick(kT0 + Duration{199'999'999}).empty());
  EXPECT_TRUE(rec.results.empty());

  // At 200 ms: abort 0x05040000 (SDO protocol timed out) to the server, then the next request.
  EXPECT_EQ(m.tick(at(200)), (std::vector<cc::Frame>{
                                 frame(0x602, {0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x05}),
                                 frame(0x602, {0x40, 0x01, 0x10, 0x00, 0, 0, 0, 0}),
                             }));
  ASSERT_EQ(rec.results.size(), 1U);
  EXPECT_EQ(rec.results[0], (SdoResult{2, 0x1000, 0x00, SdoStatus::kTimeout, 0, 0, 0x05040000}));
  const auto timeouts = of_kind(m.drain_events(), MasterEventKind::kSdoTimeout);
  ASSERT_EQ(timeouts.size(), 1U);
  EXPECT_EQ(timeouts[0].at, at(200));

  // A response arriving exactly at the deadline is too late: the request has timed out first.
  const auto out = m.on_frame(frame(0x582, {0x4F, 0x01, 0x10, 0x00, 0x00, 0, 0, 0}), at(400));
  EXPECT_EQ(out,
            std::vector<cc::Frame>{frame(0x602, {0x80, 0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x05})});
  ASSERT_EQ(rec.results.size(), 2U);
  EXPECT_EQ(rec.results[1].status, SdoStatus::kTimeout);
  EXPECT_EQ(m.counts().sdo_unexpected, 1U);
}

TEST(Master, SdoAbortDecoded) {
  Master m(config({kNode2}));
  Recorder rec;
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x2030, 0x05, rec.done()));
  m.tick(at(0));
  // Abort 0x06020000: object does not exist.
  EXPECT_TRUE(
      m.on_frame(frame(0x582, {0x80, 0x30, 0x20, 0x05, 0x00, 0x00, 0x02, 0x06}), at(3)).empty());
  ASSERT_EQ(rec.results.size(), 1U);
  EXPECT_EQ(rec.results[0], (SdoResult{2, 0x2030, 0x05, SdoStatus::kAbort, 0, 0, 0x06020000}));
  const auto aborts = of_kind(m.drain_events(), MasterEventKind::kSdoAbort);
  ASSERT_EQ(aborts.size(), 1U);
  EXPECT_EQ(std::get<SdoResult>(aborts[0].detail).abort_code, 0x06020000U);
  EXPECT_EQ(m.counts().sdo_protocol_errors, 0U);
}

TEST(Master, SdoMismatchedIndexIsProtocolError) {
  Master m(config({kNode2}));
  Recorder rec;
  const std::array<std::uint8_t, 1> one{0x01};
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1018, 0x02, rec.done()));
  ASSERT_TRUE(m.sdo_download(kNode2, 0x6040, 0x00, one, rec.done()));
  ASSERT_TRUE(m.sdo_download(kNode2, 0x6040, 0x00, one, rec.done()));
  m.tick(at(0));
  // Other index: failed, not delivered as 0x1000's value; the next request goes out.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x01, 0x10, 0x00, 1, 2, 3, 4}), at(1)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x18, 0x10, 0x02, 0, 0, 0, 0})});
  // Other subindex.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x18, 0x10, 0x01, 1, 2, 3, 4}), at(2)),
            std::vector<cc::Frame>{frame(0x602, {0x2F, 0x40, 0x60, 0x00, 0x01, 0, 0, 0})});
  // Download answered with another object's download response.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x60, 0x41, 0x60, 0x00, 0, 0, 0, 0}), at(3)),
            std::vector<cc::Frame>{frame(0x602, {0x2F, 0x40, 0x60, 0x00, 0x01, 0, 0, 0})});
  // Download answered with an upload response: the wrong command; the server is aborted.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x40, 0x60, 0x00, 1, 0, 0, 0}), at(4)),
            std::vector<cc::Frame>{frame(0x602, {0x80, 0x40, 0x60, 0x00, 0x01, 0x00, 0x04, 0x05})});

  ASSERT_EQ(rec.results.size(), 4U);
  for (const SdoResult& r : rec.results) {
    EXPECT_EQ(r.status, SdoStatus::kProtocol);
    EXPECT_EQ(r.value, 0U);
  }
  EXPECT_EQ(m.counts().sdo_protocol_errors, 4U);
  EXPECT_EQ(of_kind(m.drain_events(), MasterEventKind::kSdoProtocol).size(), 4U);
}

TEST(Master, SegmentedInitiateRefused) {
  Master m(config({kNode2}));
  Recorder rec;
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1008, 0x00, rec.done()));
  m.tick(at(0));
  // Initiate upload response, segmented (e = 0, s = 1), 26 bytes announced. Expedited only:
  // the request fails and the server is aborted with 0x05040001.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x41, 0x08, 0x10, 0x00, 0x1A, 0x00, 0x00, 0x00}), at(2)),
            std::vector<cc::Frame>{frame(0x602, {0x80, 0x08, 0x10, 0x00, 0x01, 0x00, 0x04, 0x05})});
  ASSERT_EQ(rec.results.size(), 1U);
  EXPECT_EQ(rec.results[0], (SdoResult{2, 0x1008, 0x00, SdoStatus::kProtocol, 0, 0, 0x05040001}));
  EXPECT_EQ(m.counts().sdo_protocol_errors, 1U);
  EXPECT_EQ(of_kind(m.drain_events(), MasterEventKind::kSdoProtocol).size(), 1U);

  // A segmented initiate for another object: the abort names the object the server is
  // transferring (0x1009:00), not the one requested.
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1008, 0x00, rec.done()));
  m.tick(at(10));
  EXPECT_EQ(m.on_frame(frame(0x582, {0x41, 0x09, 0x10, 0x00, 0x1A, 0x00, 0x00, 0x00}), at(12)),
            std::vector<cc::Frame>{frame(0x602, {0x80, 0x09, 0x10, 0x00, 0x01, 0x00, 0x04, 0x05})});
  ASSERT_EQ(rec.results.size(), 2U);
  EXPECT_EQ(rec.results[1], (SdoResult{2, 0x1008, 0x00, SdoStatus::kProtocol, 0, 0, 0x05040001}));
}

TEST(Master, SdoQueueOverflowRefused) {
  Master m(config({kNode2, kNode3}));
  Recorder rec;
  for (std::size_t i = 0; i < canopen_master::SdoClient::kCapacity; ++i) {
    ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done())) << i;
  }
  EXPECT_FALSE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  EXPECT_TRUE(m.sdo_upload(kNode3, 0x1000, 0x00, rec.done()));   // Per node.
  EXPECT_FALSE(m.sdo_upload(kNode5, 0x1000, 0x00, rec.done()));  // Not configured.
  const std::array<std::uint8_t, 5> five{};
  EXPECT_FALSE(m.sdo_download(kNode3, 0x1017, 0x00, five, rec.done()));
  EXPECT_FALSE(m.sdo_download(kNode3, 0x1017, 0x00, std::span<const std::uint8_t>{}, rec.done()));
  EXPECT_EQ(m.counts().sdo_refused, 4U);

  m.tick(at(0));  // The first of the 16 is now outstanding; it still counts against the bound.
  EXPECT_FALSE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  m.on_frame(frame(0x582, {0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00}), at(1));
  EXPECT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  EXPECT_EQ(m.counts().sdo_refused, 5U);
  ASSERT_EQ(rec.results.size(), 1U);  // Refused requests are never called back.
}

TEST(Master, CancelDropsOnlyQueuedTaggedRequests) {
  Master m(config({kNode2, kNode3}));
  Recorder rec;
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done(), 5));  // Will be outstanding.
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1018, 0x01, rec.done(), 5));  // Queued: dropped.
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1018, 0x02, rec.done()));     // Untagged: kept.
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1018, 0x03, rec.done(), 6));  // Other tag: kept.
  ASSERT_TRUE(
      m.sdo_upload(kNode3, 0x1018, 0x04, rec.done(), 5));  // Outstanding after the tick: kept.
  const std::vector<cc::Frame> first = m.tick(at(0));
  // Both nodes' first requests left; node 3's tagged one is outstanding now and stays.
  ASSERT_EQ(first.size(), 2U);
  EXPECT_EQ(m.cancel_sdo(0), 0U) << "untagged requests are never cancelled";
  EXPECT_EQ(m.cancel_sdo(5), 1U);
  // Node 2: the outstanding 0x1000 completes, then 0x1018:02 and 0x1018:03 follow; never :01.
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x00, 0x10, 0x00, 1, 0, 0, 0}), at(1)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x18, 0x10, 0x02, 0, 0, 0, 0})});
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x18, 0x10, 0x02, 2, 0, 0, 0}), at(2)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x18, 0x10, 0x03, 0, 0, 0, 0})});
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x18, 0x10, 0x03, 3, 0, 0, 0}), at(3)),
            std::vector<cc::Frame>{});
  // Node 3: its outstanding tagged request was on the wire already and still completes.
  m.on_frame(frame(0x583, {0x43, 0x18, 0x10, 0x04, 4, 0, 0, 0}), at(4));
  ASSERT_EQ(rec.results.size(), 4U);
  for (const SdoResult& r : rec.results) EXPECT_NE(r.sub, 0x01) << "a cancelled request ran";
  // The freed slot is usable again.
  EXPECT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done(), 5));
}

TEST(Master, StaleSdoResponseNotMatchedToQueuedRequest) {
  // A response already received when on_frame is called cannot answer a request that this
  // call only now puts on the wire.
  Master m(config({kNode2}));
  Recorder rec;
  m.tick(at(0));
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00, rec.done()));
  EXPECT_EQ(m.on_frame(frame(0x582, {0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00}), at(1)),
            std::vector<cc::Frame>{frame(0x602, {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0})});
  EXPECT_TRUE(rec.results.empty());
  EXPECT_EQ(m.counts().sdo_unexpected, 1U);
  // The real answer, received after the request went out, completes it.
  m.on_frame(frame(0x582, {0x43, 0x00, 0x10, 0x00, 0x91, 0x01, 0x02, 0x00}), at(2));
  ASSERT_EQ(rec.results.size(), 1U);
  EXPECT_EQ(rec.results[0], (SdoResult{2, 0x1000, 0x00, SdoStatus::kOk, 4, 0x00020191, 0}));
}

TEST(Master, ThrowingCallbackLeavesMasterUsable) {
  Master m(config({kNode2, kNode3}));
  Recorder rec;
  ASSERT_TRUE(m.sdo_upload(kNode2, 0x1000, 0x00,
                           [](const SdoResult&) { throw std::runtime_error("callback"); }));
  ASSERT_TRUE(m.sdo_upload(kNode3, 0x1000, 0x00, rec.done()));
  m.tick(at(0));
  // Both time out in the same call; node 2's callback runs first and throws.
  EXPECT_THROW(m.tick(at(200)), std::runtime_error);
  EXPECT_TRUE(rec.results.empty());
  // The master still works, and node 3's callback, not yet called, runs at the next call.
  ASSERT_TRUE(m.nmt(cc::NmtCommand::kStart, kNode3));
  EXPECT_EQ(m.tick(at(201)), std::vector<cc::Frame>{frame(0x000, {0x01, 0x03})});
  ASSERT_EQ(rec.results.size(), 1U);
  EXPECT_EQ(rec.results[0].status, SdoStatus::kTimeout);
  EXPECT_EQ(rec.results[0].node, 3);
}

// ---- SYNC producer ----

const cc::Frame kSync = frame(0x080, {});  // CiA 301 SYNC without counter: COB-ID 0x080, DLC 0.

// The millisecond offsets of the ticks that produced a SYNC.
std::vector<std::int64_t> sync_ticks(Master& m, const std::vector<std::int64_t>& ticks) {
  std::vector<std::int64_t> fired;
  for (const std::int64_t ms : ticks) {
    const auto out = m.tick(at(ms));
    const auto n = std::count(out.begin(), out.end(), kSync);
    EXPECT_LE(n, 1) << ms;
    if (n == 1) {
      fired.push_back(ms);
    }
  }
  return fired;
}

std::vector<std::int64_t> range(std::int64_t from, std::int64_t to) {
  std::vector<std::int64_t> v;
  for (std::int64_t ms = from; ms <= to; ++ms) {
    v.push_back(ms);
  }
  return v;
}

TEST(Master, SyncEveryPeriodAndNeverEarly) {
  EXPECT_EQ(MasterConfig{}.sync_period, milliseconds(10));  // T_bus by default.
  {
    Master m(config({kNode2}, kSyncPeriod));
    EXPECT_EQ(sync_ticks(m, range(0, 100)),
              (std::vector<std::int64_t>{0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100}));
    const auto syncs = of_kind(m.drain_events(), MasterEventKind::kSync);
    ASSERT_EQ(syncs.size(), 11U);
    for (std::size_t i = 0; i < syncs.size(); ++i) {
      EXPECT_EQ(std::get<canopen_master::SyncDetail>(syncs[i].detail).sync_index, i + 1);
      EXPECT_EQ(syncs[i].node, 0);
    }
    EXPECT_EQ(m.syncs_produced(), 11U);
    // One nanosecond before the next slot: nothing.
    EXPECT_TRUE(m.tick(at(110) - Duration{1}).empty());
    EXPECT_EQ(m.tick(at(110)), std::vector<cc::Frame>{kSync});
  }
  {
    // Irregular ticks: a SYNC goes out at the first tick at or after its slot, never before; a
    // slot less than half a period after the SYNC just sent is skipped (39 -> 40 skipped).
    Master m(config({kNode2}, kSyncPeriod));
    EXPECT_EQ(sync_ticks(m, {0, 9, 13, 17, 19, 20, 26, 39, 40, 45, 49, 50, 59, 60}),
              (std::vector<std::int64_t>{0, 13, 20, 39, 50, 60}));
  }
  {
    Master m(config({kNode2}, Duration{}));  // Zero: no SYNC production.
    EXPECT_TRUE(sync_ticks(m, range(0, 50)).empty());
  }
}

TEST(Master, SyncPauseStep) {
  Master m(config({kNode2}, kSyncPeriod));
  EXPECT_FALSE(m.sync_control(canopen_master::SyncControl::kStep));  // Only while paused.
  EXPECT_EQ(sync_ticks(m, range(0, 24)), (std::vector<std::int64_t>{0, 10, 20}));
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kPause));
  EXPECT_TRUE(sync_ticks(m, range(25, 59)).empty());
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kStep));
  EXPECT_EQ(sync_ticks(m, range(60, 61)), (std::vector<std::int64_t>{60}));  // Exactly one.
  // A second step right after: not sooner than half a period after the previous SYNC.
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kStep));
  EXPECT_EQ(sync_ticks(m, range(62, 79)), (std::vector<std::int64_t>{65}));
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kResume));
  EXPECT_EQ(sync_ticks(m, range(80, 100)), (std::vector<std::int64_t>{80, 90, 100}));
  EXPECT_EQ(m.syncs_produced(), 8U);

  // A step not yet produced is dropped by a pause.
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kPause));
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kStep));
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kPause));
  EXPECT_TRUE(sync_ticks(m, range(101, 130)).empty());
}

TEST(Master, SyncStepRefusedWhenPendingOrDisabled) {
  Master m(config({kNode2}, kSyncPeriod));
  EXPECT_EQ(sync_ticks(m, {0}), (std::vector<std::int64_t>{0}));
  ASSERT_TRUE(m.sync_control(canopen_master::SyncControl::kPause));
  EXPECT_TRUE(m.sync_control(canopen_master::SyncControl::kStep));
  EXPECT_FALSE(m.sync_control(canopen_master::SyncControl::kStep));  // One already pending.
  EXPECT_EQ(sync_ticks(m, {10, 20}), (std::vector<std::int64_t>{10}));
  EXPECT_TRUE(m.sync_control(canopen_master::SyncControl::kStep));  // Produced: a new one is fine.
  EXPECT_EQ(sync_ticks(m, {30}), (std::vector<std::int64_t>{30}));

  Master off(config({kNode2}, Duration{}));  // No SYNC production at all.
  ASSERT_TRUE(off.sync_control(canopen_master::SyncControl::kPause));
  EXPECT_FALSE(off.sync_control(canopen_master::SyncControl::kStep));
  EXPECT_TRUE(sync_ticks(off, range(0, 30)).empty());
}

// ---- EMCY consumer ----

TEST(Master, EmcyDecoded) {
  Master m(config({kNode2}, kSyncPeriod));
  m.tick(at(0));
  m.drain_events();
  // EMCY 0x082: error code 0x8210 (PDO not processed, length error), register 0x11, then the
  // five manufacturer bytes.
  EXPECT_TRUE(m.on_frame(frame(0x082, {0x10, 0x82, 0x11, 1, 2, 3, 4, 5}), at(1)).empty());
  // Error reset: code 0x0000.
  EXPECT_TRUE(m.on_frame(frame(0x082, {0x00, 0x00, 0x00, 0, 0, 0, 0, 0}), at(2)).empty());
  // Wrong DLC and a node not in the config: not reported.
  EXPECT_TRUE(m.on_frame(frame(0x082, {0x10, 0x82, 0x11, 1}), at(3)).empty());
  EXPECT_TRUE(m.on_frame(frame(0x085, {0x10, 0x82, 0x11, 1, 2, 3, 4, 5}), at(4)).empty());

  const auto emcy = of_kind(m.drain_events(), MasterEventKind::kEmcy);
  ASSERT_EQ(emcy.size(), 2U);
  EXPECT_EQ(emcy[0].node, 2);
  EXPECT_EQ(emcy[0].at, at(1));
  EXPECT_EQ(std::get<canopen_master::EmcyDetail>(emcy[0].detail),
            (canopen_master::EmcyDetail{0x8210, 0x11, {1, 2, 3, 4, 5}}));
  EXPECT_EQ(std::get<canopen_master::EmcyDetail>(emcy[1].detail),
            (canopen_master::EmcyDetail{0x0000, 0x00, {0, 0, 0, 0, 0}}));
  EXPECT_EQ(m.counts().malformed, 1U);
  EXPECT_EQ(m.counts().foreign, 1U);
}

// ---- Received PDOs ----

using canopen_master::PdoDetail;
using canopen_master::PdoField;
using canopen_master::PdoLayout;
using canopen_master::PdoValue;

// A CANopenNode-style 8-byte map (u32 + i16 + u16) and a 2-byte map (u8 + i8).
const PdoLayout kMap8{
    {{0x2110, 0x01, 32, false}, {0x2111, 0x01, 16, true}, {0x2112, 0x01, 16, false}}};
const PdoLayout kMap2{{{0x2113, 0x01, 8, false}, {0x2114, 0x01, 8, true}}};

TEST(Master, RegisteredLayoutDecodes) {
  Master m(config({kNode2}, kSyncPeriod));
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x182, kMap8));
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x282, kMap2));
  // Fields that are not whole bytes: 12 + 4 bits in 2 bytes.
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x382,
                                PdoLayout{{{0x2115, 0x00, 12, false}, {0x2116, 0x00, 4, true}}}));
  // Refused: more than 64 bits, empty, a zero-length field, a node not in the config, a COB-ID
  // of another service or beyond 11 bits.
  EXPECT_FALSE(m.register_rx_pdo(
      kNode2, 0x482,
      PdoLayout{{{0x2110, 1, 32, false}, {0x2111, 1, 32, false}, {0x2112, 1, 8, false}}}));
  EXPECT_FALSE(m.register_rx_pdo(kNode2, 0x482, PdoLayout{}));
  EXPECT_FALSE(m.register_rx_pdo(kNode2, 0x482, PdoLayout{{{0x2110, 1, 0, false}}}));
  EXPECT_FALSE(m.register_rx_pdo(kNode5, 0x185, kMap2));
  for (const std::uint32_t cob :
       {0x000U, 0x080U, 0x082U, 0x582U, 0x602U, 0x702U, 0x800U, 0x8000'0182U}) {
    EXPECT_FALSE(m.register_rx_pdo(kNode2, cob, kMap2)) << cob;
  }

  m.tick(at(0));  // SYNC 1
  m.on_frame(frame(0x182, {0x78, 0x56, 0x34, 0x12, 0xFE, 0xFF, 0x34, 0x12}), at(1));
  m.tick(at(10));  // SYNC 2
  m.on_frame(frame(0x282, {0x05, 0x80}), at(11));
  m.on_frame(frame(0x382, {0x3F, 0xA2}), at(12));

  const auto pdos = of_kind(m.drain_events(), MasterEventKind::kPdo);
  ASSERT_EQ(pdos.size(), 3U);
  const auto& a = std::get<PdoDetail>(pdos[0].detail);
  EXPECT_EQ(pdos[0].node, 2);
  EXPECT_EQ(a.cob, 0x182);
  EXPECT_EQ(a.sync_index, 1U);
  EXPECT_EQ(a.fields, (std::vector<PdoValue>{{0x2110, 0x01, 32, false, 0x12345678},
                                             {0x2111, 0x01, 16, true, 0xFFFE},
                                             {0x2112, 0x01, 16, false, 0x1234}}));
  EXPECT_EQ(a.fields[1].signed_value(), -2);
  const auto& b = std::get<PdoDetail>(pdos[1].detail);
  EXPECT_EQ(b.cob, 0x282);
  EXPECT_EQ(b.sync_index, 2U);
  EXPECT_EQ(b.fields,
            (std::vector<PdoValue>{{0x2113, 0x01, 8, false, 0x05}, {0x2114, 0x01, 8, true, 0x80}}));
  EXPECT_EQ(b.fields[1].signed_value(), -128);
  const auto& c = std::get<PdoDetail>(pdos[2].detail);
  EXPECT_EQ(c.fields, (std::vector<PdoValue>{{0x2115, 0x00, 12, false, 0x23F},
                                             {0x2116, 0x00, 4, true, 0xA}}));
  EXPECT_EQ(c.fields[1].signed_value(), -6);

  // Registering the same COB-ID again replaces its layout (a remapped PDO).
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x282, PdoLayout{{{0x2117, 0x00, 16, false}}}));
  m.on_frame(frame(0x282, {0x05, 0x80}), at(13));
  const auto again = of_kind(m.drain_events(), MasterEventKind::kPdo);
  ASSERT_EQ(again.size(), 1U);
  EXPECT_EQ(std::get<PdoDetail>(again[0].detail).fields,
            (std::vector<PdoValue>{{0x2117, 0x00, 16, false, 0x8005}}));
}

TEST(Master, WrongLengthNotDecoded) {
  Master m(config({kNode2}, kSyncPeriod));
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x182, kMap8));
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x282, kMap2));
  m.tick(at(0));
  m.drain_events();
  m.on_frame(frame(0x182, {1, 2, 3, 4, 5, 6, 7}), at(1));
  m.on_frame(frame(0x282, {1}), at(2));
  m.on_frame(frame(0x282, {1, 2, 3}), at(3));
  const auto events = m.drain_events();
  EXPECT_TRUE(of_kind(events, MasterEventKind::kPdo).empty());
  const auto errors = of_kind(events, MasterEventKind::kPdoLengthError);
  ASSERT_EQ(errors.size(), 3U);
  EXPECT_EQ(std::get<canopen_master::PdoLengthDetail>(errors[0].detail),
            (canopen_master::PdoLengthDetail{0x182, 7, 8}));
  EXPECT_EQ(std::get<canopen_master::PdoLengthDetail>(errors[1].detail),
            (canopen_master::PdoLengthDetail{0x282, 1, 2}));
  EXPECT_EQ(std::get<canopen_master::PdoLengthDetail>(errors[2].detail),
            (canopen_master::PdoLengthDetail{0x282, 3, 2}));
  EXPECT_EQ(m.counts().pdo_length_errors, 3U);
  m.on_frame(frame(0x182, {1, 2, 3, 4, 5, 6, 7, 8}), at(4));
  EXPECT_EQ(of_kind(m.drain_events(), MasterEventKind::kPdo).size(), 1U);
}

TEST(Master, UnregisteredCobCounted) {
  Master m(config({kNode2}, kSyncPeriod));
  ASSERT_TRUE(m.register_rx_pdo(kNode2, 0x182, kMap8));
  m.tick(at(0));
  m.drain_events();
  EXPECT_TRUE(m.on_frame(frame(0x183, {1, 2, 3, 4, 5, 6, 7, 8}), at(1)).empty());
  EXPECT_TRUE(m.on_frame(frame(0x282, {1, 2}), at(2)).empty());
  EXPECT_TRUE(m.on_frame(frame(0x481, {1, 2, 3, 4, 5, 6, 7, 8}), at(3)).empty());
  EXPECT_TRUE(m.drain_events().empty());
  EXPECT_EQ(m.counts().unregistered_cob, 3U);
  // Another producer's SYNC is not ours: counted as foreign, the SYNC count is unchanged.
  EXPECT_TRUE(m.on_frame(kSync, at(4)).empty());
  EXPECT_EQ(m.counts().foreign, 1U);
  EXPECT_EQ(m.syncs_produced(), 1U);
}

}  // namespace
