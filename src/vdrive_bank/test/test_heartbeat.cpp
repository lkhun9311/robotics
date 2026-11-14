// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "rig.hpp"
#include "vdrive_bank/bank.hpp"

// B3: heartbeat producer, 0x1017 = 100 ms (CiA 301: COB 0x700 + node, DLC 1, the NMT state:
// 0x00 boot-up, 0x04 Stopped, 0x05 Operational, 0x7F Pre-operational).
namespace vdrive_bank::test {
namespace {

constexpr std::uint8_t kNmtStart = 0x01;
constexpr std::uint8_t kNmtStop = 0x02;
constexpr std::uint8_t kNmtResetNode = 0x81;
constexpr std::uint8_t kNmtResetCommunication = 0x82;

std::vector<cc::Frame> all_nodes(std::uint8_t state) {
  std::vector<cc::Frame> out;
  for (std::uint16_t n = 1; n <= kNodeCount; ++n) {
    out.push_back(raw(static_cast<std::uint16_t>(0x700 + n), {state}));
  }
  return out;
}

std::vector<cc::Frame> without(std::vector<cc::Frame> frames, std::uint16_t cob_id) {
  std::erase_if(frames, [cob_id](const cc::Frame& f) { return f.cob_id == cob_id; });
  return frames;
}

TEST(Heartbeat, BootUpThenEvery100ms) {
  Rig r;
  EXPECT_EQ(r.power_on(), all_nodes(0x00));
  for (int ms = 1; ms <= 99; ++ms) {
    EXPECT_TRUE(r.at(milliseconds(ms)).empty()) << ms;
  }
  EXPECT_EQ(r.at(milliseconds(100)), all_nodes(0x7F));
  EXPECT_TRUE(r.at(milliseconds(199)).empty());
  EXPECT_EQ(r.at(milliseconds(200)), all_nodes(0x7F));
  // A late tick sends one heartbeat, not the missed ones, and keeps the phase.
  EXPECT_EQ(r.at(milliseconds(450)), all_nodes(0x7F));
  EXPECT_TRUE(r.at(milliseconds(499)).empty());
  EXPECT_EQ(r.at(milliseconds(500)), all_nodes(0x7F));
  EXPECT_EQ(r.count(BankRecord::Kind::kHeartbeat), 5 * kNodeCount);
}

TEST(Heartbeat, NothingBeforePowerOn) {
  Rig r;
  EXPECT_TRUE(r.at(milliseconds(100)).empty());
  EXPECT_TRUE(r.at(milliseconds(1000)).empty());
}

TEST(Heartbeat, StateBytePerNmtState) {
  Rig r;
  r.power_on();
  r.nmt(kNmtStart, 1);
  r.nmt(kNmtStop, 2);
  const std::vector<cc::Frame> hb = r.at(milliseconds(100));
  ASSERT_EQ(hb.size(), kNodeCount);
  EXPECT_EQ(hb[0], raw(0x701, {0x05}));
  EXPECT_EQ(hb[1], raw(0x702, {0x04}));
  EXPECT_EQ(hb[2], raw(0x703, {0x7F}));
}

TEST(Heartbeat, PhaseRestartsAtResetNode) {
  for (const std::uint8_t command : {kNmtResetNode, kNmtResetCommunication}) {
    SCOPED_TRACE(int{command});
    Rig r;
    r.power_on();
    r.at(milliseconds(50));
    EXPECT_EQ(r.nmt(command, 4), std::vector<cc::Frame>{raw(0x704, {0x00})});
    EXPECT_EQ(r.at(milliseconds(100)), without(all_nodes(0x7F), 0x704));
    EXPECT_TRUE(r.at(milliseconds(149)).empty());
    EXPECT_EQ(r.at(milliseconds(150)), std::vector<cc::Frame>{raw(0x704, {0x7F})});
    EXPECT_EQ(r.at(milliseconds(200)), without(all_nodes(0x7F), 0x704));
  }
}

TEST(Heartbeat, SentInStopped) {
  Rig r;
  r.power_on();
  r.nmt(kNmtStop, 0);
  EXPECT_EQ(r.at(milliseconds(100)), all_nodes(0x04));
  EXPECT_EQ(r.at(milliseconds(200)), all_nodes(0x04));
}

TEST(Heartbeat, MuteSuppressesPeriodicNotBootUp) {
  Rig r;
  // Muted before power-on (the --hb-muted start option): the boot-up still goes out.
  ASSERT_TRUE(r.inject("hb_mute 2 1").ok);
  EXPECT_TRUE(r.bank.heartbeat_muted(2));
  EXPECT_EQ(r.power_on(), all_nodes(0x00));
  EXPECT_EQ(r.at(milliseconds(100)), without(all_nodes(0x7F), 0x702));
  r.at(milliseconds(130));
  EXPECT_EQ(r.nmt(kNmtResetNode, 2), std::vector<cc::Frame>{raw(0x702, {0x00})});
  EXPECT_EQ(r.at(milliseconds(200)), without(all_nodes(0x7F), 0x702));
  EXPECT_EQ(r.at(milliseconds(230)), std::vector<cc::Frame>{});  // Node 2's phase: muted.
  // Unmuted: the periodic heartbeat comes back on the phase of the latest boot-up.
  ASSERT_TRUE(r.inject("hb_mute 2 0").ok);
  EXPECT_FALSE(r.bank.heartbeat_muted(2));
  EXPECT_EQ(heartbeats(r.at(milliseconds(300))), without(all_nodes(0x7F), 0x702));
  EXPECT_EQ(r.at(milliseconds(330)), std::vector<cc::Frame>{raw(0x702, {0x7F})});
}

}  // namespace
}  // namespace vdrive_bank::test
