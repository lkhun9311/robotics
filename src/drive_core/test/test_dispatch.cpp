// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/drive.hpp"

// NMT gating of the frame dispatcher (contract D12: the CiA 301 service matrix; nodes never
// self-start; reset communication keeps boot_id and the tag allocator).
namespace {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;
using drive_core::Cia402State;
using drive_core::Drive;
using runtime_core::milliseconds;
using runtime_core::MonoTime;

const cc::NodeId kNode = cc::NodeId::make(3).value();
const cc::NodeId kOther = cc::NodeId::make(4).value();
constexpr std::uint32_t kBootId = 0x1234'5678;
constexpr MonoTime kBoot{2'000'000'000};
const MonoTime kReady = kBoot + milliseconds(100);  // After the boot quarantine.

cc::Frame nmt(cc::NmtCommand command, std::optional<cc::NodeId> target = kNode) {
  return cc::encode_nmt({command, target});
}

cc::Frame boot_up() { return cc::encode_heartbeat({kNode, cc::NmtState::kBootUp}); }

cc::Frame sdo_read_statusword() {
  return cc::encode_sdo(kNode, {cc::SdoType::kUploadRequest, od::addr::statusword, 0, 0, 0})
      .value();
}

cc::Frame controlword(std::uint16_t value) { return cc::pack_rpdo3(kNode, od::Rpdo3{value}); }

// A drive after power-on, in Pre-operational.
struct Fixture {
  Drive drive{kNode, kBootId, kBoot};
  std::vector<cc::Frame> boot_frames = drive.power_on();

  std::vector<cc::Frame> send(const cc::Frame& f, MonoTime now = kReady) {
    return drive.on_frame(f, now);
  }
};

TEST(Dispatch, SdoSilentInStopped) {
  Fixture d;
  // Pre-operational: the SDO reaches the SDO service.
  d.send(sdo_read_statusword());
  EXPECT_EQ(d.drive.counts().sdo, 1U);

  EXPECT_TRUE(d.send(nmt(cc::NmtCommand::kStop)).empty());
  ASSERT_EQ(d.drive.nmt().state(), cc::NmtState::kStopped);
  const auto gated = d.drive.counts().gated;
  EXPECT_TRUE(d.send(sdo_read_statusword()).empty());
  EXPECT_EQ(d.drive.counts().sdo, 1U);  // Never reached the SDO service.
  EXPECT_EQ(d.drive.counts().gated, gated + 1);
}

TEST(Dispatch, PdoIgnoredInPreoperational) {
  Fixture d;
  ASSERT_EQ(d.drive.nmt().state(), cc::NmtState::kPreOperational);
  ASSERT_EQ(d.drive.cia402().state(), Cia402State::kSwitchOnDisabled);
  EXPECT_TRUE(d.send(controlword(0x06)).empty());
  EXPECT_TRUE(d.send(cc::pack_rpdo1(kNode, od::Rpdo1{100, 1, 1})).empty());
  EXPECT_TRUE(d.send(cc::pack_rpdo2(kNode, od::Rpdo2{1000, 1, 1})).empty());
  EXPECT_EQ(d.drive.cia402().state(), Cia402State::kSwitchOnDisabled);
  EXPECT_EQ(d.drive.counts().pdo, 0U);
  EXPECT_EQ(d.drive.counts().gated, 3U);

  // The same frame acts once the node is Operational: the gate, not the codec, held it back.
  d.send(nmt(cc::NmtCommand::kStart));
  d.send(controlword(0x06));
  EXPECT_EQ(d.drive.cia402().state(), Cia402State::kReadyToSwitchOn);
  EXPECT_EQ(d.drive.counts().pdo, 1U);
}

TEST(Dispatch, SyncIgnoredInStopped) {
  Fixture d;
  d.send(cc::encode_sync());  // Pre-operational accepts SYNC.
  d.send(nmt(cc::NmtCommand::kStart));
  d.send(cc::encode_sync());
  EXPECT_EQ(d.drive.counts().sync, 2U);

  d.send(nmt(cc::NmtCommand::kStop));
  ASSERT_EQ(d.drive.nmt().state(), cc::NmtState::kStopped);
  EXPECT_TRUE(d.send(cc::encode_sync()).empty());
  EXPECT_TRUE(d.send(controlword(0x06)).empty());
  EXPECT_EQ(d.drive.counts().sync, 2U);
  EXPECT_EQ(d.drive.counts().pdo, 0U);
  EXPECT_EQ(d.drive.cia402().state(), Cia402State::kSwitchOnDisabled);

  // NMT and heartbeat still pass in Stopped.
  d.send(cc::encode_heartbeat({kOther, cc::NmtState::kOperational}));
  EXPECT_EQ(d.drive.counts().heartbeat, 1U);
  d.send(nmt(cc::NmtCommand::kEnterPreOperational));
  EXPECT_EQ(d.drive.nmt().state(), cc::NmtState::kPreOperational);
}

TEST(Dispatch, NoSelfStartAfterBootUp) {
  Drive drive{kNode, kBootId, kBoot};
  // Before power-on (Initialisation) nothing is accepted, not even NMT.
  EXPECT_TRUE(drive.on_frame(nmt(cc::NmtCommand::kStart), kBoot).empty());
  EXPECT_EQ(drive.nmt().state(), cc::NmtState::kBootUp);

  EXPECT_EQ(drive.power_on(), std::vector<cc::Frame>{boot_up()});
  EXPECT_EQ(drive.nmt().state(), cc::NmtState::kPreOperational);
  EXPECT_EQ(drive.cia402().state(), Cia402State::kSwitchOnDisabled);

  // Time, SYNC, heartbeats and NMT commands for another node do not start it.
  for (int i = 0; i < 100; ++i) {
    const MonoTime now = kBoot + milliseconds(10 * i);
    drive.on_frame(cc::encode_sync(), now);
    drive.on_frame(cc::encode_heartbeat({kOther, cc::NmtState::kOperational}), now);
    drive.on_frame(nmt(cc::NmtCommand::kStart, kOther), now);
  }
  EXPECT_EQ(drive.nmt().state(), cc::NmtState::kPreOperational);

  // A broadcast start does.
  EXPECT_TRUE(drive.on_frame(nmt(cc::NmtCommand::kStart, std::nullopt), kReady).empty());
  EXPECT_EQ(drive.nmt().state(), cc::NmtState::kOperational);

  // After a reset the node announces itself again and waits in Pre-operational.
  EXPECT_EQ(drive.on_frame(nmt(cc::NmtCommand::kResetCommunication), kReady),
            std::vector<cc::Frame>{boot_up()});
  EXPECT_EQ(drive.nmt().state(), cc::NmtState::kPreOperational);
}

TEST(Dispatch, ResetCommunicationClearsTagSessionGeneration) {
  Fixture d;
  const auto first = d.drive.issuer().commit(7, 9, kReady);
  ASSERT_TRUE(first.has_value());
  d.send(nmt(cc::NmtCommand::kStart));
  d.send(nmt(cc::NmtCommand::kStop));  // Reset communication is accepted in Stopped too.

  EXPECT_EQ(d.send(nmt(cc::NmtCommand::kResetCommunication)), std::vector<cc::Frame>{boot_up()});
  EXPECT_EQ(d.drive.nmt().state(), cc::NmtState::kPreOperational);
  EXPECT_EQ(d.drive.issuer().current(), 0);
  EXPECT_EQ(d.drive.issuer().generation(), 0U);
  EXPECT_EQ(d.drive.issuer().session(), 0U);
  EXPECT_EQ(d.drive.issuer().boot_id(), kBootId);

  // The allocator survives: the next tag follows the revoked one, no quarantine restarts.
  const auto second = d.drive.issuer().commit(8, 10, kReady);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, static_cast<std::uint16_t>(*first + 1));
}

TEST(Dispatch, ResetNodeStartsNewBoot) {
  Fixture d;
  ASSERT_TRUE(d.drive.issuer().commit(7, 9, kReady).has_value());
  d.send(nmt(cc::NmtCommand::kStart));
  d.send(controlword(0x06));
  ASSERT_EQ(d.drive.cia402().state(), Cia402State::kReadyToSwitchOn);

  EXPECT_EQ(d.send(nmt(cc::NmtCommand::kResetNode)), std::vector<cc::Frame>{boot_up()});
  EXPECT_EQ(d.drive.nmt().state(), cc::NmtState::kPreOperational);
  EXPECT_NE(d.drive.issuer().boot_id(), kBootId);
  EXPECT_EQ(d.drive.issuer().current(), 0);
  EXPECT_EQ(d.drive.cia402().state(), Cia402State::kSwitchOnDisabled);
  // A new boot starts a new quarantine.
  EXPECT_FALSE(d.drive.issuer().commit(1, 1, kReady).has_value());
  EXPECT_TRUE(d.drive.issuer().commit(1, 1, kReady + milliseconds(100)).has_value());
}

TEST(Dispatch, ForeignAndMalformedFramesAreIgnored) {
  Fixture d;
  d.send(nmt(cc::NmtCommand::kStart));
  // Another node's controlword and an SDO response are not for this node.
  d.send(cc::pack_rpdo3(kOther, od::Rpdo3{0x06}));
  d.send(cc::encode_sdo(kNode, {cc::SdoType::kDownloadResponse, od::addr::controlword, 0, 0, 0})
             .value());
  EXPECT_EQ(d.drive.counts().foreign, 2U);
  // A controlword with a DLC other than its mapping's is dropped whole.
  cc::Frame wrong = controlword(0x06);
  wrong.dlc = 3;
  EXPECT_TRUE(d.send(wrong).empty());
  EXPECT_EQ(d.drive.counts().malformed, 1U);
  EXPECT_EQ(d.drive.counts().pdo, 0U);
  EXPECT_EQ(d.drive.cia402().state(), Cia402State::kSwitchOnDisabled);
}

}  // namespace
