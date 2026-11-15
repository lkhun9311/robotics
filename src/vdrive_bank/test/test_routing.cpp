// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "rig.hpp"
#include "vdrive_bank/bank.hpp"

// B2: COB-ID routing of received frames to the six nodes (CiA 301 predefined connection set).
namespace vdrive_bank::test {
namespace {

namespace od = canopen_codec::od;
using cc::NmtState;

constexpr std::uint8_t kNmtStart = 0x01;  // NMT command specifiers (CiA 301).
constexpr std::uint8_t kNmtStop = 0x02;

TEST(Routing, BroadcastNmtReachesAllNodes) {
  Rig r;
  r.ready();
  EXPECT_TRUE(r.nmt(kNmtStart, 0).empty());
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    EXPECT_EQ(r.bank.drive(n).nmt().state(), NmtState::kOperational) << int{n};
    EXPECT_EQ(r.bank.drive(n).counts().nmt, 1U) << int{n};
  }
  EXPECT_EQ(r.count(BankRecord::Kind::kNmt), kNodeCount);
  EXPECT_EQ(r.bank.ignored(), IgnoredCounts{});
}

TEST(Routing, AddressedNmtReachesOneNode) {
  Rig r;
  r.ready();
  r.nmt(kNmtStart, 3);
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    const auto& d = r.bank.drive(n);
    EXPECT_EQ(d.nmt().state(), n == 3 ? NmtState::kOperational : NmtState::kPreOperational)
        << int{n};
    // The other nodes never saw the frame: not even as a foreign one.
    EXPECT_EQ(d.counts().nmt, n == 3 ? 1U : 0U) << int{n};
    EXPECT_EQ(d.counts().foreign, 0U) << int{n};
  }
  ASSERT_EQ(r.count(BankRecord::Kind::kNmt), 1U);
  EXPECT_EQ(r.last(BankRecord::Kind::kNmt)->node, 3);
  EXPECT_EQ(r.last(BankRecord::Kind::kNmt)->nmt_state, 0x05);
}

TEST(Routing, UnknownNodeNmtIgnoredAndCounted) {
  Rig r;
  r.ready();
  EXPECT_TRUE(r.nmt(kNmtStart, 7).empty());
  EXPECT_TRUE(r.nmt(kNmtStop, 127).empty());
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    EXPECT_EQ(r.bank.drive(n).nmt().state(), NmtState::kPreOperational) << int{n};
    EXPECT_EQ(r.bank.drive(n).counts().nmt, 0U) << int{n};
  }
  EXPECT_EQ(r.bank.ignored().nmt_unknown_node, 2U);
  EXPECT_EQ(r.count(BankRecord::Kind::kIgnored), 2U);
  EXPECT_EQ(r.last(BankRecord::Kind::kIgnored)->cob_id, 0x000);
}

TEST(Routing, SyncReachesAllInNodeOrder) {
  Rig r;
  r.ready();
  r.nmt(kNmtStart, 0);
  const std::vector<cc::Frame> out = r.frame(raw(0x080, {}));  // SYNC, no counter.
  // TPDO1-3 of every node (Operational, accepted SYNC), node 1 first.
  ASSERT_EQ(out.size(), 3 * kNodeCount);
  for (std::size_t i = 0; i < out.size(); ++i) {
    const std::uint16_t node = static_cast<std::uint16_t>(i / 3 + 1);
    const std::uint16_t base = static_cast<std::uint16_t>(0x180 + 0x100 * (i % 3));
    EXPECT_EQ(out[i].cob_id, base + node) << i;
  }
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    EXPECT_EQ(r.bank.drive(n).counts().sync, 1U) << int{n};
  }
}

TEST(Routing, SdoAndPdoGoToTheirNode) {
  Rig r;
  r.ready();
  r.nmt(kNmtStart, 0);
  // Expedited upload of 6041h:00 to node 5 (CiA 301: ccs 2, index LE, sub).
  const std::vector<cc::Frame> reply = r.frame(raw(0x605, {0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0}));
  ASSERT_EQ(reply.size(), 1U);
  EXPECT_EQ(reply[0].cob_id, 0x585);
  EXPECT_EQ(reply[0].data[0], 0x4B);  // scs 2, expedited, size 2.
  // RPDO3 (0x400 + node) to node 2: controlword Shutdown.
  EXPECT_TRUE(r.frame(cc::pack_rpdo3(id(2), od::Rpdo3{0x06})).empty());
  // RPDO1 and RPDO2 (0x200 / 0x300 + node) to node 4.
  r.frame(cc::pack_rpdo1(id(4), od::Rpdo1{0, 1, 0}));
  r.frame(cc::pack_rpdo2(id(4), od::Rpdo2{0, 1, 0}));
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    const auto& c = r.bank.drive(n).counts();
    EXPECT_EQ(c.sdo, n == 5 ? 1U : 0U) << int{n};
    EXPECT_EQ(c.pdo, n == 2 ? 1U : n == 4 ? 2U : 0U) << int{n};
    EXPECT_EQ(c.foreign, 0U) << int{n};
  }
  EXPECT_EQ(r.bank.drive(2).cia402().state(), drive_core::Cia402State::kReadyToSwitchOn);
  EXPECT_EQ(r.bank.ignored(), IgnoredCounts{});
}

TEST(Routing, OtherCobIdsIgnoredPerClass) {
  Rig r;
  r.ready();
  r.nmt(kNmtStart, 0);
  r.frame(raw(0x087, {0, 0, 0, 0, 0, 0, 0, 0}));           // EMCY of node 7.
  r.frame(raw(0x181, {0, 0, 0, 0, 0, 0, 0, 0}));           // TPDO1 of node 1 (our own, echoed).
  r.frame(raw(0x207, {0, 0, 0, 0, 0, 0, 0, 0}));           // RPDO1 of node 7.
  r.frame(raw(0x501, {0, 0}));                             // RPDO4 of node 1: not in the EDS.
  r.frame(raw(0x581, {0x60, 0, 0, 0, 0, 0, 0, 0}));        // An SDO response.
  r.frame(raw(0x607, {0x40, 0x41, 0x60, 0, 0, 0, 0, 0}));  // SDO request to node 7.
  r.frame(raw(0x701, {0x05}));                             // A heartbeat.
  r.frame(raw(0x100, {0, 0, 0, 0, 0, 0}));                 // TIME.
  r.frame(raw(0x7E5, {0x04}));                             // LSS.
  IgnoredCounts expected;
  expected.emcy = 1;
  expected.pdo = 3;
  expected.sdo = 2;
  expected.heartbeat = 1;
  expected.other = 2;
  EXPECT_EQ(r.bank.ignored(), expected);
  EXPECT_EQ(r.count(BankRecord::Kind::kIgnored), 9U);
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    const auto& c = r.bank.drive(n).counts();
    EXPECT_EQ(c.pdo + c.sdo + c.heartbeat + c.emcy + c.foreign + c.malformed, 0U) << int{n};
  }
}

TEST(Routing, BackwardTimeIsRejected) {
  Rig r;
  EXPECT_THROW(r.bank.power_on(kStart - milliseconds(1)), std::logic_error);
  r.power_on();
  r.at(milliseconds(10));
  const MonoTime earlier = r.now - Duration{1};
  EXPECT_THROW(r.bank.tick(earlier), std::logic_error);
  EXPECT_THROW(r.bank.on_frame(raw(0x000, {kNmtStart, 0}), earlier), std::logic_error);
  EXPECT_THROW(r.bank.inject("fault 3 8611", earlier), std::logic_error);
  // Nothing reached a drive.
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    EXPECT_EQ(r.bank.drive(n).nmt().state(), NmtState::kPreOperational) << int{n};
    EXPECT_FALSE(r.bank.drive(n).cia402().fault_code().has_value()) << int{n};
  }
  // The same instant again is not earlier.
  EXPECT_NO_THROW(r.bank.tick(r.now));
  EXPECT_THROW(r.bank.drive(0), std::out_of_range);
  EXPECT_THROW(r.bank.drive(7), std::out_of_range);
}

}  // namespace
}  // namespace vdrive_bank::test
