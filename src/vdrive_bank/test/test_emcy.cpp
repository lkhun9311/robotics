// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/cia402.hpp"
#include "rig.hpp"
#include "vdrive_bank/bank.hpp"

// B4: EMCY producer (CiA 301: COB 0x080 + node, DLC 8: error code LE16, error register 1001h,
// five manufacturer bytes, here zero).
// Sent on an accepted fault and, as the error reset (code 0x0000, register 0), when the drive
// leaves Fault; not sent in Stopped.
namespace vdrive_bank::test {
namespace {

namespace od = canopen_codec::od;
using drive_core::Cia402State;

constexpr std::uint8_t kNmtStart = 0x01;
constexpr std::uint8_t kNmtStop = 0x02;

// 1001h of `node` read by expedited SDO upload: the value the EMCY must carry.
std::uint8_t error_register(Rig& r, std::uint16_t node) {
  const std::vector<cc::Frame> reply =
      r.frame(raw(static_cast<std::uint16_t>(0x600 + node), {0x40, 0x01, 0x10, 0x00, 0, 0, 0, 0}));
  EXPECT_EQ(reply.size(), 1U);
  if (reply.size() != 1U) {
    return 0xFF;
  }
  EXPECT_EQ(reply[0].cob_id, 0x580 + node);
  EXPECT_EQ(reply[0].data[0], 0x4F);  // scs 2, expedited, size 1.
  return reply[0].data[4];
}

// W ticks (2 ms) until the fault reaction of `node` ends in Fault.
void until_fault(Rig& r, std::uint8_t node) {
  for (int guard = 0; r.bank.drive(node).cia402().state() != Cia402State::kFault; ++guard) {
    ASSERT_LT(guard, 100);
    r.step(milliseconds(2));
  }
}

TEST(Emcy, FaultSendsCodeAndGenericRegister) {
  Rig r;
  r.ready();
  const Bank::InjectResult res = r.inject("fault 3 8611");
  ASSERT_TRUE(res.ok) << res.error;
  // 0x8611 following error, LE; register 0x01 (generic error); five zero bytes.
  EXPECT_EQ(res.frames,
            std::vector<cc::Frame>{raw(0x083, {0x11, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00})});
  EXPECT_EQ(error_register(r, 3), 0x01);  // The EMCY register byte is 1001h.
  ASSERT_EQ(r.count(BankRecord::Kind::kEmcySent), 1U);
  EXPECT_EQ(r.last(BankRecord::Kind::kEmcySent)->node, 3);
  EXPECT_EQ(r.last(BankRecord::Kind::kEmcySent)->emcy_code, 0x8611);
  ASSERT_EQ(r.count(BankRecord::Kind::kDrive), 1U);
  EXPECT_EQ(r.last(BankRecord::Kind::kDrive)->drive.kind, drive_core::DriveEventKind::kFault);
  EXPECT_EQ(r.bank.drive(3).cia402().state(), Cia402State::kFaultReactionActive);
}

TEST(Emcy, FaultResetSendsErrorReset) {
  Rig r;
  r.ready();
  r.nmt(kNmtStart, 3);  // The controlword reaches the drive by RPDO3 only, in Operational.
  ASSERT_TRUE(r.inject("fault 3 8611").ok);
  until_fault(r, 3);
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySent), 1U);  // The end of the reaction sends none.
  EXPECT_TRUE(r.frame(cc::pack_rpdo3(id(3), od::Rpdo3{0x00})).empty());
  const std::vector<cc::Frame> out = r.frame(cc::pack_rpdo3(id(3), od::Rpdo3{0x80}));  // Reset.
  EXPECT_EQ(r.bank.drive(3).cia402().state(), Cia402State::kSwitchOnDisabled);
  EXPECT_EQ(out,
            std::vector<cc::Frame>{raw(0x083, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})});
  EXPECT_EQ(error_register(r, 3), 0x00);
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySent), 2U);
  EXPECT_EQ(r.last(BankRecord::Kind::kEmcySent)->emcy_code, 0x0000);
}

TEST(Emcy, SuppressedInStopped) {
  Rig r;
  r.ready();
  r.nmt(kNmtStop, 3);
  const Bank::InjectResult res = r.inject("fault 3 8611");
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_TRUE(res.frames.empty());
  EXPECT_EQ(r.bank.drive(3).cia402().state(), Cia402State::kFaultReactionActive);  // Applied.
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySent), 0U);
  ASSERT_EQ(r.count(BankRecord::Kind::kEmcySuppressed), 1U);
  EXPECT_EQ(r.last(BankRecord::Kind::kEmcySuppressed)->node, 3);
  EXPECT_EQ(r.last(BankRecord::Kind::kEmcySuppressed)->emcy_code, 0x8611);
}

TEST(Emcy, TorqueLossCode) {
  Rig r;
  r.ready();
  const Bank::InjectResult res = r.inject("torque_loss 5");
  ASSERT_TRUE(res.ok) << res.error;
  // 0xFF01, LE; register 0x01.
  EXPECT_EQ(res.frames,
            std::vector<cc::Frame>{raw(0x085, {0x01, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00})});
  EXPECT_EQ(r.bank.drive(5).cia402().state(), Cia402State::kFault);
  EXPECT_EQ(error_register(r, 5), 0x01);
}

TEST(Emcy, SecondFaultSendsNothing) {
  Rig r;
  r.ready();
  ASSERT_EQ(r.inject("fault 3 8611").frames.size(), 1U);
  const Bank::InjectResult again = r.inject("fault 3 5000");
  EXPECT_TRUE(again.ok) << again.error;  // Accepted as a command; the drive keeps the first fault.
  EXPECT_TRUE(again.frames.empty());
  until_fault(r, 3);
  const Bank::InjectResult in_fault = r.inject("fault 3 5000");
  EXPECT_TRUE(in_fault.frames.empty());
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySent), 1U);
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySuppressed), 0U);
  EXPECT_EQ(r.bank.drive(3).cia402().fault_code(), 0x8611);
}

}  // namespace
}  // namespace vdrive_bank::test
