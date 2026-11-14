// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "drive_core/cia402.hpp"
#include "rig.hpp"
#include "vdrive_bank/bank.hpp"
#include "vdrive_bank/entropy.hpp"

// B6 commands and B5 boot_ids.
namespace vdrive_bank::test {
namespace {

using drive_core::Cia402State;

// Yields the given values in order, then zero forever.
class FakeEntropy final : public EntropySource {
 public:
  explicit FakeEntropy(std::vector<std::uint32_t> values) : values_(std::move(values)) {}
  std::uint32_t draw() override {
    ++draws;
    return next_ < values_.size() ? values_[next_++] : 0U;
  }
  std::size_t draws{0};

 private:
  std::vector<std::uint32_t> values_;
  std::size_t next_{0};
};

TEST(Inject, RejectsMalformedCommands) {
  Rig r;
  r.ready();
  for (const std::string_view command :
       {"fault 9 8611", "fault 3 zz", "explode 3", "fault 0 8611", "fault 3", "fault 3 18611",
        "fault 3 -1", "fault 3 0x8611", "fault 03 8611", "fault 3 8611 1", "torque_loss",
        "torque_loss 7", "stuck 3 2", "stuck 3", "hb_mute 3 yes", "", " ", "FAULT 3 8611"}) {
    SCOPED_TRACE(command);
    const Bank::InjectResult res = r.inject(command);
    EXPECT_FALSE(res.ok);
    EXPECT_FALSE(res.error.empty());
    EXPECT_TRUE(res.frames.empty());
    ASSERT_TRUE(r.last(BankRecord::Kind::kInject).has_value());
    EXPECT_EQ(std::string_view(r.last(BankRecord::Kind::kInject)->text).substr(0, 4), "err ");
  }
  for (std::uint8_t n = 1; n <= kNodeCount; ++n) {
    EXPECT_EQ(r.bank.drive(n).cia402().state(), Cia402State::kSwitchOnDisabled) << int{n};
    EXPECT_FALSE(r.bank.drive(n).cia402().fault_code().has_value()) << int{n};
    EXPECT_FALSE(r.bank.drive(n).drive_unavailable()) << int{n};
    EXPECT_FALSE(r.bank.heartbeat_muted(n)) << int{n};
  }
  EXPECT_EQ(r.count(BankRecord::Kind::kEmcySent), 0U);
  EXPECT_EQ(r.count(BankRecord::Kind::kDrive), 0U);
}

TEST(Inject, AcceptedCommandsAreTraced) {
  Rig r;
  r.ready();
  for (const std::string_view command :
       {"fault 1 8611", "torque_loss 2", "stuck 3 1", "stuck 3 0", "hb_mute 4 1", "fault 5 a"}) {
    SCOPED_TRACE(command);
    const Bank::InjectResult res = r.inject(command);
    EXPECT_TRUE(res.ok) << res.error;
    EXPECT_TRUE(res.error.empty());
    ASSERT_TRUE(r.last(BankRecord::Kind::kInject).has_value());
    EXPECT_EQ(std::string_view(r.last(BankRecord::Kind::kInject)->text),
              "ok " + std::string(command));
  }
  EXPECT_EQ(r.bank.drive(5).cia402().fault_code(), 0x000A);
  EXPECT_TRUE(r.bank.heartbeat_muted(4));
  // A long command is cut to fit the record and stays NUL-terminated.
  r.inject(std::string(100, 'x'));
  const BankRecord last = *r.last(BankRecord::Kind::kInject);
  EXPECT_EQ(std::strlen(last.text), sizeof(last.text) - 1);
}

TEST(Inject, BootIdsNonzeroAndDistinct) {
  FakeEntropy fake{{0, 7, 7, 9, 0, 9, 11, 13, 15, 7, 17}};
  EXPECT_EQ(draw_boot_ids(fake), (std::array<std::uint32_t, kNodeCount>{7, 9, 11, 13, 15, 17}));
  EXPECT_EQ(fake.draws, 11U);

  FakeEntropy broken{{}};  // Only zeros: process start fails instead of hanging.
  EXPECT_THROW(draw_boot_ids(broken), std::runtime_error);

  GetrandomEntropy kernel;
  const auto ids = draw_boot_ids(kernel);
  const std::set<std::uint32_t> distinct(ids.begin(), ids.end());
  EXPECT_EQ(distinct.size(), kNodeCount);
  EXPECT_EQ(distinct.count(0U), 0U);
}

TEST(Inject, BankRefusesZeroOrDuplicateBootIds) {
  EXPECT_THROW(Bank({1, 2, 3, 4, 5, 0}, kStart), std::invalid_argument);
  EXPECT_THROW(Bank({1, 2, 3, 4, 5, 1}, kStart), std::invalid_argument);
  const Bank bank{{1, 2, 3, 4, 5, 6}, kStart};
  EXPECT_EQ(bank.drive(6).issuer().boot_id(), 6U);
}

}  // namespace
}  // namespace vdrive_bank::test
