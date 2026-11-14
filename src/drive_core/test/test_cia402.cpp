// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "drive_core/cia402.hpp"

// The expected behaviour below is written from the CiA 402 standard text (CiA 402-2, device
// control: the power drive system finite state automaton, controlword 6040h command coding,
// statusword 6041h state coding, quick stop option code 605Ah), not derived from drive_core.
// The project choices that the standard leaves open are 605Ah = 6 (contract D10: quick stop
// ramp, then stay in Quick Stop Active) and the transition 16 hook for the D11 policy latch.
namespace {

using drive_core::Cia402;
using drive_core::Cia402State;
using drive_core::QuickStopReason;
using S = Cia402State;

// Controlword commands (CiA 402, 6040h; bits 7, 3, 2, 1, 0).
constexpr std::uint16_t kShutdown = 0x06;          // 0xxx x110
constexpr std::uint16_t kSwitchOn = 0x07;          // 0xxx 0111 (also Disable Operation)
constexpr std::uint16_t kEnableOperation = 0x0F;   // 0xxx 1111
constexpr std::uint16_t kDisableVoltage = 0x00;    // 0xxx xx0x
constexpr std::uint16_t kQuickStop = 0x02;         // 0xxx x01x
constexpr std::uint16_t kFaultReset = 0x80;        // 0 -> 1 edge of bit 7
constexpr std::uint16_t kDisableOperation = 0x07;  // 0xxx 0111

// Statusword state coding (CiA 402, 6041h): bit 0 ready to switch on, 1 switched on, 2 operation
// enabled, 3 fault, 5 quick stop (active low), 6 switch on disabled.
struct Coding {
  Cia402State state;
  std::uint16_t mask;
  std::uint16_t value;
};
const std::vector<Coding> kStatusword = {
    {S::kNotReadyToSwitchOn, 0x4F, 0x00},   // x0xx 0000
    {S::kSwitchOnDisabled, 0x4F, 0x40},     // x1xx 0000
    {S::kReadyToSwitchOn, 0x6F, 0x21},      // x01x 0001
    {S::kSwitchedOn, 0x6F, 0x23},           // x01x 0011
    {S::kOperationEnabled, 0x6F, 0x27},     // x01x 0111
    {S::kQuickStopActive, 0x6F, 0x07},      // x00x 0111
    {S::kFaultReactionActive, 0x4F, 0x0F},  // x0xx 1111
    {S::kFault, 0x4F, 0x08},                // x0xx 1000
};

// State x controlword -> next state, with the standard's transition number.
struct Row {
  Cia402State from;
  std::uint16_t controlword;
  Cia402State to;
  const char* transition;
};
const std::vector<Row> kTransitions = {
    {S::kSwitchOnDisabled, kShutdown, S::kReadyToSwitchOn, "2"},
    {S::kReadyToSwitchOn, kSwitchOn, S::kSwitchedOn, "3"},
    {S::kReadyToSwitchOn, kEnableOperation, S::kOperationEnabled, "3+4"},
    {S::kSwitchedOn, kEnableOperation, S::kOperationEnabled, "4"},
    {S::kOperationEnabled, kDisableOperation, S::kSwitchedOn, "5"},
    {S::kSwitchedOn, kShutdown, S::kReadyToSwitchOn, "6"},
    {S::kReadyToSwitchOn, kDisableVoltage, S::kSwitchOnDisabled, "7"},
    {S::kReadyToSwitchOn, kQuickStop, S::kSwitchOnDisabled, "7"},
    {S::kOperationEnabled, kShutdown, S::kReadyToSwitchOn, "8"},
    {S::kOperationEnabled, kDisableVoltage, S::kSwitchOnDisabled, "9"},
    {S::kSwitchedOn, kDisableVoltage, S::kSwitchOnDisabled, "10"},
    {S::kSwitchedOn, kQuickStop, S::kSwitchOnDisabled, "10"},
    {S::kOperationEnabled, kQuickStop, S::kQuickStopActive, "11"},
    {S::kQuickStopActive, kDisableVoltage, S::kSwitchOnDisabled, "12"},
    {S::kFault, kFaultReset, S::kSwitchOnDisabled, "15"},
    {S::kQuickStopActive, kEnableOperation, S::kOperationEnabled, "16"},
};

// The same commands with the standard's don't-care bits ("x") set: a command is a bit pattern
// over bits 7, 3, 2, 1, 0, not one controlword value. Bit 8 (halt) and up are not part of it.
const std::vector<Row> kMasked = {
    {S::kOperationEnabled, 0x0D, S::kSwitchOnDisabled, "9 (0xxx xx0x)"},
    {S::kOperationEnabled, 0x04, S::kSwitchOnDisabled, "9 (0xxx xx0x)"},
    {S::kOperationEnabled, 0x0B, S::kQuickStopActive, "11 (0xxx x01x)"},
    {S::kSwitchedOn, 0x0E, S::kReadyToSwitchOn, "6 (0xxx x110)"},
    {S::kSwitchedOn, 0x10F, S::kOperationEnabled, "4 (0xxx 1111, bit 8 set)"},
    // Bit 7 set: Fault Reset, which has no transition outside Fault; not a Shutdown.
    {S::kSwitchOnDisabled, 0x86, S::kSwitchOnDisabled, "- (1xxx xxxx)"},
};

// Commands that have no transition in the state: the state does not change.
const std::vector<Row> kIgnored = {
    {S::kNotReadyToSwitchOn, kShutdown, S::kNotReadyToSwitchOn, "-"},
    {S::kNotReadyToSwitchOn, kEnableOperation, S::kNotReadyToSwitchOn, "-"},
    {S::kSwitchOnDisabled, kSwitchOn, S::kSwitchOnDisabled, "-"},
    {S::kSwitchOnDisabled, kEnableOperation, S::kSwitchOnDisabled, "-"},
    {S::kSwitchOnDisabled, kQuickStop, S::kSwitchOnDisabled, "-"},
    {S::kSwitchOnDisabled, kDisableVoltage, S::kSwitchOnDisabled, "-"},
    {S::kSwitchOnDisabled, kFaultReset, S::kSwitchOnDisabled, "-"},
    {S::kReadyToSwitchOn, kShutdown, S::kReadyToSwitchOn, "-"},
    {S::kSwitchedOn, kSwitchOn, S::kSwitchedOn, "-"},
    {S::kOperationEnabled, kEnableOperation, S::kOperationEnabled, "-"},
    {S::kOperationEnabled, kFaultReset, S::kOperationEnabled, "-"},
    // Option 6: Quick Stop Active has only transitions 12 and 16.
    {S::kQuickStopActive, kQuickStop, S::kQuickStopActive, "-"},
    {S::kQuickStopActive, kShutdown, S::kQuickStopActive, "-"},
    {S::kQuickStopActive, kSwitchOn, S::kQuickStopActive, "-"},
    // Fault Reaction Active leaves only when the reaction is done.
    {S::kFaultReactionActive, kShutdown, S::kFaultReactionActive, "-"},
    {S::kFaultReactionActive, kEnableOperation, S::kFaultReactionActive, "-"},
    {S::kFaultReactionActive, kDisableVoltage, S::kFaultReactionActive, "-"},
    {S::kFaultReactionActive, kFaultReset, S::kFaultReactionActive, "-"},
    // Fault leaves only on Fault Reset.
    {S::kFault, kShutdown, S::kFault, "-"},
    {S::kFault, kSwitchOn, S::kFault, "-"},
    {S::kFault, kEnableOperation, S::kFault, "-"},
    {S::kFault, kDisableVoltage, S::kFault, "-"},
};

constexpr std::uint16_t kFollowingError = 0x8611;

// Drives a fresh machine into `state` through the public interface only.
Cia402 reach(Cia402State state) {
  Cia402 m;
  if (state == S::kNotReadyToSwitchOn) {
    return m;
  }
  m.complete_initialisation();
  switch (state) {
    case S::kReadyToSwitchOn:
      m.on_controlword(kShutdown);
      break;
    case S::kSwitchedOn:
      m.on_controlword(kShutdown);
      m.on_controlword(kSwitchOn);
      break;
    case S::kOperationEnabled:
      m.on_controlword(kShutdown);
      m.on_controlword(kSwitchOn);
      m.on_controlword(kEnableOperation);
      break;
    case S::kQuickStopActive:
      m.on_controlword(kShutdown);
      m.on_controlword(kSwitchOn);
      m.on_controlword(kEnableOperation);
      m.on_controlword(kQuickStop);
      break;
    case S::kFaultReactionActive:
      m.drive_fault(kFollowingError);
      break;
    case S::kFault:
      m.drive_fault(kFollowingError);
      m.fault_reaction_done();
      break;
    default:
      break;
  }
  EXPECT_EQ(m.state(), state) << "could not reach the state under test";
  return m;
}

void expect_coding(const Cia402& m) {
  for (const Coding& c : kStatusword) {
    if (c.state == m.state()) {
      EXPECT_EQ(m.statusword() & c.mask, c.value) << "statusword 0x" << std::hex << m.statusword();
      return;
    }
  }
  ADD_FAILURE() << "state missing from the statusword table";
}

void run_rows(const std::vector<Row>& rows) {
  for (const Row& r : rows) {
    SCOPED_TRACE(testing::Message() << "from " << static_cast<int>(r.from) << " controlword 0x"
                                    << std::hex << r.controlword << " transition " << r.transition);
    Cia402 m = reach(r.from);
    m.on_controlword(r.controlword);
    EXPECT_EQ(m.state(), r.to);
    expect_coding(m);
  }
}

TEST(Cia402, StatuswordFollowsStandardCoding) {
  for (const Coding& c : kStatusword) {
    SCOPED_TRACE(static_cast<int>(c.state));
    const Cia402 m = reach(c.state);
    EXPECT_EQ(m.statusword() & c.mask, c.value);
  }
}

TEST(Cia402, TransitionsFollowStandardTable) { run_rows(kTransitions); }

TEST(Cia402, InvalidCommandsIgnored) { run_rows(kIgnored); }

TEST(Cia402, NotReadyToSwitchOnLeavesAutomatically) {
  Cia402 m;
  EXPECT_EQ(m.state(), S::kNotReadyToSwitchOn);
  m.complete_initialisation();  // Transition 1.
  EXPECT_EQ(m.state(), S::kSwitchOnDisabled);
  m.complete_initialisation();  // Only from Not Ready to Switch On.
  EXPECT_EQ(m.state(), S::kSwitchOnDisabled);
}

TEST(Cia402, QuickStopOption6StaysActive) {
  Cia402 m = reach(S::kOperationEnabled);
  m.on_controlword(kQuickStop);
  ASSERT_EQ(m.state(), S::kQuickStopActive);
  EXPECT_EQ(m.quick_stop_reason(), QuickStopReason::kControlword);
  EXPECT_EQ(m.statusword() & 0x0020, 0);  // Quick stop bit is active low.
  // Option 6 has no automatic transition 12: holding the command keeps the state.
  for (int i = 0; i < 10; ++i) {
    m.on_controlword(kQuickStop);
  }
  EXPECT_EQ(m.state(), S::kQuickStopActive);
}

TEST(Cia402, PolicyQuickStopRecordsReason) {
  Cia402 m = reach(S::kOperationEnabled);
  EXPECT_TRUE(m.quick_stop(QuickStopReason::kTimeout));
  EXPECT_EQ(m.state(), S::kQuickStopActive);
  EXPECT_EQ(m.quick_stop_reason(), QuickStopReason::kTimeout);
  EXPECT_FALSE(m.quick_stop(QuickStopReason::kSyncLoss));  // Already stopping.
  EXPECT_EQ(m.quick_stop_reason(), QuickStopReason::kTimeout);

  Cia402 off = reach(S::kSwitchedOn);
  EXPECT_FALSE(off.quick_stop(QuickStopReason::kIntegrity));
  EXPECT_EQ(off.state(), S::kSwitchedOn);
}

TEST(Cia402, FaultReactionThenFault) {
  for (const Cia402State from : {S::kSwitchOnDisabled, S::kReadyToSwitchOn, S::kSwitchedOn,
                                 S::kOperationEnabled, S::kQuickStopActive}) {
    SCOPED_TRACE(static_cast<int>(from));
    Cia402 m = reach(from);
    m.drive_fault(kFollowingError);  // Transition 13.
    EXPECT_EQ(m.state(), S::kFaultReactionActive);
    EXPECT_EQ(m.fault_code(), kFollowingError);
    m.drive_fault(0x5000);  // A second fault keeps the first code.
    EXPECT_EQ(m.fault_code(), kFollowingError);
    m.fault_reaction_done();  // Transition 14.
    EXPECT_EQ(m.state(), S::kFault);
    expect_coding(m);
    m.fault_reaction_done();  // Only from Fault Reaction Active.
    EXPECT_EQ(m.state(), S::kFault);
  }
}

TEST(Cia402, FaultResetNeedsRisingEdge) {
  // Bit 7 raised during the reaction and held: no edge in Fault.
  Cia402 m = reach(S::kFaultReactionActive);
  m.on_controlword(kFaultReset);
  m.fault_reaction_done();
  m.on_controlword(kFaultReset);
  EXPECT_EQ(m.state(), S::kFault);
  m.on_controlword(0x00);
  EXPECT_EQ(m.state(), S::kFault);
  m.on_controlword(kFaultReset);  // 0 -> 1.
  EXPECT_EQ(m.state(), S::kSwitchOnDisabled);
  EXPECT_EQ(m.fault_code(), std::nullopt);
}

TEST(Cia402, EnableFromQuickStopWithoutInhibit) {
  Cia402 m = reach(S::kQuickStopActive);
  m.on_controlword(kEnableOperation);  // Transition 16, allowed with option 6.
  EXPECT_EQ(m.state(), S::kOperationEnabled);
  EXPECT_EQ(m.quick_stop_reason(), std::nullopt);
}

TEST(Cia402, EnableFromQuickStopInhibited) {
  Cia402 m = reach(S::kQuickStopActive);
  m.set_operation_enable_inhibited(true);
  m.on_controlword(kEnableOperation);
  EXPECT_EQ(m.state(), S::kQuickStopActive);
  m.on_controlword(kDisableVoltage);  // Transition 12 stays available.
  EXPECT_EQ(m.state(), S::kSwitchOnDisabled);
}

// Every way into Operation Enabled (transitions 4, 3+4, 16) is refused while inhibited and works
// again once the inhibit is cleared.
TEST(Cia402, OperationEnableInhibitBlocksEveryEntry) {
  for (const Cia402State from : {S::kSwitchedOn, S::kReadyToSwitchOn, S::kQuickStopActive}) {
    SCOPED_TRACE(static_cast<int>(from));
    Cia402 allowed = reach(from);
    allowed.on_controlword(kEnableOperation);
    EXPECT_EQ(allowed.state(), S::kOperationEnabled);

    Cia402 m = reach(from);
    m.set_operation_enable_inhibited(true);
    EXPECT_TRUE(m.operation_enable_inhibited());
    m.on_controlword(kEnableOperation);
    EXPECT_EQ(m.state(), from);
    expect_coding(m);
    m.set_operation_enable_inhibited(false);
    m.on_controlword(kEnableOperation);
    EXPECT_EQ(m.state(), S::kOperationEnabled);
  }
}

TEST(Cia402, CommandsDecodeByMaskNotValue) { run_rows(kMasked); }

TEST(Cia402, CommStateIsAQuickStopReason) {
  Cia402 m = reach(S::kOperationEnabled);
  EXPECT_TRUE(m.quick_stop(QuickStopReason::kCommState));
  EXPECT_EQ(m.quick_stop_reason(), QuickStopReason::kCommState);
}

TEST(Mode, OnlyCspIsAccepted) {
  Cia402 m = reach(S::kSwitchOnDisabled);
  EXPECT_EQ(m.mode_display(), 8);
  for (int mode = -128; mode <= 127; ++mode) {
    const auto requested = static_cast<std::int8_t>(mode);
    EXPECT_EQ(m.request_mode(requested), mode == 8) << mode;
    EXPECT_EQ(m.mode_display(), 8) << mode;
  }
}

}  // namespace
