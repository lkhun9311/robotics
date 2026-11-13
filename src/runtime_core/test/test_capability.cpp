// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "runtime_core/capability.hpp"

namespace rc = runtime_core;
using C = rc::Capability;
using S = rc::Support;

namespace {
rc::Requirements all_required() { return {true, true, true, true, true}; }
}  // namespace

TEST(Capability, VirtualCanopenMeetsEveryRequirement) {
  EXPECT_TRUE(rc::missing_capabilities(all_required(), rc::virtual_canopen_profile()).empty());
}

TEST(Capability, DirectBackendExpiryIsProvidedByTheHost) {
  EXPECT_EQ(rc::direct_backend_profile().device_side_expiry, S::kProvidedByHost);
  EXPECT_TRUE(rc::missing_capabilities(all_required(), rc::direct_backend_profile()).empty());
}

TEST(Capability, GenericCia402ReportsUnsupportedAndUnknownSeparately) {
  const std::vector<rc::Missing> expected{{C::kDeviceSideExpiry, S::kUnsupported},
                                          {C::kExecutionGeneration, S::kUnsupported},
                                          {C::kLocalStop, S::kUnknown},
                                          {C::kApplyAck, S::kUnsupported},
                                          {C::kSourceTimestamp, S::kUnknown}};
  EXPECT_EQ(rc::missing_capabilities(all_required(), rc::generic_cia402_profile()), expected);
}

TEST(Capability, ModbusLocalStopIsNotApplicableAndStillRefused) {
  rc::Requirements required{};
  required.local_stop = true;
  const std::vector<rc::Missing> expected{{C::kLocalStop, S::kNotApplicable}};
  EXPECT_EQ(rc::missing_capabilities(required, rc::modbus_io_profile()), expected);
  EXPECT_TRUE(rc::missing_capabilities(rc::Requirements{}, rc::modbus_io_profile()).empty());
}

TEST(Capability, Names) {
  EXPECT_EQ(std::string{rc::to_string(C::kApplyAck)}, "APPLY_ACK");
  EXPECT_EQ(std::string{rc::to_string(S::kNotApplicable)}, "NOT_APPLICABLE");
}
