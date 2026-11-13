// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "runtime_core/deadline.hpp"
#include "runtime_core/ids.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/seq.hpp"

namespace rc = runtime_core;

TEST(Deadline, ExpiredAtExactBoundary) {
  EXPECT_FALSE(rc::is_expired(rc::MonoTime{99}, rc::MonoTime{100}));
  EXPECT_TRUE(rc::is_expired(rc::MonoTime{100}, rc::MonoTime{100}));
  EXPECT_TRUE(rc::is_expired(rc::MonoTime{101}, rc::MonoTime{100}));
}

TEST(Deadline, CommandNeverOutlivesPermit) {
  EXPECT_EQ(rc::clamp_expiry(rc::MonoTime{50}, rc::MonoTime{80}).ns, 50);
  EXPECT_EQ(rc::clamp_expiry(rc::MonoTime{90}, rc::MonoTime{80}).ns, 80);
}

TEST(Seq16, WrapsAround) {
  EXPECT_TRUE(rc::seq16_newer(1, 0));
  EXPECT_TRUE(rc::seq16_newer(0, 0xFFFF));
  EXPECT_FALSE(rc::seq16_newer(0xFFFF, 0));
  EXPECT_FALSE(rc::seq16_newer(5, 5));
}

TEST(Seq16, HalfRangeIsNotNewerEitherWay) {
  EXPECT_FALSE(rc::seq16_newer(0x8000, 0));
  EXPECT_FALSE(rc::seq16_newer(0, 0x8000));
  EXPECT_TRUE(rc::seq16_newer(0x7FFF, 0));
}

TEST(IdAllocator, StrictlyIncreasingAndNeverZero) {
  rc::IdAllocator ids;
  const rc::SessionId s1 = ids.next_session();
  const rc::SessionId s2 = ids.next_session();
  const rc::Generation g1 = ids.next_generation();
  const rc::Generation g2 = ids.next_generation();
  EXPECT_NE(s1.value, 0U);
  EXPECT_LT(s1, s2);
  EXPECT_NE(g1.value, 0U);
  EXPECT_LT(g1, g2);
}

TEST(MotionPermit, CarriesIdentityAndExpiry) {
  const rc::MotionPermit p{rc::ManagerInstance{1}, rc::SessionId{2}, rc::Generation{3},
                           rc::MonoTime{4}};
  EXPECT_EQ(p.session.value, 2U);
  EXPECT_EQ(p.generation.value, 3U);
  EXPECT_EQ(p.expires_at.ns, 4);
}
