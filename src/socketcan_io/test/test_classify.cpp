// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <linux/can.h>

#include <cstring>

#include "socketcan_io/classify.hpp"

namespace {

using socketcan_io::RxClass;

canfd_frame make(canid_t id, unsigned char len) {
  canfd_frame f;
  std::memset(&f, 0, sizeof f);
  f.can_id = id;
  f.len = len;
  return f;
}

}  // namespace

TEST(Classify, DataFrameMapsToCodec) {
  canfd_frame f = make(0x203, 8);
  for (unsigned char i = 0; i < 8; ++i) {
    f.data[i] = static_cast<unsigned char>(0x10 + i);
  }
  EXPECT_EQ(socketcan_io::classify(f, CAN_MTU), RxClass::kData);
  const auto codec = socketcan_io::to_codec(f, CAN_MTU);
  ASSERT_TRUE(codec.has_value());
  EXPECT_EQ(codec->cob_id, 0x203);
  EXPECT_EQ(codec->dlc, 8);
  EXPECT_EQ(codec->data[0], 0x10);
  EXPECT_EQ(codec->data[7], 0x17);
}

TEST(Classify, RejectsRtrErrorFdExtendedAndBadLength) {
  struct Case {
    canfd_frame frame;
    std::size_t nbytes;
    RxClass expected;
  };
  const Case cases[] = {
      {make(0x203 | CAN_RTR_FLAG, 0), CAN_MTU, RxClass::kRtr},
      {make(0x004 | CAN_ERR_FLAG, 8), CAN_MTU, RxClass::kError},
      {make(0x203, 8), CANFD_MTU, RxClass::kFd},
      {make(0x203 | CAN_EFF_FLAG, 8), CAN_MTU, RxClass::kExtendedId},
      {make(0x203, 9), CAN_MTU, RxClass::kBadLength},
      {make(0x203, 8), 5, RxClass::kBadLength},
  };
  for (const Case& c : cases) {
    EXPECT_EQ(socketcan_io::classify(c.frame, c.nbytes), c.expected);
    EXPECT_FALSE(socketcan_io::to_codec(c.frame, c.nbytes).has_value());
  }
}
