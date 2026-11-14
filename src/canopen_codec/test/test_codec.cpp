// SPDX-License-Identifier: Apache-2.0
// Rejection rules and round trips that the golden vectors (all well-formed) do not exercise.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>

#include "canopen_codec/codec.hpp"
#include "golden.hpp"

namespace cc = canopen_codec;
namespace od = canopen_codec::od;

namespace {

cc::NodeId node(std::uint8_t value) { return cc::NodeId::make(value).value(); }

cc::Frame with_dlc(cc::Frame frame, std::uint8_t dlc) {
  frame.dlc = dlc;
  return frame;
}

cc::Frame with_cob(cc::Frame frame, std::uint16_t cob_id) {
  frame.cob_id = cob_id;
  return frame;
}

}  // namespace

TEST(Codec, DlcMustMatchMapping) {
  static_assert(od::Rpdo3::kLength == 2);
  static_assert(od::Rpdo1::kLength == 8);
  const auto n = node(3);

  const cc::Frame rpdo3 = cc::pack_rpdo3(n, od::Rpdo3{0x000F});
  EXPECT_EQ(rpdo3.dlc, 2);
  EXPECT_TRUE(cc::unpack_rpdo3(n, rpdo3).has_value());
  EXPECT_FALSE(cc::unpack_rpdo3(n, with_dlc(rpdo3, 8)).has_value());
  EXPECT_FALSE(cc::unpack_rpdo3(n, with_dlc(rpdo3, 1)).has_value());

  const cc::Frame rpdo1 = cc::pack_rpdo1(n, od::Rpdo1{-5, 7, 9});
  EXPECT_EQ(rpdo1.dlc, 8);
  EXPECT_TRUE(cc::unpack_rpdo1(n, rpdo1).has_value());
  EXPECT_FALSE(cc::unpack_rpdo1(n, with_dlc(rpdo1, 7)).has_value());
  EXPECT_FALSE(cc::unpack_rpdo1(n, with_dlc(rpdo1, 9)).has_value());

  // Every 8-byte PDO rejects a short frame.
  EXPECT_FALSE(cc::unpack_rpdo2(n, with_dlc(cc::pack_rpdo2(n, {}), 7)).has_value());
  EXPECT_FALSE(cc::unpack_tpdo1(n, with_dlc(cc::pack_tpdo1(n, {}), 7)).has_value());
  EXPECT_FALSE(cc::unpack_tpdo2(n, with_dlc(cc::pack_tpdo2(n, {}), 7)).has_value());
  EXPECT_FALSE(cc::unpack_tpdo3(n, with_dlc(cc::pack_tpdo3(n, {}), 7)).has_value());
}

TEST(Codec, WrongCobIdIsRejected) {
  const auto n = node(6);
  const cc::Frame rpdo1 = cc::pack_rpdo1(n, od::Rpdo1{1, 2, 3});
  EXPECT_EQ(rpdo1.cob_id, 0x206);
  EXPECT_FALSE(cc::unpack_rpdo1(node(5), rpdo1).has_value());             // another node's RPDO1
  EXPECT_FALSE(cc::unpack_rpdo1(n, with_cob(rpdo1, 0x306)).has_value());  // same bytes as RPDO2
  EXPECT_FALSE(cc::unpack_rpdo2(n, rpdo1).has_value());
  EXPECT_FALSE(cc::unpack_tpdo1(n, cc::pack_tpdo2(n, {})).has_value());
  EXPECT_FALSE(cc::unpack_rpdo3(n, with_cob(cc::pack_rpdo3(n, {}), 0x406 + 1)).has_value());

  // Service frames on a foreign identifier.
  EXPECT_FALSE(
      cc::decode_nmt(with_cob(cc::encode_nmt({cc::NmtCommand::kStart, n}), 1)).has_value());
  EXPECT_FALSE(cc::is_sync(with_cob(cc::encode_sync(), 0x81)));
  EXPECT_FALSE(cc::decode_heartbeat(with_cob(cc::encode_heartbeat({n, cc::NmtState::kOperational}),
                                             0x700))
                   .has_value());  // node 0 has no heartbeat
  EXPECT_FALSE(cc::decode_emcy(with_cob(cc::encode_emcy({n, 0x8130, 0x11, {}}), 0x80)).has_value());
  const auto sdo = cc::encode_sdo(n, {cc::SdoType::kUploadRequest, {0x1000, 0}, 0, 0, 0});
  ASSERT_TRUE(sdo.has_value());
  EXPECT_FALSE(cc::decode_sdo(node(7), *sdo).has_value());
}

TEST(Codec, ControlwordRoundTrip) {
  const auto n = node(1);
  constexpr std::array<std::uint16_t, 7> kWords{0x0000, 0x0006, 0x0007, 0x000F,
                                                0x0080, 0x8001, 0xFFFF};
  for (const std::uint16_t word : kWords) {
    const cc::Frame frame = cc::pack_rpdo3(n, od::Rpdo3{word});
    EXPECT_EQ(frame.cob_id, 0x401);
    EXPECT_EQ(frame.data[0], word & 0xFF);
    EXPECT_EQ(frame.data[1], word >> 8);
    const auto back = cc::unpack_rpdo3(n, frame);
    ASSERT_TRUE(back.has_value()) << word;
    EXPECT_EQ(back->controlword, word);
  }
}

TEST(Codec, NodeIdRange) {
  EXPECT_FALSE(cc::NodeId::make(0).has_value());
  EXPECT_FALSE(cc::NodeId::make(128).has_value());
  EXPECT_FALSE(cc::NodeId::make(255).has_value());
  EXPECT_EQ(cc::NodeId::make(1)->value(), 1);
  EXPECT_EQ(cc::NodeId::make(127)->value(), 127);
}

TEST(Codec, DlcAboveEightIsRejected) {
  const auto n = node(2);
  EXPECT_FALSE(cc::unpack_rpdo1(n, with_dlc(cc::pack_rpdo1(n, {}), 15)).has_value());
  EXPECT_FALSE(cc::decode_nmt(with_dlc(cc::encode_nmt({cc::NmtCommand::kStop, n}), 9)).has_value());
  EXPECT_FALSE(cc::decode_emcy(with_dlc(cc::encode_emcy({n, 1, 2, {}}), 9)).has_value());
  const auto sdo = cc::encode_sdo(n, {cc::SdoType::kDownloadResponse, {0x6040, 0}, 0, 0, 0});
  ASSERT_TRUE(sdo.has_value());
  EXPECT_FALSE(cc::decode_sdo(n, with_dlc(*sdo, 9)).has_value());
}

TEST(Codec, ServiceFramesRejectUnknownContent) {
  const auto n = node(4);
  cc::Frame nmt = cc::encode_nmt({cc::NmtCommand::kStart, n});
  nmt.data[0] = 0x03;  // not an NMT command
  EXPECT_FALSE(cc::decode_nmt(nmt).has_value());
  nmt.data[0] = 0x01;
  nmt.data[1] = 128;  // not a node-ID
  EXPECT_FALSE(cc::decode_nmt(nmt).has_value());

  cc::Frame hb = cc::encode_heartbeat({n, cc::NmtState::kOperational});
  hb.data[0] = 0x85;  // node-guarding toggle bit is not part of a heartbeat
  EXPECT_FALSE(cc::decode_heartbeat(hb).has_value());
  EXPECT_FALSE(cc::decode_heartbeat(with_dlc(cc::encode_heartbeat({n, cc::NmtState::kStopped}), 2))
                   .has_value());

  EXPECT_FALSE(cc::is_sync(with_dlc(cc::encode_sync(), 1)));  // no SYNC counter is configured
}

TEST(Codec, SdoRejectsWhatIsNotExpedited) {
  const auto n = node(1);
  const cc::Frame base =
      cc::encode_sdo(n, {cc::SdoType::kDownloadRequest, {0x6040, 0}, 2, 6, 0}).value();
  EXPECT_EQ(base.data[0], 0x2B);
  cc::Frame segmented = base;
  segmented.data[0] = 0x21;  // download initiate, normal (segmented) transfer
  EXPECT_FALSE(cc::decode_sdo(n, segmented).has_value());
  cc::Frame unknown = base;
  unknown.data[0] = 0xC0;  // block transfer
  EXPECT_FALSE(cc::decode_sdo(n, unknown).has_value());
  EXPECT_FALSE(cc::decode_sdo(n, with_dlc(base, 7)).has_value());
  // A response on the request identifier and a request on the response identifier.
  const cc::Frame upload_resp =
      cc::encode_sdo(n, {cc::SdoType::kUploadResponse, {0x6041, 0}, 2, 0x37, 0}).value();
  EXPECT_FALSE(cc::decode_sdo(n, with_cob(upload_resp, 0x601)).has_value());
  EXPECT_FALSE(cc::decode_sdo(n, with_cob(base, 0x581)).has_value());
}

TEST(Codec, SdoEncodeRefusesBadSize) {
  const auto n = node(1);
  EXPECT_FALSE(
      cc::encode_sdo(n, {cc::SdoType::kDownloadRequest, {0x6040, 0}, 0, 0, 0}).has_value());
  EXPECT_FALSE(
      cc::encode_sdo(n, {cc::SdoType::kDownloadRequest, {0x6040, 0}, 5, 0, 0}).has_value());
  EXPECT_FALSE(
      cc::encode_sdo(n, {cc::SdoType::kUploadResponse, {0x6040, 0}, 2, 0x10000, 0}).has_value());
  EXPECT_FALSE(
      cc::encode_sdo(n, {cc::SdoType::kUploadResponse, {0x6040, 0}, 1, 0x100, 0}).has_value());
  const auto three = cc::encode_sdo(n, {cc::SdoType::kUploadResponse, {0x2000, 1}, 3, 0xABCDEF, 0});
  ASSERT_TRUE(three.has_value());
  EXPECT_EQ(three->data[0], 0x47);
  const auto back = cc::decode_sdo(n, *three);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->size, 3);
  EXPECT_EQ(back->value, 0xABCDEFU);
}

TEST(Codec, ClientAbortRoundTrip) {
  const auto n = node(9);
  const auto frame =
      cc::encode_sdo(n, {cc::SdoType::kAbortToServer, {0x2011, 0}, 0, 0, 0x08000020});
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->cob_id, 0x609);
  const auto back = cc::decode_sdo(n, *frame);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->type, cc::SdoType::kAbortToServer);
  EXPECT_EQ(back->abort_code, 0x08000020U);
}

TEST(Codec, SignedFieldsAreTwosComplementLittleEndian) {
  const auto n = node(1);
  const cc::Frame f = cc::pack_tpdo2(n, od::Tpdo2{-2, 0x1234, 0xBEEF});
  EXPECT_EQ(f.cob_id, 0x281);
  const std::array<std::uint8_t, 8> expected{0xFE, 0xFF, 0xFF, 0xFF, 0x34, 0x12, 0xEF, 0xBE};
  EXPECT_EQ(f.data, expected);
  EXPECT_EQ(cc::unpack_tpdo2(n, f)->velocity_actual_value, -2);
}
