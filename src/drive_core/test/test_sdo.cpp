// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/commit.hpp"
#include "drive_core/drive.hpp"
#include "drive_core/exec_tag.hpp"
#include "drive_core/sdo_server.hpp"
#include "drive_core/validity.hpp"
#include "runtime_core/time.hpp"

// The SDO server (CiA 301 expedited) and the execution commit transaction (contract D17), driven
// only through frames: no private state is seeded. Logic time: a FakeClock, W ticks before the
// frames of the same instant.
namespace {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;
namespace abort_code = drive_core::sdo_abort;
using drive_core::Cia402State;
using drive_core::CommitRequest;
using drive_core::Drive;
using drive_core::DriveEvent;
using drive_core::DriveEventKind;
using drive_core::DropReason;
using drive_core::to_wire_us;
using runtime_core::Duration;
using runtime_core::FakeClock;
using runtime_core::milliseconds;
using runtime_core::MonoTime;
namespace flag = drive_core::drive_flag;

const cc::NodeId kNode = cc::NodeId::make(3).value();
constexpr std::uint32_t kBootId = 0x1234'5678;
constexpr MonoTime kBoot{2'000'000'000};
constexpr Duration kW = drive_core::kTickPeriod;
constexpr std::int32_t kFar = 100'000'000;

// A boot_id whose first tag is `start` (found by search, independent of the mixing function).
std::optional<std::uint32_t> boot_id_with_start(std::uint16_t start) {
  for (std::uint32_t id = 0; id < (1U << 24); ++id) {
    if (drive_core::exec_tag_start(id) == start) {
      return id;
    }
  }
  return std::nullopt;
}

bool is_ok(const cc::Sdo& r) { return r.type == cc::SdoType::kDownloadResponse; }

// The abort code of a response, or 0 when it is not an abort.
std::uint32_t abort_of(const cc::Sdo& r) {
  return r.type == cc::SdoType::kAbortToClient ? r.abort_code : 0U;
}

// A drive on the wire: powered on, `after_boot` later, NMT Start sent.
struct Wire {
  FakeClock clock{kBoot};
  Drive drive;
  std::vector<DriveEvent> events;
  std::uint16_t tag{0};
  std::uint16_t seq{0};

  explicit Wire(std::uint32_t boot_id = kBootId, Duration after_boot = milliseconds(100))
      : drive(kNode, boot_id, kBoot) {
    drive.set_event_sink([this](const DriveEvent& e) { events.push_back(e); });
    drive.power_on();
    clock.advance(after_boot);
    nmt(cc::NmtCommand::kStart);
  }

  MonoTime now() const { return clock.now(); }
  std::vector<cc::Frame> send(const cc::Frame& f) { return drive.on_frame(f, now()); }
  void nmt(cc::NmtCommand command) { send(cc::encode_nmt({command, kNode})); }
  void cw(std::uint16_t value) { send(cc::pack_rpdo3(kNode, od::Rpdo3{value})); }
  Cia402State state() const { return drive.cia402().state(); }

  // One SDO request; exactly one response frame, on 0x580 + node, is expected.
  cc::Sdo request(const cc::Sdo& sdo) {
    const auto out = send(cc::encode_sdo(kNode, sdo).value());
    EXPECT_EQ(out.size(), 1U);
    if (out.size() != 1U) {
      return {};
    }
    EXPECT_EQ(out[0].cob_id, 0x580 + kNode.value());
    const auto decoded = cc::decode_sdo(kNode, out[0]);
    EXPECT_TRUE(decoded.has_value());
    if (decoded) {
      EXPECT_EQ(decoded->address.index, sdo.address.index);
      EXPECT_EQ(decoded->address.sub, sdo.address.sub);
    }
    return decoded.value_or(cc::Sdo{});
  }
  cc::Sdo read(od::Address a) { return request({cc::SdoType::kUploadRequest, a, 0, 0, 0}); }
  std::uint32_t value(od::Address a) {
    const cc::Sdo r = read(a);
    EXPECT_EQ(r.type, cc::SdoType::kUploadResponse);
    return r.value;
  }
  cc::Sdo write(od::Address a, std::uint32_t v, std::uint8_t size = 4) {
    return request({cc::SdoType::kDownloadRequest, a, size, v, 0});
  }
  std::uint16_t flags() {
    return static_cast<std::uint16_t>(value(od::addr::feedback_ext_drive_flags));
  }

  // D17 staging: 0x2010:04 first, then 01-03. Returns the matching 0x2011 value.
  std::uint32_t stage(const CommitRequest& r) {
    EXPECT_TRUE(is_ok(write(od::addr::exec_request_attempt_id, r.attempt_id)));
    EXPECT_TRUE(is_ok(write(od::addr::exec_request_generation_lo, r.generation_lo)));
    EXPECT_TRUE(is_ok(write(od::addr::exec_request_generation_hi, r.generation_hi)));
    EXPECT_TRUE(is_ok(write(od::addr::exec_request_session_lo, r.session_lo)));
    return r.crc();
  }
  // The whole adoption: staging, 0x2011, then 0x2016 and 0x2012 read together; the tag is used
  // only if 0x2016 equals the attempt_id.
  std::uint16_t adopt(const CommitRequest& r) {
    const std::uint32_t crc = stage(r);
    EXPECT_TRUE(is_ok(write(od::addr::exec_commit, crc)));
    const std::uint32_t committed = value(od::addr::committed_attempt);
    const auto t = static_cast<std::uint16_t>(value(od::addr::exec_tag));
    EXPECT_EQ(committed, r.attempt_id);
    tag = committed == r.attempt_id ? t : 0;
    return tag;
  }

  void tick() {
    clock.advance(kW);
    drive.tick(now());
  }
  void ticks(int n) {
    for (int i = 0; i < n; ++i) {
      tick();
    }
  }
  std::vector<cc::Frame> sync() { return send(cc::encode_sync()); }
  void fragments(std::uint16_t t, std::uint16_t s, std::int32_t target) {
    send(cc::pack_rpdo1(kNode, od::Rpdo1{target, s, t}));
    send(cc::pack_rpdo2(kNode, od::Rpdo2{to_wire_us(now() + milliseconds(48)), s, t}));
  }
  void command(std::int32_t target = kFar) { fragments(tag, ++seq, target); }
  // Adopted, enabled and tracking a few commands.
  void run(std::uint32_t attempt = 1) {
    cw(0x06);
    cw(0x07);
    adopt({attempt, 7, 0, 9});
    cw(0x0F);
    ASSERT_EQ(state(), Cia402State::kOperationEnabled);
    sync();
    command();
    for (int i = 0; i < 3; ++i) {
      ticks(5);
      sync();
      command();
    }
    ASSERT_TRUE(drive.actuated().has_value());
  }

  std::uint32_t dropped(DropReason reason) const {
    std::uint32_t n = 0;
    for (const auto& e : events) {
      if (e.kind == DriveEventKind::kDropped && e.drop == reason) {
        n += e.count;
      }
    }
    return n;
  }
  std::optional<DriveEvent> last(DriveEventKind kind) const {
    for (auto it = events.rbegin(); it != events.rend(); ++it) {
      if (it->kind == kind) {
        return *it;
      }
    }
    return std::nullopt;
  }
};

// ---- CRC (D17) ----

TEST(Crc, IsoHdlcCheckValueAndCommitVector) {
  // CRC catalogue check value of CRC-32/ISO-HDLC.
  const std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(drive_core::crc32_iso_hdlc(check.data(), check.size()), 0xCBF4'3926U);
  // Python: zlib.crc32(struct.pack('<IIII', 1, 0x89ABCDEF, 0x01234567, 0xDEADBEEF)).
  const CommitRequest r{1, 0x89AB'CDEF, 0x0123'4567, 0xDEAD'BEEF};
  EXPECT_EQ(r.crc(), 0xC2FA'724FU);
  EXPECT_EQ(r.generation(), 0x0123'4567'89AB'CDEFU);
}

// ---- SDO server ----

TEST(Sdo, ReadsDeviceType) {
  Wire w;
  const cc::Sdo r = w.read(od::addr::device_type);
  EXPECT_EQ(r.type, cc::SdoType::kUploadResponse);
  EXPECT_EQ(r.size, 4);
  EXPECT_EQ(r.value, 0x0002'0192U);  // CiA 402 servo drive (EDS 0x1000).
  // Pre-operational serves SDO too (D12).
  w.nmt(cc::NmtCommand::kEnterPreOperational);
  EXPECT_EQ(w.value(od::addr::device_type), 0x0002'0192U);
  // Sizes come from the EDS: an UNSIGNED16 answers 2 bytes, an INTEGER8 1 byte.
  EXPECT_EQ(w.read(od::addr::statusword).size, 2);
  EXPECT_EQ(w.read(od::addr::modes_of_operation_display).size, 1);
  EXPECT_EQ(w.value(od::addr::modes_of_operation_display), 8U);
  EXPECT_EQ(w.value(od::addr::boot_id), kBootId);
  EXPECT_EQ(w.value(od::addr::command_timeout_ms), 30U);
}

TEST(Sdo, UnknownIndexAborts0602) {
  Wire w;
  EXPECT_EQ(abort_of(w.read({0x5000, 0x00})), abort_code::kNoObject);
  EXPECT_EQ(abort_of(w.write({0x5000, 0x00}, 1)), abort_code::kNoObject);
  EXPECT_EQ(abort_of(w.read({0x2017, 0x00})), abort_code::kNoObject);
  // An existing index with a missing sub-index is CiA 301's 0x06090011.
  EXPECT_EQ(abort_of(w.read({0x1000, 0x01})), abort_code::kNoSubIndex);
  EXPECT_EQ(abort_of(w.write({0x2010, 0x05}, 1)), abort_code::kNoSubIndex);
  // A client abort gets no answer.
  EXPECT_TRUE(
      w.send(cc::encode_sdo(kNode, {cc::SdoType::kAbortToServer, {0x1000, 0}, 0, 0, 0x0504'0000})
                 .value())
          .empty());
}

TEST(Sdo, ReadOnlyWriteAborts) {
  Wire w;
  for (const od::Address a :
       {od::addr::device_type, od::addr::statusword, od::addr::exec_tag,
        od::addr::committed_attempt, od::addr::boot_id, od::addr::command_timeout_ms,
        od::addr::identity_object_vendor_id, od::addr::exec_request_highest_sub_index_supported,
        od::addr::feedback_ext_drive_flags, od::addr::cob_id_emcy}) {
    SCOPED_TRACE(a.index);
    const auto size = static_cast<std::uint8_t>(drive_core::SdoServer::find(a)->size);
    EXPECT_EQ(abort_of(w.write(a, 1, size)), abort_code::kReadOnly);
  }
  EXPECT_EQ(w.value(od::addr::boot_id), kBootId);  // Unchanged.
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
}

TEST(Sdo, RangeAborts) {
  Wire w;
  // 6060h: the EDS limits are 8..8, so every other mode is out of range.
  EXPECT_EQ(abort_of(w.write(od::addr::modes_of_operation, 7, 1)), abort_code::kRange);
  EXPECT_EQ(abort_of(w.write(od::addr::modes_of_operation, 0xFF, 1)), abort_code::kRange);  // -1
  EXPECT_EQ(abort_of(w.write(od::addr::modes_of_operation, 9, 1)), abort_code::kRange);
  EXPECT_TRUE(is_ok(w.write(od::addr::modes_of_operation, 8, 1)));
  EXPECT_EQ(w.value(od::addr::modes_of_operation_display), 8U);
  // 0x2015 policy_reset: 0..1.
  EXPECT_EQ(abort_of(w.write(od::addr::policy_reset, 2, 1)), abort_code::kRange);
}

TEST(Sdo, FixedParametersAreConst) {
  // Parameters this drive never changes at run time are `const` in the EDS, so the EDS does not
  // advertise a write the drive would refuse: every write aborts as read-only, even the value
  // the object already has, and the value stays.
  Wire w;
  for (const od::Address a :
       {od::addr::cob_id_sync, od::addr::producer_heartbeat_time,
        od::addr::rpdo1_communication_parameter_cob_id,
        od::addr::rpdo1_communication_parameter_transmission_type,
        od::addr::rpdo2_communication_parameter_cob_id,
        od::addr::rpdo2_communication_parameter_transmission_type,
        od::addr::rpdo3_communication_parameter_cob_id,
        od::addr::rpdo3_communication_parameter_transmission_type,
        od::addr::tpdo1_communication_parameter_cob_id,
        od::addr::tpdo1_communication_parameter_transmission_type,
        od::addr::tpdo2_communication_parameter_cob_id,
        od::addr::tpdo2_communication_parameter_transmission_type,
        od::addr::tpdo3_communication_parameter_cob_id,
        od::addr::tpdo3_communication_parameter_transmission_type, od::addr::quick_stop_option_code,
        od::addr::fault_reaction_option_code, od::addr::quick_stop_deceleration}) {
    SCOPED_TRACE(a.index * 0x100 + a.sub);
    const auto* entry = drive_core::SdoServer::find(a);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->access, od::Access::kConst);
    const std::uint32_t before = w.value(a);
    EXPECT_EQ(abort_of(w.write(a, before, entry->size)), abort_code::kReadOnly);
    EXPECT_EQ(w.value(a), before);
  }
}

TEST(Sdo, UnsupportedCommandSpecifierAborts) {
  // Segmented and block transfers are not served. Every client command specifier this server
  // does not serve is answered with abort 0x05040001; the multiplexer is echoed for the
  // specifiers that carry one and is zero for segments. A client abort is never answered.
  struct Case {
    std::uint8_t command;
    bool has_multiplexer;
  };
  Wire w;
  for (const Case c : {Case{0x21, true}, Case{0x20, true}, Case{0x00, false}, Case{0x10, false},
                       Case{0x60, false}, Case{0x70, false}, Case{0xA0, true}, Case{0xC1, true},
                       Case{0xE0, false}, Case{0x41, true}}) {
    SCOPED_TRACE(static_cast<int>(c.command));
    const cc::Frame request{static_cast<std::uint16_t>(0x600 + kNode.value()),
                            8,
                            {c.command, 0x17, 0x10, 0x00, 0x64, 0x00, 0x00, 0x00}};
    const auto out = w.send(request);
    ASSERT_EQ(out.size(), 1U);
    // CiA 301 abort, bytes as the independent wire encoder writes them (golden sdo.json holds
    // abort 0x05040001 vectors, which the codec reproduces byte for byte).
    const std::array<std::uint8_t, 8> expected =
        c.has_multiplexer
            ? std::array<std::uint8_t, 8>{0x80, 0x17, 0x10, 0x00, 0x01, 0x00, 0x04, 0x05}
            : std::array<std::uint8_t, 8>{0x80, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x05};
    EXPECT_EQ(out[0].cob_id, 0x580 + kNode.value());
    EXPECT_EQ(out[0].dlc, 8);
    EXPECT_EQ(out[0].data, expected);
  }
  EXPECT_EQ(w.value(od::addr::producer_heartbeat_time), 100U);  // Nothing was written.
  // A client abort with reserved bits set is still a client abort: no answer.
  EXPECT_TRUE(
      w.send({static_cast<std::uint16_t>(0x600 + kNode.value()), 8, {0x81, 0, 0x10, 0}}).empty());
  // A frame with the wrong length is not an SDO request at all: no answer.
  EXPECT_TRUE(
      w.send({static_cast<std::uint16_t>(0x600 + kNode.value()), 7, {0x21, 0x17, 0x10}}).empty());
}

TEST(Sdo, SizeUnspecifiedExpeditedDownload) {
  // Expedited with the size indicator clear (e = 1, s = 0): the server takes the entry's size;
  // data bytes beyond it must be zero.
  Wire w;
  const auto download = [&w](std::array<std::uint8_t, 8> data) {
    const auto out = w.send({static_cast<std::uint16_t>(0x600 + kNode.value()), 8, data});
    EXPECT_EQ(out.size(), 1U);
    return out.empty() ? cc::Sdo{} : cc::decode_sdo(kNode, out[0]).value_or(cc::Sdo{});
  };
  EXPECT_TRUE(is_ok(download({0x22, 0x60, 0x60, 0x00, 0x08, 0x00, 0x00, 0x00})));
  EXPECT_TRUE(is_ok(download({0x22, 0x10, 0x20, 0x04, 0x05, 0x00, 0x00, 0x00})));
  EXPECT_EQ(w.value(od::addr::exec_request_attempt_id), 5U);
  EXPECT_EQ(abort_of(download({0x22, 0x60, 0x60, 0x00, 0x08, 0x01, 0x00, 0x00})),
            abort_code::kLength);
  EXPECT_EQ(abort_of(download({0x22, 0x60, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00})),
            abort_code::kRange);
}

TEST(Sdo, LengthMismatchAborts) {
  Wire w;
  EXPECT_EQ(abort_of(w.write(od::addr::modes_of_operation, 8, 2)), abort_code::kLength);
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, 1, 2)), abort_code::kLength);
  EXPECT_EQ(abort_of(w.write(od::addr::exec_request_attempt_id, 1, 1)), abort_code::kLength);
  EXPECT_EQ(abort_of(w.write(od::addr::policy_reset, 1, 4)), abort_code::kLength);
  EXPECT_EQ(w.value(od::addr::exec_request_attempt_id), 0U);  // Nothing staged.
}

TEST(Sdo, PdoOnlyObjectsRefuseSdoWrite) {
  Wire w;
  EXPECT_EQ(abort_of(w.write(od::addr::controlword, 0x06, 2)), abort_code::kUnsupportedAccess);
  EXPECT_EQ(w.state(), Cia402State::kSwitchOnDisabled);
  EXPECT_EQ(abort_of(w.write(od::addr::target_position, 5)), abort_code::kUnsupportedAccess);
  EXPECT_EQ(abort_of(w.write(od::addr::rpdo1_ext_exec_tag, 5, 2)), abort_code::kUnsupportedAccess);
  EXPECT_EQ(abort_of(w.write(od::addr::rpdo2_ext_valid_until_us, 5)),
            abort_code::kUnsupportedAccess);
  w.cw(0x06);
  EXPECT_EQ(w.value(od::addr::controlword), 0x06U);  // Readable: the last RPDO3 value.
}

TEST(Sdo, SyncAndEmcyCobIdsReadBack) {
  Wire w;
  EXPECT_EQ(w.value(od::addr::cob_id_sync), 0x80U);
  EXPECT_EQ(w.value(od::addr::cob_id_sync), od::kSyncCobId);
  EXPECT_EQ(w.value(od::addr::cob_id_emcy), 0x80U + kNode.value());
  EXPECT_EQ(w.value(od::addr::cob_id_emcy), od::kEmcyCobIdBase + kNode.value());
  // PDO COB-IDs are node-relative too.
  EXPECT_EQ(w.value(od::addr::rpdo1_communication_parameter_cob_id), 0x200U + kNode.value());
}

TEST(Sdo, CommitInWrongStateAborts) {
  const CommitRequest r{1, 1, 0, 1};
  {
    SCOPED_TRACE("Switch On Disabled and Ready to Switch On");
    Wire w;
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage(r))), abort_code::kDeviceState);
    w.cw(0x06);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage(r))), abort_code::kDeviceState);
    EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
    EXPECT_EQ(w.value(od::addr::committed_attempt), 0U);
  }
  {
    SCOPED_TRACE("boot quarantine");
    Wire w(kBootId, milliseconds(50));
    w.cw(0x06);
    w.cw(0x07);
    ASSERT_EQ(w.state(), Cia402State::kSwitchedOn);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage(r))), abort_code::kDeviceState);
    EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
    w.clock.advance(milliseconds(50));
    EXPECT_NE(w.adopt(r), 0);
  }
  {
    SCOPED_TRACE("Operation Enabled with an active command");
    Wire w;
    w.run();
    const std::uint16_t tag = w.tag;
    EXPECT_EQ(abort_of(w.write(od::addr::exec_request_attempt_id, 2)), abort_code::kDeviceState);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_request_generation_lo, 2)), abort_code::kDeviceState);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, CommitRequest{2, 7, 0, 9}.crc())),
              abort_code::kDeviceState);
    EXPECT_EQ(w.value(od::addr::exec_tag), tag);  // The running command keeps its tag.
  }
  {
    SCOPED_TRACE("Fault");
    Wire w;
    w.cw(0x06);
    w.cw(0x07);
    w.drive.inject_fault(drive_core::kErrorFollowing, w.now());
    w.ticks(10);
    ASSERT_EQ(w.state(), Cia402State::kFault);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage(r))), abort_code::kDeviceState);
  }
  {
    SCOPED_TRACE("tag wrap");
    const auto id = boot_id_with_start(0xFFFF);
    ASSERT_TRUE(id.has_value());
    Wire w(*id);
    w.cw(0x06);
    w.cw(0x07);
    EXPECT_EQ(w.adopt(r), 0xFFFF);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage({2, 1, 0, 1}))),
              abort_code::kDeviceState);
    EXPECT_EQ(w.value(od::addr::exec_tag), 0U);  // The last tag dies with the refusal.
    EXPECT_EQ(w.value(od::addr::committed_attempt), 1U);
  }
}

// ---- Commit transaction (D17) ----

TEST(Commit, WireFlowActuates) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  ASSERT_EQ(w.state(), Cia402State::kSwitchedOn);
  const CommitRequest r{1, 0x89AB'CDEF, 0x0123'4567, 0xDEAD'BEEF};
  const std::uint32_t crc = w.stage(r);
  EXPECT_EQ(w.value(od::addr::exec_request_attempt_id), 1U);
  EXPECT_EQ(w.value(od::addr::exec_request_generation_lo), 0x89AB'CDEFU);
  EXPECT_EQ(w.value(od::addr::exec_request_generation_hi), 0x0123'4567U);
  EXPECT_EQ(w.value(od::addr::exec_request_session_lo), 0xDEAD'BEEFU);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);  // Nothing issued before 0x2011.
  ASSERT_TRUE(is_ok(w.write(od::addr::exec_commit, crc)));
  EXPECT_EQ(w.value(od::addr::committed_attempt), 1U);
  const auto tag = static_cast<std::uint16_t>(w.value(od::addr::exec_tag));
  EXPECT_EQ(tag, drive_core::exec_tag_start(kBootId));
  EXPECT_EQ(w.drive.issuer().generation(), 0x0123'4567'89AB'CDEFU);
  EXPECT_EQ(w.drive.issuer().session(), 0xDEAD'BEEFU);

  w.cw(0x0F);
  ASSERT_EQ(w.state(), Cia402State::kOperationEnabled);
  w.sync();
  w.fragments(tag, 1, 1'000);
  w.ticks(5);
  EXPECT_FALSE(w.drive.actuated().has_value());  // Not before the next SYNC (D7).
  w.sync();
  ASSERT_TRUE(w.drive.actuated().has_value());
  EXPECT_EQ(w.drive.actuated()->exec_tag, tag);
  EXPECT_EQ(w.drive.actuated()->target_position, 1'000);
  ASSERT_TRUE(w.last(DriveEventKind::kActuated).has_value());
  EXPECT_EQ(w.last(DriveEventKind::kActuated)->tag, tag);
  w.ticks(5);
  const auto fb = w.sync();
  ASSERT_EQ(fb.size(), 3U);
  EXPECT_EQ(cc::unpack_tpdo3(kNode, fb[2])->applied_tag, tag);
  EXPECT_EQ(w.value(od::addr::feedback_ext_applied_tag), tag);
  EXPECT_EQ(w.value(od::addr::feedback_ext_applied_seq), 1U);
  EXPECT_EQ(w.value(od::addr::feedback_ext_sample_time_us),
            cc::unpack_tpdo3(kNode, fb[2])->sample_time_us);
  EXPECT_GT(w.drive.joint().position_nrad, 0);
}

TEST(Commit, RecommitKillsInFlightFragments) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const std::uint16_t a = w.adopt({1, 1, 0, 1});
  w.cw(0x0F);
  w.sync();
  w.fragments(a, 1, 1'000);  // In flight when the re-commit lands.
  const std::uint16_t b = w.adopt({2, 2, 0, 1});
  ASSERT_NE(b, 0);
  EXPECT_NE(a, b);
  w.fragments(a, 2, 1'000);  // Still in flight after it: dropped as another tag's.
  w.ticks(5);
  w.sync();
  EXPECT_FALSE(w.drive.actuated().has_value());  // The held pair of `a` died with its tag.
  EXPECT_EQ(w.dropped(DropReason::kOtherTag), 2U);
  w.fragments(b, 1, 2'000);
  w.ticks(5);
  w.sync();
  ASSERT_TRUE(w.drive.actuated().has_value());
  EXPECT_EQ(w.drive.actuated()->exec_tag, b);
  EXPECT_EQ(w.drive.actuated()->target_position, 2'000);
}

TEST(Commit, MissingStagingWriteAborts) {
  for (const od::Address missing :
       {od::addr::exec_request_generation_lo, od::addr::exec_request_generation_hi,
        od::addr::exec_request_session_lo}) {
    SCOPED_TRACE(missing.sub);
    Wire w;
    w.cw(0x06);
    w.cw(0x07);
    const CommitRequest r{1, 0, 0, 0};  // All zero: a missing write would not change the CRC.
    ASSERT_TRUE(is_ok(w.write(od::addr::exec_request_attempt_id, r.attempt_id)));
    for (const od::Address a :
         {od::addr::exec_request_generation_lo, od::addr::exec_request_generation_hi,
          od::addr::exec_request_session_lo}) {
      if (a.sub != missing.sub) {
        ASSERT_TRUE(is_ok(w.write(a, 0)));
      }
    }
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, r.crc())), abort_code::kTransfer);
    EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
    EXPECT_EQ(w.value(od::addr::committed_attempt), 0U);
    // The staging was cleared: the late write has no window to land in, and a retry fails.
    EXPECT_EQ(abort_of(w.write(missing, 0)), abort_code::kTransfer);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, r.crc())), abort_code::kTransfer);
    // A complete staging of the same attempt_id commits (nothing was committed yet).
    EXPECT_NE(w.adopt(r), 0);
  }
}

TEST(Commit, StaleStagingWriteFailsCrc) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  w.stage({1, 10, 0, 5});  // Abandoned attempt.
  const std::uint32_t crc = w.stage({2, 20, 0, 5});
  // A delayed staging write of the abandoned attempt lands after the new attempt's writes.
  ASSERT_TRUE(is_ok(w.write(od::addr::exec_request_generation_lo, 10)));
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, crc)), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
  EXPECT_EQ(w.value(od::addr::committed_attempt), 0U);
  EXPECT_EQ(w.value(od::addr::exec_request_attempt_id), 0U);  // Staging cleared.
  // A plain wrong CRC fails the same way.
  const std::uint32_t good = w.stage({2, 20, 0, 5});
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, good ^ 1U)), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
}

TEST(Commit, DuplicateCommitIsIdempotent) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const CommitRequest r{1, 3, 0, 4};
  const std::uint16_t tag = w.adopt(r);
  ASSERT_NE(tag, 0);
  const std::uint64_t epoch = w.drive.issuer().epoch();
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(is_ok(w.write(od::addr::exec_commit, r.crc())));
    EXPECT_EQ(w.value(od::addr::committed_attempt), 1U);
    EXPECT_EQ(w.value(od::addr::exec_tag), tag);
    EXPECT_EQ(w.drive.issuer().epoch(), epoch);  // No new allocation, no revocation.
  }
  // No tag was burnt: the next attempt gets the very next tag.
  EXPECT_EQ(w.adopt({2, 3, 0, 4}), static_cast<std::uint16_t>(tag + 1));
}

TEST(Commit, LostResponseRetrySameTag) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const CommitRequest r{1, 3, 0, 4};
  const cc::Frame commit = cc::encode_sdo(kNode, {cc::SdoType::kDownloadRequest,
                                                  od::addr::exec_commit, 4, w.stage(r), 0})
                               .value();
  const auto first = w.send(commit);  // The response is lost on the way to the master.
  ASSERT_EQ(first.size(), 1U);
  const auto tag = static_cast<std::uint16_t>(w.value(od::addr::exec_tag));
  // The master retries after the staging window has closed: still the same answer and tag.
  w.clock.advance(milliseconds(1'500));
  EXPECT_EQ(w.send(commit), first);
  EXPECT_EQ(w.value(od::addr::committed_attempt), 1U);
  EXPECT_EQ(w.value(od::addr::exec_tag), tag);
  // A newer attempt staged meanwhile is not disturbed by a late retry of the old one.
  w.write(od::addr::exec_request_attempt_id, 2);
  EXPECT_EQ(w.send(commit), first);
  EXPECT_EQ(w.value(od::addr::exec_request_attempt_id), 2U);
  // Once that tag is dead, the retry is refused instead of reporting a dead tag as adopted.
  w.nmt(cc::NmtCommand::kResetCommunication);
  const auto late = cc::decode_sdo(kNode, w.send(commit).at(0));
  ASSERT_TRUE(late.has_value());
  EXPECT_EQ(abort_of(*late), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
}

TEST(Commit, WindowExpiryAborts) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const std::uint32_t crc = w.stage({1, 1, 0, 1});
  w.clock.advance(drive_core::kStagingWindow);  // Expired at exact equality.
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, crc)), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
  // A staging write after the window has nothing to attach to.
  ASSERT_TRUE(is_ok(w.write(od::addr::exec_request_attempt_id, 2)));
  w.clock.advance(drive_core::kStagingWindow);
  EXPECT_EQ(abort_of(w.write(od::addr::exec_request_generation_lo, 1)), abort_code::kTransfer);
  // One nanosecond inside the window commits.
  const std::uint32_t crc3 = w.stage({3, 1, 0, 1});
  w.clock.advance(Duration{drive_core::kStagingWindow.ns - 1});
  EXPECT_TRUE(is_ok(w.write(od::addr::exec_commit, crc3)));
  EXPECT_EQ(w.value(od::addr::committed_attempt), 3U);
}

TEST(Commit, NonIncreasingAttemptIsRefused) {
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  ASSERT_NE(w.adopt({5, 1, 0, 1}), 0);
  for (const std::uint32_t attempt : {5U, 4U, 0U}) {
    SCOPED_TRACE(attempt);
    EXPECT_EQ(abort_of(w.write(od::addr::exec_request_attempt_id, attempt)), abort_code::kTransfer);
  }
  ASSERT_TRUE(is_ok(w.write(od::addr::exec_request_attempt_id, 6)));
  // The highest committed attempt_id survives reset communication (D12).
  w.nmt(cc::NmtCommand::kResetCommunication);
  EXPECT_EQ(abort_of(w.write(od::addr::exec_request_attempt_id, 5)), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::committed_attempt), 5U);
  // A new boot starts over.
  w.nmt(cc::NmtCommand::kResetNode);
  w.clock.advance(milliseconds(100));
  w.nmt(cc::NmtCommand::kStart);
  EXPECT_EQ(w.value(od::addr::committed_attempt), 0U);
  w.cw(0x06);
  w.cw(0x07);
  EXPECT_NE(w.adopt({1, 1, 0, 1}), 0);
}

// ---- Policy reset over SDO (D11) ----

// Integrity violation in Switched On with a tag: latched, outside Operation Enabled.
void integrity_latch(Wire& w) {
  w.cw(0x06);
  w.cw(0x07);
  const std::uint16_t tag = w.adopt({1, 1, 0, 1});
  w.send(cc::pack_rpdo1(kNode, od::Rpdo1{1, 1, tag}));
  w.send(cc::pack_rpdo1(kNode, od::Rpdo1{2, 1, tag}));  // Same seq, other payload.
}

TEST(Latch, PolicyResetOverSdo) {
  Wire w;
  integrity_latch(w);
  w.tick();
  ASSERT_TRUE(w.drive.policy_latched());
  ASSERT_EQ(w.state(), Cia402State::kSwitchedOn);
  // Before stop_confirmed the reset is refused; 0 asks for nothing.
  EXPECT_EQ(abort_of(w.write(od::addr::policy_reset, 1, 1)), abort_code::kDeviceState);
  EXPECT_TRUE(is_ok(w.write(od::addr::policy_reset, 0, 1)));
  EXPECT_TRUE(w.drive.policy_latched());
  // A commit is refused while latched, even in Switched On with a valid staging.
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, w.stage({2, 1, 0, 1}))),
            abort_code::kDeviceState);
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
  w.ticks(5);
  ASSERT_TRUE(w.drive.stop_confirmed());
  EXPECT_TRUE(is_ok(w.write(od::addr::policy_reset, 1, 1)));
  EXPECT_FALSE(w.drive.policy_latched());
  EXPECT_EQ(w.state(), Cia402State::kSwitchOnDisabled);
  // Rearm entirely through frames.
  w.cw(0x06);
  w.cw(0x07);
  EXPECT_NE(w.adopt({3, 1, 0, 1}), 0);
  w.cw(0x0F);
  EXPECT_EQ(w.state(), Cia402State::kOperationEnabled);
}

TEST(Commit, RefusedWhileIntegrityStopPending) {
  // Between the detection of a violation and the quick stop at the next W tick, no commit and
  // no staging may start: a tag issued in that gap would outlive the violation it follows.
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const std::uint16_t tag = w.adopt({1, 1, 0, 1});
  ASSERT_NE(tag, 0);
  const std::uint32_t crc = w.stage({2, 2, 0, 2});  // Fully staged before the violation.
  w.send(cc::pack_rpdo1(kNode, od::Rpdo1{1, 1, tag}));
  w.send(cc::pack_rpdo1(kNode, od::Rpdo1{2, 1, tag}));
  ASSERT_FALSE(w.drive.policy_latched());  // Detected; the stop starts at the next tick.
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, crc)), abort_code::kDeviceState);
  EXPECT_EQ(abort_of(w.write(od::addr::exec_request_attempt_id, 3)), abort_code::kDeviceState);
  EXPECT_EQ(w.value(od::addr::committed_attempt), 1U);
  EXPECT_EQ(w.value(od::addr::exec_commit), (CommitRequest{1, 1, 0, 1}.crc()));
  w.tick();
  EXPECT_TRUE(w.drive.policy_latched());
  EXPECT_EQ(w.value(od::addr::exec_tag), 0U);
}

TEST(Commit, AbortedCommitWriteDoesNotChangeReadback) {
  // 0x2011 reads back the value of the last successful write only.
  Wire w;
  w.cw(0x06);
  w.cw(0x07);
  const CommitRequest first{1, 1, 0, 1};
  w.adopt(first);
  EXPECT_EQ(w.value(od::addr::exec_commit), first.crc());
  EXPECT_EQ(abort_of(w.write(od::addr::exec_commit, 0xDEAD'BEEF)), abort_code::kTransfer);
  EXPECT_EQ(w.value(od::addr::exec_commit), first.crc());
}

// ---- 0x2030:05 drive_flags, one test per bit ----

TEST(DriveFlags, BitValuesArePinned) {
  // The wire layout of 0x2030:05 as the EDS comment states it; a master decodes these literals.
  static_assert(flag::kPolicyLatched == 0x0001);
  static_assert(flag::kDriveUnavailable == 0x0002);
  static_assert(flag::kIntegrity == 0x0004);
  static_assert(flag::kStopUnconfirmed == 0x0008);
  Wire w;
  w.drive.inject_torque_loss(w.now());
  EXPECT_EQ(w.value(od::addr::feedback_ext_drive_flags), 0x0002U | 0x0008U);
}

TEST(DriveFlags, PolicyLatchBit) {
  Wire w;
  w.run();
  EXPECT_EQ(w.flags(), 0U);
  for (int i = 0; i < 8 && !w.drive.policy_latched(); ++i) {
    w.ticks(5);  // No SYNC, no command: a policy stop.
  }
  ASSERT_TRUE(w.drive.policy_latched());
  EXPECT_EQ(w.flags(), flag::kPolicyLatched);
  for (int i = 0; i < 200 && !w.drive.stop_confirmed(); ++i) {
    w.tick();
  }
  ASSERT_TRUE(is_ok(w.write(od::addr::policy_reset, 1, 1)));
  EXPECT_EQ(w.flags(), 0U);
}

TEST(DriveFlags, DriveUnavailableBit) {
  Wire w;
  EXPECT_EQ(w.flags() & flag::kDriveUnavailable, 0U);
  w.drive.inject_torque_loss(w.now());
  // Torque loss is never a confirmed stop, so stop_unconfirmed comes with it.
  EXPECT_EQ(w.flags(), flag::kDriveUnavailable | flag::kStopUnconfirmed);
  w.cw(0x80);  // Fault Reset changes nothing while the authority is lost.
  EXPECT_EQ(w.flags() & flag::kDriveUnavailable, flag::kDriveUnavailable);
}

TEST(DriveFlags, IntegrityBit) {
  Wire w;
  integrity_latch(w);
  EXPECT_EQ(w.flags(), flag::kIntegrity);  // Detected at ingestion, stop at the next W tick.
  w.tick();
  EXPECT_EQ(w.flags(), flag::kIntegrity | flag::kPolicyLatched);
  w.ticks(5);
  ASSERT_TRUE(is_ok(w.write(od::addr::policy_reset, 1, 1)));
  EXPECT_EQ(w.flags(), 0U);
}

TEST(DriveFlags, StopUnconfirmedBit) {
  Wire w;
  w.run();
  w.drive.inject_stuck_joint(true);  // The brake does not act: the stop cannot be confirmed.
  for (int i = 0; i < 8 && !w.drive.policy_latched(); ++i) {
    w.ticks(5);
  }
  ASSERT_TRUE(w.drive.policy_latched());
  EXPECT_EQ(w.flags(), flag::kPolicyLatched);
  w.ticks(static_cast<int>(drive_core::kStopDeadline.ns / kW.ns));
  ASSERT_TRUE(w.drive.stop_unconfirmed());
  EXPECT_EQ(w.flags(), flag::kPolicyLatched | flag::kStopUnconfirmed);
}

}  // namespace
