// SPDX-License-Identifier: Apache-2.0
// Every golden vector (computed by tools/od/wire.py, independently of the C++ generator) must
// decode to its stated fields and re-encode to its exact bytes. Each failure names the vector.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>

#include "canopen_codec/codec.hpp"
#include "golden.hpp"

namespace cc = canopen_codec;
namespace od = canopen_codec::od;

namespace {

using Fields = std::map<std::string, std::int64_t>;

bool starts_with(const std::string& text, const std::string& prefix) {
  return text.rfind(prefix, 0) == 0;
}

std::optional<cc::NodeId> node_of(const golden::Vector& v) {
  if (v.node < 0 || v.node > 255) {
    return std::nullopt;
  }
  return cc::NodeId::make(static_cast<std::uint8_t>(v.node));
}

// Reads a field the vector must carry; records a failure naming the vector otherwise.
std::int64_t field(const golden::Vector& v, const std::string& name) {
  const auto it = v.fields.find(name);
  if (it == v.fields.end()) {
    ADD_FAILURE() << v.name << ": no field " << name;
    return 0;
  }
  return it->second;
}

// ---- PDO field binding, written here by name and independent of the codec's own ----

template <class Pdo, class T>
struct Bind {
  const char* name;
  T Pdo::*member;
};

template <class Pdo, class T>
constexpr Bind<Pdo, T> bind(const char* name, T Pdo::*member) {
  return {name, member};
}

template <class Pdo>
constexpr auto members();

template <>
constexpr auto members<od::Rpdo1>() {
  return std::make_tuple(bind("target_position", &od::Rpdo1::target_position),
                         bind("command_seq", &od::Rpdo1::command_seq),
                         bind("exec_tag", &od::Rpdo1::exec_tag));
}
template <>
constexpr auto members<od::Rpdo2>() {
  return std::make_tuple(bind("valid_until_us", &od::Rpdo2::valid_until_us),
                         bind("command_seq", &od::Rpdo2::command_seq),
                         bind("exec_tag", &od::Rpdo2::exec_tag));
}
template <>
constexpr auto members<od::Rpdo3>() {
  return std::make_tuple(bind("controlword", &od::Rpdo3::controlword));
}
template <>
constexpr auto members<od::Tpdo1>() {
  return std::make_tuple(bind("statusword", &od::Tpdo1::statusword),
                         bind("position_actual_value", &od::Tpdo1::position_actual_value),
                         bind("sample_seq", &od::Tpdo1::sample_seq));
}
template <>
constexpr auto members<od::Tpdo2>() {
  return std::make_tuple(bind("velocity_actual_value", &od::Tpdo2::velocity_actual_value),
                         bind("sample_seq", &od::Tpdo2::sample_seq),
                         bind("applied_seq", &od::Tpdo2::applied_seq));
}
template <>
constexpr auto members<od::Tpdo3>() {
  return std::make_tuple(bind("sample_time_us", &od::Tpdo3::sample_time_us),
                         bind("sample_seq", &od::Tpdo3::sample_seq),
                         bind("applied_tag", &od::Tpdo3::applied_tag));
}

template <class Pdo>
Fields to_fields(const Pdo& pdo) {
  Fields out;
  std::apply(
      [&](const auto&... b) { ((out[b.name] = static_cast<std::int64_t>(pdo.*(b.member))), ...); },
      members<Pdo>());
  return out;
}

template <class Pdo>
Pdo from_fields(const golden::Vector& v) {
  Pdo pdo{};
  std::apply(
      [&](const auto&... b) {
        (
            [&] {
              const std::int64_t value = field(v, b.name);
              using T = std::remove_reference_t<decltype(pdo.*(b.member))>;
              pdo.*(b.member) = static_cast<T>(value);
              EXPECT_EQ(static_cast<std::int64_t>(pdo.*(b.member)), value)
                  << v.name << ": " << b.name << " does not fit its type";
            }(),
            ...);
      },
      members<Pdo>());
  return pdo;
}

template <class Pdo, class Pack, class Unpack>
void check_pdo(const golden::Vector& v, Pack pack, Unpack unpack) {
  const auto node = node_of(v);
  ASSERT_TRUE(node.has_value()) << v.name << ": invalid node";
  const cc::Frame frame = golden::frame_of(v);
  const std::optional<Pdo> decoded = unpack(*node, frame);
  ASSERT_TRUE(decoded.has_value()) << v.name << ": did not decode";
  EXPECT_EQ(to_fields(*decoded), v.fields) << v.name << ": decoded fields differ";
  EXPECT_EQ(pack(*node, from_fields<Pdo>(v)), frame) << v.name << ": re-encoded bytes differ";
}

}  // namespace

TEST(Golden, FamilyCountsArePinned) {
  // tools/od/make_golden.py writes exactly this many vectors per family. A regeneration that
  // silently yields fewer (or more) vectors fails here; change these numbers only together with
  // the generator.
  const std::map<std::string, std::size_t> expected{
      {"nmt", 12}, {"heartbeat", 9}, {"sync", 1}, {"emcy", 7}, {"sdo", 60}, {"pdo", 182},
  };
  for (const auto& [family, count] : expected) {
    EXPECT_EQ(golden::load(family).size(), count) << family;
  }
}

TEST(Golden, Nmt) {
  const auto vectors = golden::load("nmt");
  ASSERT_FALSE(vectors.empty());
  bool saw_broadcast = false;
  for (const auto& v : vectors) {
    const cc::Frame frame = golden::frame_of(v);
    const auto decoded = cc::decode_nmt(frame);
    ASSERT_TRUE(decoded.has_value()) << v.name << ": did not decode";
    const std::int64_t target = decoded->target ? decoded->target->value() : 0;
    saw_broadcast = saw_broadcast || !decoded->target;
    EXPECT_EQ(
        (Fields{{"command", static_cast<std::int64_t>(decoded->command)}, {"target_node", target}}),
        v.fields)
        << v.name << ": decoded fields differ";
    const auto target_node = field(v, "target_node");
    cc::NmtMessage message{static_cast<cc::NmtCommand>(field(v, "command")), std::nullopt};
    if (target_node != 0) {
      message.target = cc::NodeId::make(static_cast<std::uint8_t>(target_node));
      ASSERT_TRUE(message.target.has_value()) << v.name;
    }
    EXPECT_EQ(cc::encode_nmt(message), frame) << v.name << ": re-encoded bytes differ";
  }
  EXPECT_TRUE(saw_broadcast) << "no node-0 (all nodes) NMT vector";
}

TEST(Golden, Heartbeat) {
  const auto vectors = golden::load("heartbeat");
  ASSERT_FALSE(vectors.empty());
  for (const auto& v : vectors) {
    const cc::Frame frame = golden::frame_of(v);
    const auto decoded = cc::decode_heartbeat(frame);
    ASSERT_TRUE(decoded.has_value()) << v.name << ": did not decode";
    EXPECT_EQ(decoded->node.value(), v.node) << v.name;
    EXPECT_EQ((Fields{{"state", static_cast<std::int64_t>(decoded->state)}}), v.fields)
        << v.name << ": decoded fields differ";
    const auto node = node_of(v);
    ASSERT_TRUE(node.has_value()) << v.name;
    const cc::Heartbeat in{*node, static_cast<cc::NmtState>(field(v, "state"))};
    EXPECT_EQ(cc::encode_heartbeat(in), frame) << v.name << ": re-encoded bytes differ";
  }
}

TEST(Golden, Sync) {
  const auto vectors = golden::load("sync");
  ASSERT_FALSE(vectors.empty());
  for (const auto& v : vectors) {
    const cc::Frame frame = golden::frame_of(v);
    EXPECT_TRUE(cc::is_sync(frame)) << v.name << ": not recognised as SYNC";
    EXPECT_TRUE(v.fields.empty()) << v.name;
    EXPECT_EQ(cc::encode_sync(), frame) << v.name << ": re-encoded bytes differ";
  }
}

TEST(Golden, Emcy) {
  const auto vectors = golden::load("emcy");
  ASSERT_FALSE(vectors.empty());
  for (const auto& v : vectors) {
    const cc::Frame frame = golden::frame_of(v);
    const auto decoded = cc::decode_emcy(frame);
    ASSERT_TRUE(decoded.has_value()) << v.name << ": did not decode";
    EXPECT_EQ(decoded->node.value(), v.node) << v.name;
    std::int64_t manufacturer = 0;
    for (std::size_t i = decoded->manufacturer.size(); i-- > 0;) {
      manufacturer = (manufacturer << 8) | decoded->manufacturer.at(i);
    }
    EXPECT_EQ((Fields{{"error_code", decoded->error_code},
                      {"error_register", decoded->error_register},
                      {"manufacturer", manufacturer}}),
              v.fields)
        << v.name << ": decoded fields differ";
    const auto node = node_of(v);
    ASSERT_TRUE(node.has_value()) << v.name;
    cc::Emcy in{*node,
                static_cast<std::uint16_t>(field(v, "error_code")),
                static_cast<std::uint8_t>(field(v, "error_register")),
                {}};
    const auto mfr = static_cast<std::uint64_t>(field(v, "manufacturer"));
    for (std::size_t i = 0; i < in.manufacturer.size(); ++i) {
      in.manufacturer.at(i) = static_cast<std::uint8_t>(mfr >> (8 * i));
    }
    EXPECT_EQ(cc::encode_emcy(in), frame) << v.name << ": re-encoded bytes differ";
  }
}

TEST(Golden, Sdo) {
  const std::map<std::string, cc::SdoType> prefixes{
      {"sdo_upload_request_", cc::SdoType::kUploadRequest},
      {"sdo_upload_response_", cc::SdoType::kUploadResponse},
      {"sdo_download_request_", cc::SdoType::kDownloadRequest},
      {"sdo_download_response_", cc::SdoType::kDownloadResponse},
      {"sdo_abort_", cc::SdoType::kAbortToClient},
      {"sdo_client_abort_", cc::SdoType::kAbortToServer},
  };
  const auto vectors = golden::load("sdo");
  ASSERT_FALSE(vectors.empty());
  std::set<cc::SdoType> seen;
  for (const auto& v : vectors) {
    std::optional<cc::SdoType> type;
    for (const auto& [prefix, t] : prefixes) {
      if (starts_with(v.name, prefix)) {
        type = t;
      }
    }
    ASSERT_TRUE(type.has_value()) << v.name << ": unknown SDO vector";
    seen.insert(*type);
    const auto node = node_of(v);
    ASSERT_TRUE(node.has_value()) << v.name;
    const cc::Frame frame = golden::frame_of(v);
    const auto decoded = cc::decode_sdo(*node, frame);
    ASSERT_TRUE(decoded.has_value()) << v.name << ": did not decode";
    EXPECT_EQ(decoded->type, *type) << v.name;
    Fields got{{"index", decoded->address.index}, {"sub", decoded->address.sub}};
    cc::Sdo in{
        *type,
        {static_cast<std::uint16_t>(field(v, "index")), static_cast<std::uint8_t>(field(v, "sub"))},
        0,
        0,
        0};
    if (*type == cc::SdoType::kUploadResponse || *type == cc::SdoType::kDownloadRequest) {
      got["size"] = decoded->size;
      got["value"] = decoded->value;
      in.size = static_cast<std::uint8_t>(field(v, "size"));
      in.value = static_cast<std::uint32_t>(field(v, "value"));
    }
    if (*type == cc::SdoType::kAbortToClient || *type == cc::SdoType::kAbortToServer) {
      got["abort_code"] = decoded->abort_code;
      in.abort_code = static_cast<std::uint32_t>(field(v, "abort_code"));
    }
    EXPECT_EQ(got, v.fields) << v.name << ": decoded fields differ";
    const auto encoded = cc::encode_sdo(*node, in);
    ASSERT_TRUE(encoded.has_value()) << v.name << ": did not encode";
    EXPECT_EQ(*encoded, frame) << v.name << ": re-encoded bytes differ";
  }
  EXPECT_EQ(seen.size(), prefixes.size()) << "an SDO type has no golden vector";
}

TEST(Golden, Pdo) {
  const auto vectors = golden::load("pdo");
  ASSERT_FALSE(vectors.empty());
  std::map<std::string, int> per_pdo;
  for (const auto& v : vectors) {
    if (starts_with(v.name, "rpdo1_")) {
      check_pdo<od::Rpdo1>(v, cc::pack_rpdo1, cc::unpack_rpdo1);
      ++per_pdo["rpdo1"];
    } else if (starts_with(v.name, "rpdo2_")) {
      check_pdo<od::Rpdo2>(v, cc::pack_rpdo2, cc::unpack_rpdo2);
      ++per_pdo["rpdo2"];
    } else if (starts_with(v.name, "rpdo3_")) {
      check_pdo<od::Rpdo3>(v, cc::pack_rpdo3, cc::unpack_rpdo3);
      ++per_pdo["rpdo3"];
    } else if (starts_with(v.name, "tpdo1_")) {
      check_pdo<od::Tpdo1>(v, cc::pack_tpdo1, cc::unpack_tpdo1);
      ++per_pdo["tpdo1"];
    } else if (starts_with(v.name, "tpdo2_")) {
      check_pdo<od::Tpdo2>(v, cc::pack_tpdo2, cc::unpack_tpdo2);
      ++per_pdo["tpdo2"];
    } else if (starts_with(v.name, "tpdo3_")) {
      check_pdo<od::Tpdo3>(v, cc::pack_tpdo3, cc::unpack_tpdo3);
      ++per_pdo["tpdo3"];
    } else {
      ADD_FAILURE() << v.name << ": unknown PDO vector";
    }
  }
  EXPECT_EQ(per_pdo.size(), 6U) << "a generated PDO has no golden vector";
}
