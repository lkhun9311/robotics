// SPDX-License-Identifier: Apache-2.0
// The generated header's PDO layouts must equal the layouts computed here, directly from the
// EDS mapping objects (0x1600.., 0x1A00..) and communication objects (0x1400.., 0x1800..).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "canopen_codec/od.hpp"

namespace od = canopen_codec::od;

namespace {

using Sections = std::map<std::string, std::map<std::string, std::string>>;

std::string trim(const std::string& s) {
  const auto begin = s.find_first_not_of(" \t\r");
  if (begin == std::string::npos) {
    return "";
  }
  return s.substr(begin, s.find_last_not_of(" \t\r") - begin + 1);
}

// A minimal INI reader: [section] headers and key=value lines; ';' starts a comment line.
Sections read_eds() {
  std::ifstream in(CANOPEN_CODEC_EDS_PATH);
  if (!in) {
    throw std::runtime_error("cannot open " CANOPEN_CODEC_EDS_PATH);
  }
  Sections out;
  std::string line, section;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == ';') {
      continue;
    }
    if (line.front() == '[' && line.back() == ']') {
      section = line.substr(1, line.size() - 2);
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos || section.empty()) {
      throw std::runtime_error("unexpected EDS line: " + line);
    }
    out[section][trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
  }
  return out;
}

std::string section_name(std::uint32_t index, std::uint32_t sub) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "%04Xsub%u", static_cast<unsigned>(index),
                static_cast<unsigned>(sub));
  return buf;
}

std::string default_value(const Sections& eds, std::uint32_t index, std::uint32_t sub) {
  return eds.at(section_name(index, sub)).at("DefaultValue");
}

struct Expected {
  std::uint16_t index;
  std::uint8_t sub;
  std::uint8_t offset;
  std::uint8_t size;
};

// Offsets are the running sum of the mapped lengths, in mapping sub-index order.
std::vector<Expected> layout_from_eds(const Sections& eds, std::uint32_t map_index,
                                      std::size_t& length) {
  std::vector<Expected> out;
  const auto count = std::stoul(default_value(eds, map_index, 0), nullptr, 0);
  std::size_t offset = 0;
  for (std::uint32_t sub = 1; sub <= count; ++sub) {
    const auto word = std::stoul(default_value(eds, map_index, sub), nullptr, 0);
    const auto bits = word & 0xFFU;
    EXPECT_EQ(bits % 8, 0U) << std::hex << map_index << " sub " << sub;
    out.push_back({static_cast<std::uint16_t>(word >> 16),
                   static_cast<std::uint8_t>((word >> 8) & 0xFFU),
                   static_cast<std::uint8_t>(offset), static_cast<std::uint8_t>(bits / 8)});
    offset += bits / 8;
  }
  length = offset;
  return out;
}

template <class Pdo>
void check_pdo(const Sections& eds, std::uint32_t comm_index, std::uint32_t map_index) {
  SCOPED_TRACE(section_name(map_index, 0));
  std::size_t length = 0;
  const auto expected = layout_from_eds(eds, map_index, length);
  EXPECT_EQ(Pdo::kLength, length);
  ASSERT_EQ(Pdo::kFields.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(Pdo::kFields[i].index, expected[i].index) << "field " << i;
    EXPECT_EQ(Pdo::kFields[i].sub, expected[i].sub) << "field " << i;
    EXPECT_EQ(Pdo::kFields[i].offset, expected[i].offset) << "field " << i;
    EXPECT_EQ(Pdo::kFields[i].size, expected[i].size) << "field " << i;
  }
  // COB-ID "$NODEID+0x..." and the transmission type, from the communication object.
  const std::string cob = default_value(eds, comm_index, 1);
  const std::string prefix = "$NODEID+";
  ASSERT_EQ(cob.rfind(prefix, 0), 0U) << cob;
  EXPECT_TRUE(Pdo::kCobIdAddsNodeId);
  EXPECT_EQ(Pdo::kCobIdBase, std::stoul(cob.substr(prefix.size()), nullptr, 0));
  EXPECT_EQ(Pdo::kTransmissionType, std::stoul(default_value(eds, comm_index, 2), nullptr, 0));
}

}  // namespace

TEST(GeneratedHeader, PdoLayoutsMatchEdsMappings) {
  const Sections eds = read_eds();
  check_pdo<od::Rpdo1>(eds, 0x1400, 0x1600);
  check_pdo<od::Rpdo2>(eds, 0x1401, 0x1601);
  check_pdo<od::Rpdo3>(eds, 0x1402, 0x1602);
  check_pdo<od::Tpdo1>(eds, 0x1800, 0x1A00);
  check_pdo<od::Tpdo2>(eds, 0x1801, 0x1A01);
  check_pdo<od::Tpdo3>(eds, 0x1802, 0x1A02);
  // The contract's lengths (D4, D13): RPDO3 carries the controlword alone.
  EXPECT_EQ(od::Rpdo3::kLength, 2U);
  for (const std::size_t len : {od::Rpdo1::kLength, od::Rpdo2::kLength, od::Tpdo1::kLength,
                                od::Tpdo2::kLength, od::Tpdo3::kLength}) {
    EXPECT_EQ(len, 8U);
  }
}

TEST(GeneratedHeader, ServiceCobIdsMatchEds) {
  const Sections eds = read_eds();
  EXPECT_EQ(od::kSyncCobId, std::stoul(eds.at("1005").at("DefaultValue"), nullptr, 0));
  const std::string emcy = eds.at("1014").at("DefaultValue");
  const std::string prefix = "$NODEID+";
  ASSERT_EQ(emcy.rfind(prefix, 0), 0U) << emcy;
  EXPECT_EQ(od::kEmcyCobIdBase, std::stoul(emcy.substr(prefix.size()), nullptr, 0));
}
