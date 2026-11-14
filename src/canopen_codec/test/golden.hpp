// SPDX-License-Identifier: Apache-2.0
#pragma once

// Loader for the golden frame vectors written by tools/od/make_golden.py. The files are
// machine-generated JSON: an array of objects with string, integer, integer-array and flat
// object values. This parser accepts exactly that subset and throws on anything else, so a
// malformed file fails the test instead of yielding fewer vectors.

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "canopen_codec/codec.hpp"

namespace canopen_codec {

// Readable frames in test failure messages.
inline void PrintTo(const Frame& f, std::ostream* os) {
  *os << "{cob_id 0x" << std::hex << f.cob_id << std::dec << ", dlc " << static_cast<int>(f.dlc)
      << ", data";
  for (const auto byte : f.data) {
    *os << ' ' << static_cast<int>(byte);
  }
  *os << '}';
}

}  // namespace canopen_codec

namespace golden {

struct Vector {
  std::string name;
  std::int64_t node{};
  std::int64_t cob_id{};
  std::int64_t dlc{};
  std::vector<std::int64_t> data;
  std::map<std::string, std::int64_t> fields;
};

class Parser {
 public:
  Parser(std::string text, std::string origin)
      : text_(std::move(text)), origin_(std::move(origin)) {}

  std::vector<Vector> vectors() {
    std::vector<Vector> out;
    expect('[');
    if (!consume(']')) {
      do {
        out.push_back(vector());
      } while (consume(','));
      expect(']');
    }
    skip_space();
    if (pos_ != text_.size()) {
      fail("trailing content");
    }
    return out;
  }

 private:
  Vector vector() {
    Vector v;
    bool has_name = false, has_node = false, has_cob = false, has_dlc = false, has_data = false,
         has_fields = false;
    expect('{');
    do {
      const std::string key = string();
      expect(':');
      if (key == "name") {
        v.name = string();
        has_name = true;
      } else if (key == "node") {
        v.node = integer();
        has_node = true;
      } else if (key == "cob_id") {
        v.cob_id = integer();
        has_cob = true;
      } else if (key == "dlc") {
        v.dlc = integer();
        has_dlc = true;
      } else if (key == "data") {
        expect('[');
        if (!consume(']')) {
          do {
            v.data.push_back(integer());
          } while (consume(','));
          expect(']');
        }
        has_data = true;
      } else if (key == "fields") {
        expect('{');
        if (!consume('}')) {
          do {
            const std::string field = string();
            expect(':');
            if (!v.fields.emplace(field, integer()).second) {
              fail("duplicate field " + field);
            }
          } while (consume(','));
          expect('}');
        }
        has_fields = true;
      } else {
        fail("unknown key " + key);
      }
    } while (consume(','));
    expect('}');
    if (!(has_name && has_node && has_cob && has_dlc && has_data && has_fields)) {
      fail("vector " + v.name + " lacks a key");
    }
    return v;
  }

  std::string string() {
    expect('"');
    std::string out;
    while (pos_ < text_.size() && text_[pos_] != '"') {
      if (text_[pos_] == '\\') {
        fail("escapes are not expected in golden files");
      }
      out.push_back(text_[pos_++]);
    }
    expect('"');
    return out;
  }

  std::int64_t integer() {
    skip_space();
    const std::size_t start = pos_;
    if (pos_ < text_.size() && text_[pos_] == '-') {
      ++pos_;
    }
    while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_])) != 0) {
      ++pos_;
    }
    if (pos_ == start || (pos_ == start + 1 && text_[start] == '-')) {
      fail("expected an integer");
    }
    return std::stoll(text_.substr(start, pos_ - start));
  }

  void skip_space() {
    while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_])) != 0) {
      ++pos_;
    }
  }

  bool consume(char c) {
    skip_space();
    if (pos_ < text_.size() && text_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }

  void expect(char c) {
    if (!consume(c)) {
      fail(std::string("expected '") + c + "'");
    }
  }

  [[noreturn]] void fail(const std::string& what) const {
    throw std::runtime_error(origin_ + " at byte " + std::to_string(pos_) + ": " + what);
  }

  std::string text_;
  std::string origin_;
  std::size_t pos_{0};
};

inline std::vector<Vector> load(const std::string& family) {
  const std::string path = std::string(CANOPEN_CODEC_GOLDEN_DIR) + "/" + family + ".json";
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return Parser(buffer.str(), path).vectors();
}

// The frame a vector describes; bytes at and beyond dlc stay zero, as the codec encodes them.
inline canopen_codec::Frame frame_of(const Vector& v) {
  if (v.dlc < 0 || v.dlc > 8 || static_cast<std::size_t>(v.dlc) != v.data.size()) {
    throw std::runtime_error(v.name + ": dlc does not match data");
  }
  canopen_codec::Frame f;
  f.cob_id = static_cast<std::uint16_t>(v.cob_id);
  f.dlc = static_cast<std::uint8_t>(v.dlc);
  for (std::size_t i = 0; i < v.data.size(); ++i) {
    f.data.at(i) = static_cast<std::uint8_t>(v.data[i]);
  }
  return f;
}

}  // namespace golden
