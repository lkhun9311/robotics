// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/trace.hpp"

#include <cstdio>
#include <utility>

namespace runtime_core {

std::string json_escape(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    if (c == '"') {
      out += "\\\"";
    } else if (c == '\\') {
      out += "\\\\";
    } else if (c < 0x20) {
      char buf[7];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
      out += buf;
    } else {
      out += ch;
    }
  }
  return out;
}

JsonlTraceSink::JsonlTraceSink(std::ostream& out, std::string clock_domain)
    : out_(out), clock_domain_(std::move(clock_domain)) {}

void JsonlTraceSink::emit(MonoTime t, std::string_view kind,
                          std::initializer_list<TraceField> fields) {
  std::string line = "{\"t_ns\":" + std::to_string(t.ns) + ",\"clock\":\"" +
                     json_escape(clock_domain_) + "\",\"kind\":\"" + json_escape(kind) + "\"";
  for (const TraceField& field : fields) {
    line += ",\"" + json_escape(field.key) + "\":";
    if (const auto* number = std::get_if<std::int64_t>(&field.value)) {
      line += std::to_string(*number);
    } else {
      line += "\"" + json_escape(std::get<std::string_view>(field.value)) + "\"";
    }
  }
  line += "}\n";
  out_ << line;
}

}  // namespace runtime_core
