// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <initializer_list>
#include <ostream>
#include <string>
#include <string_view>
#include <variant>

#include "runtime_core/time.hpp"

namespace runtime_core {

struct TraceField {
  std::string_view key;
  std::variant<std::int64_t, std::string_view> value;
};

class TraceSink {
 public:
  virtual ~TraceSink() = default;
  virtual void emit(MonoTime t, std::string_view kind,
                    std::initializer_list<TraceField> fields) = 0;
};

// One JSON object per line. The clock domain is written on every line so a reader
// never subtracts timestamps from different clocks (spec §18.5).
class JsonlTraceSink final : public TraceSink {
 public:
  JsonlTraceSink(std::ostream& out, std::string clock_domain);
  void emit(MonoTime t, std::string_view kind, std::initializer_list<TraceField> fields) override;

 private:
  std::ostream& out_;
  std::string clock_domain_;
};

std::string json_escape(std::string_view s);

}  // namespace runtime_core
