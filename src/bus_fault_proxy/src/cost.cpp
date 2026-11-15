// SPDX-License-Identifier: Apache-2.0
#include "bus_fault_proxy/cost.hpp"

namespace bus_fault_proxy {

std::vector<ValueCount> run_lengths(const std::vector<std::int64_t>& sorted) {
  std::vector<ValueCount> out;
  for (const std::int64_t v : sorted) {
    if (out.empty() || out.back().value != v) {
      out.push_back(ValueCount{v, 0});
    }
    ++out.back().count;
  }
  return out;
}

}  // namespace bus_fault_proxy
