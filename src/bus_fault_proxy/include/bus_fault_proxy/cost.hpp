// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

// The distribution of the proxy's normal-path forwarding cost (B10), in the form the exit record
// carries it: every distinct sample value with how often it occurred.
// It is exact (no binning), so a histogram can be drawn from it at any bin width.
namespace bus_fault_proxy {

struct ValueCount {
  std::int64_t value{};
  std::uint64_t count{};
};

// The distinct values of `sorted` (ascending order required) with their counts, ascending.
std::vector<ValueCount> run_lengths(const std::vector<std::int64_t>& sorted);

}  // namespace bus_fault_proxy
