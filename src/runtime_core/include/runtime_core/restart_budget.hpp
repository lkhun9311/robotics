// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <deque>

#include "runtime_core/time.hpp"

namespace runtime_core {

// Sliding-window restart budget (spec §8.4). There is deliberately no reset method:
// reconnecting or restarting a peer must not refill the budget.
class RestartBudget {
 public:
  RestartBudget(std::size_t max_restarts, Duration window);
  bool try_consume(MonoTime now);
  std::size_t used(MonoTime now);

 private:
  void prune(MonoTime now);

  std::size_t max_restarts_;
  Duration window_;
  std::deque<MonoTime> attempts_;
};

}  // namespace runtime_core
