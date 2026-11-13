// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/restart_budget.hpp"

namespace runtime_core {

RestartBudget::RestartBudget(std::size_t max_restarts, Duration window)
    : max_restarts_(max_restarts), window_(window) {}

void RestartBudget::prune(MonoTime now) {
  while (!attempts_.empty() && (now - attempts_.front()) >= window_) attempts_.pop_front();
}

bool RestartBudget::try_consume(MonoTime now) {
  prune(now);
  if (attempts_.size() >= max_restarts_) return false;
  attempts_.push_back(now);
  return true;
}

std::size_t RestartBudget::used(MonoTime now) {
  prune(now);
  return attempts_.size();
}

}  // namespace runtime_core
