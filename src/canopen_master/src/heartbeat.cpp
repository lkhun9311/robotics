// SPDX-License-Identifier: Apache-2.0
#include "canopen_master/heartbeat.hpp"

namespace canopen_master {

using runtime_core::Duration;
using runtime_core::MonoTime;

HeartbeatConsumer::HeartbeatConsumer(Duration consumer_time, Duration boot_timeout)
    : consumer_time_(consumer_time.ns > 0 ? consumer_time : Duration{}),
      boot_timeout_(boot_timeout),
      status_(consumer_time.ns > 0 ? HeartbeatStatus::kIdle : HeartbeatStatus::kDisabled) {}

void HeartbeatConsumer::start(MonoTime now) {
  if (status_ == HeartbeatStatus::kDisabled) {
    return;
  }
  status_ = HeartbeatStatus::kAwaitingBoot;
  deadline_ = now + boot_timeout_;
}

void HeartbeatConsumer::set_consumer_time(Duration consumer_time, MonoTime now) {
  if (consumer_time.ns <= 0) {
    consumer_time_ = Duration{};
    status_ = HeartbeatStatus::kDisabled;
    return;
  }
  const bool was_disabled = status_ == HeartbeatStatus::kDisabled;
  consumer_time_ = consumer_time;
  if (was_disabled) {
    status_ = HeartbeatStatus::kIdle;
    start(now);
  } else if (status_ == HeartbeatStatus::kAlive) {
    deadline_ = last_heartbeat_ + consumer_time_;
  }
}

HeartbeatChange HeartbeatConsumer::on_heartbeat(MonoTime now) {
  HeartbeatChange change = HeartbeatChange::kNone;
  switch (status_) {
    case HeartbeatStatus::kDisabled:
    case HeartbeatStatus::kIdle:
      return HeartbeatChange::kNone;
    case HeartbeatStatus::kAwaitingBoot:
      change = HeartbeatChange::kFirst;
      break;
    case HeartbeatStatus::kBootTimedOut:
    case HeartbeatStatus::kLost:
      change = HeartbeatChange::kResumed;
      break;
    case HeartbeatStatus::kAlive:
      break;
  }
  status_ = HeartbeatStatus::kAlive;
  last_heartbeat_ = now;
  deadline_ = now + consumer_time_;
  return change;
}

HeartbeatChange HeartbeatConsumer::expire(MonoTime now) {
  if (now < deadline_) {
    return HeartbeatChange::kNone;
  }
  if (status_ == HeartbeatStatus::kAwaitingBoot) {
    status_ = HeartbeatStatus::kBootTimedOut;
    return HeartbeatChange::kBootTimeout;
  }
  if (status_ == HeartbeatStatus::kAlive) {
    status_ = HeartbeatStatus::kLost;
    return HeartbeatChange::kLost;
  }
  return HeartbeatChange::kNone;
}

}  // namespace canopen_master
