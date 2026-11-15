// SPDX-License-Identifier: Apache-2.0
// Runs the backend contract suite, unchanged, against DirectBackend.
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "direct_backend/direct_backend.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/testing/contract_suite.hpp"
#include "runtime_core/time.hpp"

namespace db = direct_backend;
namespace hc = hal_contract;
namespace ht = hal_contract::testing;
namespace rc = runtime_core;

namespace {

class DirectFixture {
 public:
  struct Traits {
    static constexpr rc::Duration kArmLatency = rc::milliseconds(2);         // 1 W from rest
    static constexpr rc::Duration kAcceptanceLatency = rc::milliseconds(2);  // 1 W (C19)
  };

  std::unique_ptr<hc::Backend> make() {
    db::DirectConfig config{};
    config.start_position = ht::kStartPosition;
    config.max_feedback_age = ht::kMaxFeedbackAge;
    auto backend = std::make_unique<db::DirectBackend>(config);
    backend->set_event_sink(&DirectFixture::collect, this);
    backend_ = backend.get();
    return backend;
  }
  rc::MonoTime now() const { return now_; }
  void advance(rc::Duration d) {
    const rc::MonoTime until = now_ + d;
    while (next_step_ <= until) {
      backend_->step(next_step_);
      next_step_ = next_step_ + ht::kW;
    }
    now_ = until;
  }
  void stall_feedback(bool on) { backend_->stall_feedback(on); }
  void delay_feedback(rc::Duration d) { backend_->delay_feedback(d); }
  void stop_accepting_commands() { backend_->drop_commands(); }
  void inject_node_stop(std::size_t node) { backend_->inject_stop(node); }
  ht::DeviceState device() const {
    return ht::DeviceState{backend_->joint_position(), backend_->joint_velocity()};
  }
  std::vector<ht::DeviceEvent> drain_events() {
    std::vector<ht::DeviceEvent> out;
    out.swap(events_);
    return out;
  }

 private:
  static ht::DeviceEventKind translate(db::EventKind k) {
    switch (k) {
      case db::EventKind::kActuated:
        return ht::DeviceEventKind::kActuated;
      case db::EventKind::kDisabled:
        return ht::DeviceEventKind::kDisabled;
      case db::EventKind::kCommitted:
        return ht::DeviceEventKind::kCommitted;
      case db::EventKind::kEnabled:
        return ht::DeviceEventKind::kEnabled;
      case db::EventKind::kLocalStop:
        return ht::DeviceEventKind::kLocalStop;
      case db::EventKind::kQuickStop:
        return ht::DeviceEventKind::kQuickStop;
    }
    return ht::DeviceEventKind::kActuated;
  }
  static void collect(void* self, const db::Event& e) {
    static_cast<DirectFixture*>(self)->events_.push_back(ht::DeviceEvent{
        e.at, e.node, translate(e.kind), e.target, e.position, e.velocity, e.source_seq, e.id});
  }

  rc::MonoTime now_{1'000'000'000};
  rc::MonoTime next_step_{now_ + ht::kW};
  db::DirectBackend* backend_{nullptr};  // owned by the Hal under test
  std::vector<ht::DeviceEvent> events_;
};

static_assert(ht::BackendFixture<DirectFixture>);

}  // namespace

namespace hal_contract::testing {
using DirectFixtures = ::testing::Types<DirectFixture>;
INSTANTIATE_TYPED_TEST_SUITE_P(Direct, Contract, DirectFixtures);
}  // namespace hal_contract::testing
