// SPDX-License-Identifier: Apache-2.0
// Runs the backend contract suite, unchanged, against CanopenBackend over an in-memory transport
// to six drive_core::Drive, in logic time.
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "canopen_backend/canopen_backend.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/testing/contract_suite.hpp"
#include "rig.hpp"
#include "runtime_core/time.hpp"

namespace cbt = canopen_backend::testing;
namespace hc = hal_contract;
namespace ht = hal_contract::testing;
namespace rc = runtime_core;

namespace {

class CanopenFixture {
 public:
  struct Traits {
    // A stated bound, not a measured distribution: from rest the arm is the commit SDOs, two
    // hold updates (2 x T_bus), the enable, then the SYNCs that report the armed tag (C2: within
    // 4 x T_bus of the enable), about 8 x T_bus; 100 ms leaves the rest as margin.
    static constexpr rc::Duration kArmLatency = rc::milliseconds(100);
    static constexpr rc::Duration kAcceptanceLatency = rc::milliseconds(20);  // 2 x T_bus (C19)
  };

  CanopenFixture() { rig_.place(ht::kStartPosition); }

  std::unique_ptr<hc::Backend> make() { return rig_.make_backend(); }
  rc::MonoTime now() const { return rig_.now; }
  void advance(rc::Duration d) { rig_.advance(d); }
  void stall_feedback(bool on) { rig_.stall_samples = on; }
  void delay_feedback(rc::Duration d) { rig_.delay = d; }
  void stop_accepting_commands() { rig_.drop_commands = true; }
  void inject_node_stop(std::size_t node) { rig_.inject_stop(node); }
  ht::DeviceState device() const { return ht::DeviceState{rig_.position(), rig_.velocity()}; }
  std::vector<ht::DeviceEvent> drain_events() {
    std::vector<ht::DeviceEvent> out;
    out.reserve(rig_.events.size());
    for (const cbt::RigEvent& e : rig_.events) {
      out.push_back(ht::DeviceEvent{e.at, e.node, translate(e.kind), e.target, e.position,
                                    e.velocity, e.source_seq, e.id});
    }
    rig_.events.clear();
    return out;
  }

 private:
  static ht::DeviceEventKind translate(cbt::RigEventKind k) {
    switch (k) {
      case cbt::RigEventKind::kActuated:
        return ht::DeviceEventKind::kActuated;
      case cbt::RigEventKind::kDisabled:
        return ht::DeviceEventKind::kDisabled;
      case cbt::RigEventKind::kCommitted:
        return ht::DeviceEventKind::kCommitted;
      case cbt::RigEventKind::kEnabled:
        return ht::DeviceEventKind::kEnabled;
      case cbt::RigEventKind::kLocalStop:
        return ht::DeviceEventKind::kLocalStop;
      case cbt::RigEventKind::kQuickStop:
        return ht::DeviceEventKind::kQuickStop;
    }
    return ht::DeviceEventKind::kActuated;
  }

  cbt::Rig rig_;
};

static_assert(ht::BackendFixture<CanopenFixture>);

}  // namespace

namespace hal_contract::testing {
using CanopenFixtures = ::testing::Types<CanopenFixture>;
INSTANTIATE_TYPED_TEST_SUITE_P(Canopen, Contract, CanopenFixtures);
}  // namespace hal_contract::testing
