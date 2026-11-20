// SPDX-License-Identifier: Apache-2.0
// The arm_hw plugins as components (C12, C13): each test parses the test URDF (fixture.hpp) into a
// HardwareInfo, exports the handles, sets command handles and calls the lifecycle callbacks and
// read()/write() directly (no controller), or goes through hardware_interface::ResourceManager
// where ros2_control's own sequencing matters (on_error from read()). CanopenArmHw runs its real
// worker thread over the wall-clock in-memory drive bus (drive_bus_transport.hpp).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "arm_hw/arm_system.hpp"
#include "arm_hw/interfaces.hpp"
#include "direct_backend/direct_backend.hpp"
#include "drive_bus_transport.hpp"
#include "fixture.hpp"
#include "hal_contract/faults.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcutils/logging.h"
#include "runtime_core/time.hpp"

namespace {

namespace ah = arm_hw;
namespace at = arm_hw::test;
namespace hc = hal_contract;
namespace hi = hardware_interface;
namespace rc = runtime_core;
using CallbackReturn = hi::CallbackReturn;
using Ms = std::chrono::milliseconds;

constexpr Ms kWatchdog{5000};

// ---- the log, as ros2_control's logger writes it ----

std::mutex g_log_mu;
std::string g_log;
rcutils_logging_output_handler_t g_default_handler = nullptr;

void capture(const rcutils_log_location_t* location, int severity, const char* name,
             rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
  va_list copy;
  va_copy(copy, *args);
  char buf[2048];
  std::vsnprintf(buf, sizeof buf, format, copy);
  va_end(copy);
  {
    std::lock_guard<std::mutex> lock(g_log_mu);
    g_log += buf;
    g_log += '\n';
  }
  if (g_default_handler != nullptr)
    g_default_handler(location, severity, name, timestamp, format, args);
}

std::string take_log() {
  std::lock_guard<std::mutex> lock(g_log_mu);
  std::string out;
  out.swap(g_log);
  return out;
}

class LogCapture {
 public:
  LogCapture() {
    g_default_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture);
    take_log();
  }
  ~LogCapture() { rcutils_logging_set_output_handler(g_default_handler); }
  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;
};

// ---- the transport hook of CanopenArmHw ----

std::shared_ptr<at::DriveBusSim> g_sim;

std::unique_ptr<canopen_backend::Transport> sim_transport(const std::string& /*iface*/,
                                                          std::string* error) {
  if (!g_sim) {
    *error = "test: no drive bus";
    return nullptr;
  }
  return std::make_unique<at::DriveBusTransport>(g_sim);
}

class SimBus {
 public:
  SimBus() {
    g_sim = std::make_shared<at::DriveBusSim>();
    // Past the drives' boot quarantine (100 ms), as a bench looks before a test.
    std::this_thread::sleep_for(Ms(200));
    ah::testing::set_transport_factory(&sim_transport);
  }
  ~SimBus() {
    g_sim->unhold();
    ah::testing::set_transport_factory(nullptr);
    g_sim.reset();
  }
  SimBus(const SimBus&) = delete;
  SimBus& operator=(const SimBus&) = delete;
  at::DriveBusSim& operator*() { return *g_sim; }
  at::DriveBusSim* operator->() { return g_sim.get(); }
};

rc::MonoTime mono_now() { return rc::SystemMonotonicClock{}.now(); }
std::uint64_t ns(rc::MonoTime t) { return static_cast<std::uint64_t>(t.ns); }

// ---- one component, driven directly ----

class Component {
 public:
  explicit Component(std::unique_ptr<ah::ArmSystem> sys, const at::UrdfOptions& o = {})
      : sys_(std::move(sys)) {
    init_result_ = sys_->init(at::component_params(o));
    if (init_result_ != CallbackReturn::SUCCESS) return;
    for (const auto& s : sys_->on_export_state_interfaces()) states_[s->get_name()] = s;
    for (const auto& c : sys_->on_export_command_interfaces()) commands_[c->get_name()] = c;
  }

  CallbackReturn init_result() const { return init_result_; }
  ah::ArmSystem& operator*() { return *sys_; }
  ah::ArmSystem* operator->() { return sys_.get(); }
  std::unique_ptr<ah::ArmSystem>& owner() { return sys_; }

  CallbackReturn configure() { return sys_->on_configure(state()); }
  CallbackReturn activate() { return sys_->on_activate(state()); }
  CallbackReturn deactivate() { return sys_->on_deactivate(state()); }
  CallbackReturn cleanup() { return sys_->on_cleanup(state()); }
  CallbackReturn error() { return sys_->on_error(state()); }
  hi::return_type read() { return sys_->read(rclcpp::Time(), rclcpp::Duration(0, 0)); }
  hi::return_type write() { return sys_->write(rclcpp::Time(), rclcpp::Duration(0, 0)); }

  void set_wide(std::string_view gpio, std::string_view field, std::uint64_t v) {
    const ah::Halves h = ah::split(v);
    ASSERT_TRUE(cmd(gpio, std::string(field) + "_lo")->set_value(h.lo, true));
    ASSERT_TRUE(cmd(gpio, std::string(field) + "_hi")->set_value(h.hi, true));
  }
  std::uint64_t cmd_wide(std::string_view gpio, std::string_view field) {
    ah::Halves h;
    EXPECT_TRUE(cmd(gpio, std::string(field) + "_lo")->get_value(h.lo, true));
    EXPECT_TRUE(cmd(gpio, std::string(field) + "_hi")->get_value(h.hi, true));
    return ah::join(h);
  }
  void set_present(std::uint8_t v) { ASSERT_TRUE(cmd("permit", "present")->set_value(v, true)); }
  std::uint8_t present() {
    std::uint8_t v = 0xFF;
    EXPECT_TRUE(cmd("permit", "present")->get_value(v, true));
    return v;
  }
  void set_joints(const hc::Joints& q) {
    for (std::size_t i = 0; i < q.size(); ++i) {
      ASSERT_TRUE(
          commands_.at(std::string(at::kJointNames.at(i)) + "/position")->set_value(q.at(i), true));
    }
  }
  std::uint8_t hal8(std::string_view name) {
    std::uint8_t v = 0xFF;
    EXPECT_TRUE(states_.at("hal/" + std::string(name))->get_value(v, true));
    return v;
  }
  std::uint32_t hal32(std::string_view name) {
    std::uint32_t v = 0;
    EXPECT_TRUE(states_.at("hal/" + std::string(name))->get_value(v, true));
    return v;
  }
  std::uint64_t hal_wide(std::string_view field) {
    return ah::join(
        ah::Halves{hal32(std::string(field) + "_lo"), hal32(std::string(field) + "_hi")});
  }
  hc::HalVerdict verdict() { return static_cast<hc::HalVerdict>(hal8("verdict")); }
  hc::Joints joints() {
    hc::Joints q{};
    for (std::size_t i = 0; i < q.size(); ++i) {
      EXPECT_TRUE(
          states_.at(std::string(at::kJointNames.at(i)) + "/position")->get_value(q.at(i), true));
    }
    return q;
  }

  // An arm request under identity (1, 1, 1), holding 500 ms.
  void request_arm(std::uint64_t request_id) {
    set_wide("arm", "manager", 1);
    set_wide("arm", "session", 1);
    set_wide("arm", "generation", 1);
    set_wide("arm", "hold_until_ns", ns(mono_now() + rc::milliseconds(500)));
    set_wide("arm", "request_id", request_id);
  }
  // A permit for (1, 1, 1) open for 100 ms.
  void grant_permit() {
    set_wide("permit", "manager", 1);
    set_wide("permit", "session", 1);
    set_wide("permit", "generation", 1);
    set_wide("permit", "expires_ns", ns(mono_now() + rc::milliseconds(100)));
    set_present(1);
  }
  void setpoint(std::uint64_t seq, const hc::Joints& q) {
    set_wide("setpoint", "manager", 1);
    set_wide("setpoint", "session", 1);
    set_wide("setpoint", "generation", 1);
    set_wide("setpoint", "seq", seq);
    set_joints(q);
  }
  // Reads every 2 ms until hal/arm_state is `code` or `limit` passed.
  bool read_until_arm_state(ah::ArmCode code, Ms limit) {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      read();
      if (hal8("arm_state") == static_cast<std::uint8_t>(code)) return true;
      std::this_thread::sleep_for(Ms(2));
    }
    return false;
  }
  // configure, activate, arm with request id 7.
  void arm() {
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS) << sys_->last_error();
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS) << sys_->last_error();
    request_arm(7);
    ASSERT_EQ(write(), hi::return_type::OK);
    ASSERT_TRUE(read_until_arm_state(ah::ArmCode::kArmed, Ms(1000)))
        << "INCONCLUSIVE: the arm never completed (arm_state " << int{hal8("arm_state")} << ")";
  }

 private:
  static rclcpp_lifecycle::State state() { return rclcpp_lifecycle::State(); }
  hi::CommandInterface::SharedPtr cmd(std::string_view gpio, std::string_view name) {
    return commands_.at(std::string(gpio) + "/" + std::string(name));
  }

  std::unique_ptr<ah::ArmSystem> sys_;
  CallbackReturn init_result_{CallbackReturn::ERROR};
  std::map<std::string, hi::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hi::CommandInterface::SharedPtr> commands_;
};

at::UrdfOptions canopen_urdf() {
  at::UrdfOptions o;
  o.plugin = "arm_hw/CanopenArmHw";
  return o;
}

std::unique_ptr<ah::ArmSystem> direct() { return std::make_unique<ah::DirectArmHw>(); }
std::unique_ptr<ah::ArmSystem> canopen() { return std::make_unique<ah::CanopenArmHw>(); }

// A call that must not block: on a timeout the process ends (the blocked thread cannot be
// joined), which fails the test rather than hanging the suite.
template <class F>
void must_return(const char* what, F f) {
  if (!at::returns_within(kWatchdog, std::move(f))) {
    std::fprintf(stderr, "FAIL: %s did not return within %lld ms\n", what,
                 static_cast<long long>(kWatchdog.count()));
    std::_Exit(3);
  }
}

}  // namespace

// ---- interfaces ----

TEST(ArmSystem, HalvesRoundTripExactly) {
  for (const std::uint64_t v :
       {std::uint64_t{0}, std::uint64_t{1}, (std::uint64_t{1} << 53U) + 1U,
        std::numeric_limits<std::uint64_t>::max(), std::uint64_t{0x8000'0000'0000'0000ULL}}) {
    EXPECT_EQ(ah::join(ah::split(v)), v) << v;
  }
  // The point of the halves: a double cannot carry 2^53 + 1.
  const std::uint64_t odd = (std::uint64_t{1} << 53U) + 1U;
  EXPECT_NE(static_cast<std::uint64_t>(static_cast<double>(odd)), odd);
  EXPECT_EQ(ah::split(std::numeric_limits<std::uint64_t>::max()),
            (ah::Halves{0xFFFF'FFFFU, 0xFFFF'FFFFU}));
  // And through real uint32 handles of the exported interfaces.
  Component c(direct());
  ASSERT_EQ(c.init_result(), CallbackReturn::SUCCESS) << c->last_error();
  for (const std::uint64_t v : {odd, std::numeric_limits<std::uint64_t>::max()}) {
    c.set_wide("arm", "request_id", v);
    EXPECT_EQ(c.cmd_wide("arm", "request_id"), v);
  }
}

TEST(ArmSystem, GpioWithoutInitialValueIsRejected) {
  at::UrdfOptions o;
  o.omit_request_initial = true;
  LogCapture log;
  Component c(direct(), o);
  EXPECT_EQ(c.init_result(), CallbackReturn::ERROR);
  EXPECT_NE(c->last_error().find("arm/request_id_lo needs an explicit initial_value of 0"),
            std::string::npos)
      << c->last_error();
}

TEST(ArmSystem, OutOfProfileTimingRefused) {
  struct Case {
    const char* key;
    const char* value;
  };
  for (const Case k : {Case{"bus_period_ms", "9"}, Case{"bus_period_ms", "11"},
                       Case{"lease_ms", "0"}, Case{"lease_ms", "49"},
                       Case{"max_feedback_age_ms", "19"}, Case{"max_feedback_age_ms", "31"}}) {
    at::UrdfOptions o;
    o.params[k.key] = k.value;
    LogCapture log;
    Component c(direct(), o);
    ASSERT_EQ(c.init_result(), CallbackReturn::SUCCESS) << k.key << "=" << k.value;
    EXPECT_EQ(c.configure(), CallbackReturn::FAILURE) << k.key << "=" << k.value;
    EXPECT_NE(c->last_error().find(ah::kTimingProfileAllowed), std::string::npos)
        << c->last_error();
    EXPECT_NE(take_log().find(ah::kTimingProfileAllowed), std::string::npos)
        << "the allowed values are not in the log";
    EXPECT_EQ(c->hal(), nullptr) << "a refused configure built a Hal";
  }
  // The edges of the profile are accepted.
  at::UrdfOptions edge;
  edge.params["lease_ms"] = "1";
  edge.params["max_feedback_age_ms"] = "20";
  Component c(direct(), edge);
  EXPECT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
}

// ---- DirectArmHw ----

TEST(ArmSystem, UntouchedHandlesAreInert) {
  Component c(direct());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  // The exported command handles as the URDF initialized them.
  EXPECT_EQ(c.present(), 0);
  EXPECT_EQ(c.cmd_wide("arm", "request_id"), 0U);
  EXPECT_EQ(c.cmd_wide("permit", "expires_ns"), 0U);
  for (int k = 0; k < 20; ++k) {
    ASSERT_EQ(c.read(), hi::return_type::OK);
    ASSERT_EQ(c.write(), hi::return_type::OK);
    std::this_thread::sleep_for(Ms(2));
  }
  EXPECT_EQ(c->hal()->arm_state(), hc::ArmState::kIdle) << "an untouched arm/request_id armed";
  EXPECT_FALSE(c->hal()->adopted().has_value());
  EXPECT_EQ(c.hal8("arm_state"), static_cast<std::uint8_t>(ah::ArmCode::kIdle));
  EXPECT_EQ(c.verdict(), hc::HalVerdict::kNoPermit) << hc::to_string(c.verdict());
}

TEST(ArmSystem, ArmRequestPostsOpWithoutWaiting) {
  Component c(direct());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  c.request_arm(42);
  ASSERT_EQ(c.write(), hi::return_type::OK);
  // DirectArmHw advances only in read(): nothing has stepped, so the op can only be posted.
  EXPECT_EQ(c->hal()->arm_state(), hc::ArmState::kArming);
  // The same request id again posts nothing new.
  ASSERT_EQ(c.write(), hi::return_type::OK);
  ASSERT_TRUE(c.read_until_arm_state(ah::ArmCode::kArmed, Ms(1000)));
  EXPECT_EQ(c.hal_wide("armed_request_id"), 42U);
  EXPECT_EQ(c->hal()->armed_request_id(), 42U);
}

TEST(ArmSystem, MissingPermitIsNoPermitVerdict) {
  Component c(direct());
  c.arm();
  c.read();
  const hc::Joints at = c.joints();
  c.grant_permit();
  c.setpoint(1, at);
  c.read();
  ASSERT_EQ(c.write(), hi::return_type::OK);
  ASSERT_EQ(c.verdict(), hc::HalVerdict::kSend)
      << "INCONCLUSIVE: a valid write was refused: " << hc::to_string(c.verdict());
  std::uint64_t seq = 1;
  for (const std::uint8_t present : {std::uint8_t{0}, std::uint8_t{2}, std::uint8_t{255}}) {
    c.set_present(present);
    c.setpoint(++seq, at);
    c.read();
    EXPECT_EQ(c.write(), hi::return_type::OK);
    EXPECT_EQ(c.verdict(), hc::HalVerdict::kNoPermit)
        << "present = " << int{present} << ": " << hc::to_string(c.verdict());
  }
}

// The device takes an approved command no earlier than the write that approved it.
// read() replays the device steps since the previous read, so write() must catch up first.
TEST(ArmSystem, DirectCommandIsNeverTakenBeforeItsApproval) {
  struct Sink {
    std::vector<direct_backend::Event> events;
    static void put(void* self, const direct_backend::Event& e) {
      static_cast<Sink*>(self)->events.push_back(e);
    }
  };
  Component c(direct());
  c.arm();
  Sink sink;
  dynamic_cast<ah::DirectArmHw&>(*c).backend()->set_event_sink(&Sink::put, &sink);
  c.read();
  c.grant_permit();
  c.setpoint(1, c.joints());
  // Steps of the device are due since this read; the write comes 10 ms later.
  std::this_thread::sleep_for(Ms(10));
  const rc::MonoTime approved = mono_now();
  ASSERT_EQ(c.write(), hi::return_type::OK);
  ASSERT_EQ(c.verdict(), hc::HalVerdict::kSend) << "INCONCLUSIVE: " << hc::to_string(c.verdict());
  for (int k = 0; k < 5; ++k) {
    std::this_thread::sleep_for(Ms(2));
    c.read();
  }
  bool applied = false;
  for (const direct_backend::Event& e : sink.events) {
    if (e.kind != direct_backend::EventKind::kActuated || e.source_seq != 1U) continue;
    applied = true;
    EXPECT_GE(e.at, approved) << "applied " << (approved - e.at).ns << " ns before its approval";
  }
  ASSERT_TRUE(applied) << "INCONCLUSIVE: the command was never applied";
}

TEST(ArmSystem, GuardRefusalReturnsOk) {
  Component c(direct());
  c.arm();
  c.read();
  const hc::Joints at = c.joints();
  c.grant_permit();
  c.setpoint(1, at);
  c.read();
  ASSERT_EQ(c.write(), hi::return_type::OK);
  ASSERT_EQ(c.verdict(), hc::HalVerdict::kSend) << "INCONCLUSIVE: " << hc::to_string(c.verdict());
  hc::Joints beyond = at;
  beyond.at(2) = 7.0;  // past the URDF max of 6.28
  c.setpoint(2, beyond);
  c.read();
  EXPECT_EQ(c.write(), hi::return_type::OK) << "a guard refusal is a verdict, never an error";
  EXPECT_EQ(c.verdict(), hc::HalVerdict::kPositionLimit) << hc::to_string(c.verdict());
  for (int k = 0; k < 20; ++k) {
    EXPECT_EQ(c.read(), hi::return_type::OK);
    std::this_thread::sleep_for(Ms(2));
  }
  EXPECT_LT(std::abs(dynamic_cast<ah::DirectArmHw&>(*c).backend()->joint_position().at(2)), 0.1)
      << "the refused target reached the joint";
}

TEST(ArmSystem, ActivationRefusedReturnsFailure) {
  at::UrdfOptions o;
  o.params["backend_profile_limit"] = "generic_cia402";
  o.params["required_capabilities"] = "all";
  LogCapture log;
  Component c(direct(), o);
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  EXPECT_EQ(c.activate(), CallbackReturn::FAILURE);
  const std::string text = take_log();
  EXPECT_NE(text.find("missing capabilities"), std::string::npos) << text;
  EXPECT_NE(text.find("DEVICE_SIDE_EXPIRY"), std::string::npos) << text;
  // Without the limit the same requirements pass on the direct profile.
  at::UrdfOptions plain;
  plain.params["required_capabilities"] = "all";
  Component d(direct(), plain);
  ASSERT_EQ(d.configure(), CallbackReturn::SUCCESS) << d->last_error();
  EXPECT_EQ(d.activate(), CallbackReturn::SUCCESS) << d->last_error();
}

// Carried from Task 5: Hal::configure clears the Hal's state before a backend can refuse a
// configure while armed, so a cycle's Hal is never reconfigured: on_configure refuses while active
// and a new cycle builds a new Hal.
TEST(ArmSystem, ConfigureWhileActiveIsRefused) {
  Component c(direct());
  c.arm();
  hc::Hal* armed = c->hal();
  EXPECT_EQ(c.configure(), CallbackReturn::FAILURE);
  EXPECT_NE(c->last_error().find("deactivate before configure"), std::string::npos)
      << c->last_error();
  EXPECT_EQ(c->hal(), armed);
  EXPECT_EQ(armed->arm_state(), hc::ArmState::kArmed) << "the refused configure touched the Hal";
  ASSERT_EQ(c.deactivate(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.cleanup(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  EXPECT_EQ(c->hal()->arm_state(), hc::ArmState::kIdle);
  EXPECT_FALSE(c->hal()->adopted().has_value());
  // After an error from an armed state, too: the next configure starts a fresh cycle.
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  c.request_arm(8);
  c.write();
  ASSERT_TRUE(c.read_until_arm_state(ah::ArmCode::kArmed, Ms(1000)));
  ASSERT_EQ(c.error(), CallbackReturn::SUCCESS);
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  EXPECT_EQ(c->hal()->arm_state(), hc::ArmState::kIdle);
}

// ---- CanopenArmHw over its worker thread ----

TEST(ArmSystem, TransportLossReturnsErrorOnce) {
  SimBus sim;
  Component c(canopen(), canopen_urdf());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.read(), hi::return_type::OK) << "INCONCLUSIVE: ERROR before the loss";
  sim->lose();
  ASSERT_TRUE(sim->wait_polls(3, Ms(1000))) << "INCONCLUSIVE: the worker stopped polling";
  int errors = 0;
  for (int k = 0; k < 20; ++k) {
    if (c.read() == hi::return_type::ERROR) ++errors;
    std::this_thread::sleep_for(Ms(2));
  }
  EXPECT_EQ(errors, 1);
  EXPECT_TRUE(hc::FaultSet(c.hal32("faults")).has(hc::Fault::kTransportLost));
  EXPECT_EQ(c.write(), hi::return_type::OK);
  EXPECT_EQ(c.verdict(), hc::HalVerdict::kInhibited) << hc::to_string(c.verdict());
  EXPECT_EQ(c.error(), CallbackReturn::SUCCESS);
}

TEST(ArmSystem, CleanupRequestsShutdown) {
  SimBus sim;
  Component c(canopen(), canopen_urdf());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.deactivate(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(sim->releases(), 0) << "INCONCLUSIVE: released before cleanup";
  EXPECT_EQ(c.cleanup(), CallbackReturn::SUCCESS) << c->last_error();
  EXPECT_EQ(sim->releases(), 1) << "the worker did not release (close) its transport";
  EXPECT_EQ(sim->destroyed_count(), 1) << "the worker state outlived cleanup";
  EXPECT_EQ(c->hal(), nullptr);
  // One Worker per configure cycle: the next configure makes a new one.
  EXPECT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  EXPECT_EQ(sim->created_count(), 2);
}

TEST(ArmSystem, DestructorWithStalledWorkerDoesNotCrash) {
  SimBus sim;
  Component c(canopen(), canopen_urdf());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  sim->hold();
  ASSERT_TRUE(sim->wait_blocked(Ms(1000))) << "INCONCLUSIVE: the worker never blocked";
  std::unique_ptr<ah::ArmSystem>& owner = c.owner();
  const auto t0 = std::chrono::steady_clock::now();
  must_return("the destructor", [&] { owner.reset(); });
  const auto took = std::chrono::steady_clock::now() - t0;
  std::printf("destructor with a stalled worker took %.0f ms\n",
              std::chrono::duration<double, std::milli>(took).count());
  EXPECT_GE(took, Ms(990)) << "the destructor did not give the worker its 1 s";
  EXPECT_TRUE(ah::CanopenArmHw::library_pinned());
  EXPECT_EQ(sim->destroyed_count(), 0) << "INCONCLUSIVE: the worker state went with the object";
  // The detached worker ends once released, and frees what it owns.
  sim->unhold();
  EXPECT_TRUE(sim->wait_destroyed(1, Ms(3000))) << "the detached worker never exited";
  EXPECT_EQ(sim->releases(), 1);
}

TEST(CanopenArmHw, WriteDoesNotBlockOnStalledWorker) {
  SimBus sim;
  Component c(canopen(), canopen_urdf());
  ASSERT_EQ(c.configure(), CallbackReturn::SUCCESS) << c->last_error();
  ASSERT_EQ(c.activate(), CallbackReturn::SUCCESS) << c->last_error();
  c.read();
  sim->hold();
  ASSERT_TRUE(sim->wait_blocked(Ms(1000))) << "INCONCLUSIVE: the worker never blocked";
  // Every path of write(): a new arm request each call (each posted to the blocked worker; past
  // the op queue's 8 they are refused there), a permit, a setpoint.
  c.request_arm(5);
  c.grant_permit();
  std::vector<double> us;
  us.reserve(1000);
  must_return("1000 write() calls", [&] {
    for (std::uint64_t k = 1; k <= 1000; ++k) {
      c.setpoint(k, hc::Joints{});
      c.set_wide("arm", "request_id", 1000 + k);
      const auto t0 = std::chrono::steady_clock::now();
      const hi::return_type r = c.write();
      us.push_back(
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
      if (r != hi::return_type::OK) ADD_FAILURE() << "write " << k << " returned ERROR";
      std::ignore = c.read();
    }
  });
  ASSERT_TRUE(sim->wait_blocked(Ms(0))) << "INCONCLUSIVE: the worker was released during the test";
  ASSERT_EQ(us.size(), 1000U);
  std::sort(us.begin(), us.end());
  std::printf("write() with a blocked worker, us: median %.1f p99 %.1f max %.1f\n", us.at(500),
              us.at(989), us.back());
  // Arm ops were posted (past the worker's op queue they fail at once), none completed.
  EXPECT_NE(c->hal()->arm_state(), hc::ArmState::kIdle) << "no arm op was posted";
  EXPECT_NE(c->hal()->arm_state(), hc::ArmState::kArmed);
  sim->unhold();
}

// ---- through hardware_interface::ResourceManager ----

namespace {

class Managed {
 public:
  Managed() : rm_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), rclcpp::get_logger("rm")) {
    hi::HardwareComponentParams p = at::component_params(canopen_urdf());
    name_ = p.hardware_info.name;
    rm_.import_component(std::make_unique<ah::CanopenArmHw>(), p);
  }
  hi::return_type to(std::uint8_t id, const char* label) {
    rclcpp_lifecycle::State s(id, label);
    return rm_.set_component_state(name_, s);
  }
  hi::return_type configure() {
    return to(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  }
  hi::return_type activate() {
    return to(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  }
  std::uint8_t state() { return rm_.get_components_status().at(name_).state.id(); }
  hi::HardwareReadWriteStatus read() {
    return rm_.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 10'000'000));
  }

 private:
  hi::ResourceManager rm_;
  std::string name_;
};

// Configure and activate through the manager, then a transport loss the worker publishes, then
// the worker held inside its next wake.
void lose_then_block(Managed& m, at::DriveBusSim& sim) {
  ASSERT_EQ(m.configure(), hi::return_type::OK);
  ASSERT_EQ(m.activate(), hi::return_type::OK);
  ASSERT_EQ(m.read().result, hi::return_type::OK) << "INCONCLUSIVE: ERROR before the loss";
  sim.lose();
  ASSERT_TRUE(sim.wait_polls(3, Ms(1000))) << "INCONCLUSIVE: the worker stopped polling";
  sim.hold();
  ASSERT_TRUE(sim.wait_blocked(Ms(1000))) << "INCONCLUSIVE: the worker never blocked";
}

}  // namespace

TEST(CanopenArmHw, OnErrorThroughResourceManagerDoesNotBlock) {
  SimBus sim;
  Managed m;
  lose_then_block(m, *sim);
  hi::HardwareReadWriteStatus status{};
  must_return("ResourceManager::read with on_error", [&] { status = m.read(); });
  EXPECT_EQ(status.result, hi::return_type::ERROR);
  EXPECT_EQ(m.state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      << "on_error did not take the component to UNCONFIGURED";
  ASSERT_TRUE(sim->wait_blocked(Ms(0))) << "INCONCLUSIVE: the worker was not blocked throughout";
  sim->unhold();
}

TEST(ArmSystem, ErrorThenConfigureWaitsForTeardown) {
  SimBus sim;
  Managed m;
  lose_then_block(m, *sim);
  must_return("ResourceManager::read with on_error", [&] { std::ignore = m.read(); });
  ASSERT_EQ(m.state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  // The teardown is held: the previous cycle's worker still runs, so configure must fail.
  EXPECT_NE(m.configure(), hi::return_type::OK);
  EXPECT_EQ(m.state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(sim->created_count(), 1) << "a second worker was created while the first ran";
  sim->unhold();
  // Once that worker has exited, a configure succeeds on a new worker.
  bool configured = false;
  const auto end = std::chrono::steady_clock::now() + Ms(3000);
  while (!configured && std::chrono::steady_clock::now() < end) {
    if (sim->releases() == 1) configured = m.configure() == hi::return_type::OK;
    if (!configured) std::this_thread::sleep_for(Ms(20));
  }
  EXPECT_TRUE(configured) << "configure never succeeded after the teardown";
  EXPECT_EQ(m.state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(sim->created_count(), 2);
}
