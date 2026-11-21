// SPDX-License-Identifier: Apache-2.0
// The plugin library and a detached worker (C13 teardown, carried from Task 6).
// This executable does not link the plugin: it loads it through class_loader as pluginlib does,
// destroys a CanopenArmHw whose worker is stalled (the destructor detaches it after 1 s), unloads
// the library and only then lets the worker go on; the worker must end cleanly and the process must
// not crash.
//
// What this can and cannot show.
// In this stack the unload never unmaps the library, whatever the plugin does: GCC gives it
// STB_GNU_UNIQUE symbols (inline and template statics), which glibc never unloads, and class_loader
// refuses every unload once a "non-pure" plugin library was opened in the process
// (hardware_interface's initialization opens the urdf parser plugins).
// So this test cannot fail for a missing pin; the plugin pins itself (RTLD_NODELETE) for a stack
// where the unload would be real, and ArmSystem.DestructorWithStalledWorkerDoesNotCrash checks that
// the pin is taken.
// Here the whole pluginlib path is exercised end to end.
#include <dlfcn.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "class_loader/class_loader.hpp"
#include "drive_bus_transport.hpp"
#include "fixture.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace {

namespace at = arm_hw::test;
using Ms = std::chrono::milliseconds;
using CallbackReturn = hardware_interface::CallbackReturn;
using Factory = std::unique_ptr<canopen_backend::Transport> (*)(const std::string&, std::string*);

std::shared_ptr<at::DriveBusSim> g_sim;

std::unique_ptr<canopen_backend::Transport> sim_transport(const std::string& /*iface*/,
                                                          std::string* /*error*/) {
  return std::make_unique<at::DriveBusTransport>(g_sim);
}

bool mapped() {
  void* h = ::dlopen(ARM_HW_LIBRARY, RTLD_NOW | RTLD_NOLOAD);
  if (h == nullptr) return false;
  ::dlclose(h);
  return true;
}

}  // namespace

TEST(ArmHwUnload, DetachedWorkerOutlivesTheUnload) {
  g_sim = std::make_shared<at::DriveBusSim>();
  std::this_thread::sleep_for(Ms(200));  // past the drives' boot quarantine
  ASSERT_FALSE(mapped()) << "INCONCLUSIVE: the plugin library is loaded before the test";
  {
    auto loader = std::make_unique<class_loader::ClassLoader>(ARM_HW_LIBRARY);
    void* self = ::dlopen(ARM_HW_LIBRARY, RTLD_NOW | RTLD_NOLOAD);
    ASSERT_NE(self, nullptr) << "INCONCLUSIVE: class_loader did not load the library";
    auto* set =
        reinterpret_cast<void (*)(Factory)>(::dlsym(self, "arm_hw_testing_set_transport_factory"));
    ::dlclose(self);
    ASSERT_NE(set, nullptr) << ::dlerror();
    set(&sim_transport);
    auto hw =
        loader->createUniqueInstance<hardware_interface::SystemInterface>("arm_hw::CanopenArmHw");
    at::UrdfOptions o;
    o.plugin = "arm_hw/CanopenArmHw";
    ASSERT_EQ(hw->init(at::component_params(o)), CallbackReturn::SUCCESS);
    std::ignore = hw->on_export_state_interfaces();
    std::ignore = hw->on_export_command_interfaces();
    ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    g_sim->hold();
    ASSERT_TRUE(g_sim->wait_blocked(Ms(1000))) << "INCONCLUSIVE: the worker never blocked";
    hw.reset();  // ~1 s: the worker is detached, still blocked inside the plugin's code.
    ASSERT_EQ(g_sim->destroyed_count(), 0) << "INCONCLUSIVE: the worker did not outlive the object";
    loader.reset();  // class_loader's unload.
  }
  EXPECT_TRUE(mapped()) << "the plugin library was unmapped while its worker ran";
  // The worker returns into the library's code, ends and frees its state.
  g_sim->unhold();
  EXPECT_TRUE(g_sim->wait_destroyed(1, Ms(3000))) << "the detached worker never exited";
  EXPECT_EQ(g_sim->releases(), 1) << "the detached worker did not release its transport";
  g_sim.reset();
}
