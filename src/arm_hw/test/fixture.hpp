// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <chrono>
#include <future>
#include <map>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"

// The test URDF of both plugins (the same interfaces as robot_bringup/config/arm_hw.urdf.xacro)
// and a watchdog for calls that must not block.
namespace arm_hw::test {

inline constexpr std::array<std::string_view, 6> kJointNames{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

struct UrdfOptions {
  std::string plugin{"arm_hw/DirectArmHw"};
  std::map<std::string, std::string> params{
      {"bus_period_ms", "10"},           {"lease_ms", "48"},         {"max_feedback_age_ms", "30"},
      {"required_capabilities", "none"}, {"can_interface", "test0"}, {"node_ids", "1,2,3,4,5,6"}};
  // Leave out the initial_value of arm/request_id_lo (a URDF the plugin must refuse, C12).
  bool omit_request_initial{false};
};

inline std::string gpio_wide(std::string_view field, bool command, bool initial = true) {
  const char* tag = command ? "command_interface" : "state_interface";
  std::string out;
  for (const char* half : {"_lo", "_hi"}) {
    out += std::string("      <") + tag + " name=\"" + std::string(field) + half +
           "\" data_type=\"uint32\">";
    if (initial) out += "<param name=\"initial_value\">0</param>";
    out += std::string("</") + tag + ">\n";
  }
  return out;
}

inline std::string urdf(const UrdfOptions& o = {}) {
  std::string s = "<?xml version=\"1.0\"?>\n<robot name=\"arm\">\n  <link name=\"base\"/>\n";
  std::string parent = "base";
  for (std::size_t i = 0; i < kJointNames.size(); ++i) {
    const std::string child = "link" + std::to_string(i + 1);
    s += "  <link name=\"" + child + "\"/>\n";
    s += "  <joint name=\"" + std::string(kJointNames.at(i)) +
         "\" type=\"revolute\"><parent link=\"" + parent + "\"/><child link=\"" + child +
         "\"/><axis xyz=\"0 0 1\"/><limit lower=\"-6.28\" upper=\"6.28\" effort=\"10\" "
         "velocity=\"1\"/></joint>\n";
    parent = child;
  }
  s += "  <ros2_control name=\"arm\" type=\"system\">\n    <hardware>\n      <plugin>" + o.plugin +
       "</plugin>\n";
  for (const auto& [k, v] : o.params) s += "      <param name=\"" + k + "\">" + v + "</param>\n";
  s += "    </hardware>\n";
  for (const std::string_view j : kJointNames) {
    s += "    <joint name=\"" + std::string(j) +
         "\">\n      <command_interface name=\"position\"><param name=\"min\">-6.28</param>"
         "<param name=\"max\">6.28</param></command_interface>\n"
         "      <state_interface name=\"position\"/>\n      <state_interface "
         "name=\"velocity\"/>\n    </joint>\n";
  }
  s += "    <gpio name=\"arm\">\n";
  for (const char* f : {"manager", "session", "generation", "hold_until_ns"}) {
    s += gpio_wide(f, true);
  }
  if (o.omit_request_initial) {
    s += "      <command_interface name=\"request_id_lo\" data_type=\"uint32\"/>\n"
         "      <command_interface name=\"request_id_hi\" data_type=\"uint32\">"
         "<param name=\"initial_value\">0</param></command_interface>\n";
  } else {
    s += gpio_wide("request_id", true);
  }
  s += "    </gpio>\n    <gpio name=\"permit\">\n";
  for (const char* f : {"manager", "session", "generation", "expires_ns"}) s += gpio_wide(f, true);
  s += "      <command_interface name=\"present\" data_type=\"uint8\"><param "
       "name=\"initial_value\">0</param></command_interface>\n    </gpio>\n"
       "    <gpio name=\"setpoint\">\n";
  for (const char* f : {"manager", "session", "generation", "seq"}) s += gpio_wide(f, true);
  s += "    </gpio>\n    <gpio name=\"hal\">\n"
       "      <state_interface name=\"verdict\" data_type=\"uint8\"><param "
       "name=\"initial_value\">0</param></state_interface>\n"
       "      <state_interface name=\"arm_state\" data_type=\"uint8\"><param "
       "name=\"initial_value\">0</param></state_interface>\n";
  s += gpio_wide("armed_request_id", false);
  s += gpio_wide("applied_seq", false);
  s += "      <state_interface name=\"faults\" data_type=\"uint32\"><param "
       "name=\"initial_value\">0</param></state_interface>\n";
  for (int i = 1; i <= 6; ++i) {
    s += "      <state_interface name=\"stop_status_" + std::to_string(i) +
         "\" data_type=\"uint8\"><param name=\"initial_value\">0</param></state_interface>\n";
  }
  s += "    </gpio>\n  </ros2_control>\n</robot>\n";
  return s;
}

inline hardware_interface::HardwareInfo hardware_info(const UrdfOptions& o = {}) {
  return hardware_interface::parse_control_resources_from_urdf(urdf(o)).at(0);
}

inline hardware_interface::HardwareComponentParams component_params(const UrdfOptions& o = {}) {
  hardware_interface::HardwareComponentParams p;
  p.hardware_info = hardware_info(o);
  p.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  p.logger = rclcpp::get_logger("arm_hw_test");
  return p;
}

// Runs `f` on its own thread; true when it returned within `limit`.
// A call still blocked when the watchdog fires is a failure the caller reports; the process then
// ends (the thread cannot be joined), so a blocking call can never pass by hanging the suite.
template <class F>
bool returns_within(std::chrono::milliseconds limit, F f) {
  auto task = std::make_shared<std::packaged_task<void()>>(std::move(f));
  std::future<void> done = task->get_future();
  std::thread([task] { (*task)(); }).detach();
  if (done.wait_for(limit) == std::future_status::ready) {
    done.get();
    return true;
  }
  return false;
}

}  // namespace arm_hw::test
