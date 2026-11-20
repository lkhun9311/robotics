// SPDX-License-Identifier: Apache-2.0
#include "arm_hw/arm_system.hpp"

#include <charconv>
#include <cmath>
#include <exception>
#include <limits>
#include <set>
#include <string>
#include <string_view>
#include <utility>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/logging.hpp"
#include "runtime_core/guard.hpp"
#include "runtime_core/permit.hpp"

namespace arm_hw {

namespace hc = hal_contract;
namespace hi = hardware_interface;
namespace rc = runtime_core;

// ---- interfaces.hpp ----

ArmCode arm_code(hc::ArmState state) {
  switch (state) {
    case hc::ArmState::kIdle:
      return ArmCode::kIdle;
    case hc::ArmState::kArming:
      return ArmCode::kArming;
    case hc::ArmState::kArmed:
      return ArmCode::kArmed;
    case hc::ArmState::kFailed:
      return ArmCode::kFailed;
  }
  return ArmCode::kFailed;
}

StopCode stop_code(hc::StopStatus status) {
  switch (status) {
    case hc::StopStatus::kNone:
    case hc::StopStatus::kNotEnabled:
      return StopCode::kNone;
    case hc::StopStatus::kStopping:
      return StopCode::kStopping;
    case hc::StopStatus::kConfirmed:
      return StopCode::kConfirmed;
    case hc::StopStatus::kUnconfirmed:
      return StopCode::kUnconfirmed;
  }
  return StopCode::kUnconfirmed;
}

bool is_loss(hc::FaultSet faults) {
  return faults.has(hc::Fault::kTransportLost) || faults.has(hc::Fault::kNodeLost) ||
         faults.has(hc::Fault::kBootTimeout) || faults.has(hc::Fault::kDriveFault);
}

std::string check_timing(const Timing& t) {
  if (t.bus_period_ms == 10 && t.lease_ms > 0 && t.lease_ms <= 48 && t.max_feedback_age_ms >= 20 &&
      t.max_feedback_age_ms <= 30) {
    return {};
  }
  return "timing outside the supported profile (bus_period_ms = " +
         std::to_string(t.bus_period_ms) + ", lease_ms = " + std::to_string(t.lease_ms) +
         ", max_feedback_age_ms = " + std::to_string(t.max_feedback_age_ms) + "); " +
         std::string(kTimingProfileAllowed);
}

std::optional<std::int64_t> parse_integer(std::string_view text, std::string* error) {
  std::int64_t value = 0;
  const char* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (text.empty() || ec != std::errc() || ptr != end) {
    *error = "not a decimal integer: '" + std::string(text) + "'";
    return std::nullopt;
  }
  return value;
}

std::optional<rc::Requirements> parse_requirements(std::string_view text, std::string* error) {
  rc::Requirements r{};
  if (text == "none") return r;
  if (text == "all") return rc::Requirements{true, true, true, true, true};
  while (true) {
    const std::size_t comma = text.find(',');
    const std::string_view item = text.substr(0, comma);
    bool* flag = nullptr;
    if (item == "device_side_expiry") {
      flag = &r.device_side_expiry;
    } else if (item == "execution_generation") {
      flag = &r.execution_generation;
    } else if (item == "local_stop") {
      flag = &r.local_stop;
    } else if (item == "apply_ack") {
      flag = &r.apply_ack;
    } else if (item == "source_timestamp") {
      flag = &r.source_timestamp;
    }
    if (flag == nullptr || *flag) {
      *error = "required_capabilities: " +
               std::string(flag == nullptr ? "unknown capability '" : "listed twice: '") +
               std::string(item) +
               "' (known: device_side_expiry, execution_generation, local_stop, apply_ack, "
               "source_timestamp; or none, all)";
      return std::nullopt;
    }
    *flag = true;
    if (comma == std::string_view::npos) break;
    text = text.substr(comma + 1);
  }
  return r;
}

std::optional<std::optional<rc::DeviceCapabilities>> parse_profile_limit(std::string_view text,
                                                                         std::string* error) {
  using Limit = std::optional<rc::DeviceCapabilities>;
  if (text == "none") return Limit{};
  if (text == "virtual_canopen") return Limit{rc::virtual_canopen_profile()};
  if (text == "generic_cia402") return Limit{rc::generic_cia402_profile()};
  if (text == "direct") return Limit{rc::direct_backend_profile()};
  if (text == "modbus_io") return Limit{rc::modbus_io_profile()};
  *error = "backend_profile_limit: unknown profile '" + std::string(text) +
           "' (known: none, virtual_canopen, generic_cia402, direct, modbus_io)";
  return std::nullopt;
}

std::optional<std::array<std::uint8_t, hc::kJoints>> parse_node_ids(std::string_view text,
                                                                    std::string* error) {
  std::array<std::uint8_t, hc::kJoints> out{};
  std::set<std::int64_t> seen;
  std::size_t n = 0;
  while (true) {
    const std::size_t comma = text.find(',');
    const std::optional<std::int64_t> v = parse_integer(text.substr(0, comma), error);
    if (!v || *v < 1 || *v > 127 || !seen.insert(*v).second || n >= out.size()) {
      *error = "node_ids: expected six distinct node IDs in 1..127, comma separated";
      return std::nullopt;
    }
    out.at(n++) = static_cast<std::uint8_t>(*v);
    if (comma == std::string_view::npos) break;
    text = text.substr(comma + 1);
  }
  if (n != out.size()) {
    *error = "node_ids: expected six distinct node IDs in 1..127, comma separated";
    return std::nullopt;
  }
  return out;
}

// ---- ArmSystem ----

namespace {

constexpr rc::Duration kJoinTimeout = rc::milliseconds(1000);
// What a lifecycle call waits beyond an op's own deadline: the backend fails the op at its
// deadline (C10), so this only covers the report reaching us.
constexpr rc::Duration kAwaitMargin = rc::milliseconds(500);

constexpr rc::Duration with_margin(rc::Duration op) {
  return rc::Duration{op.ns + kAwaitMargin.ns};
}

std::string full(std::string_view prefix, std::string_view name) {
  return std::string(prefix) + "/" + std::string(name);
}

const hi::ComponentInfo* find_gpio(const hi::HardwareInfo& info, std::string_view name) {
  for (const hi::ComponentInfo& g : info.gpios) {
    if (g.name == name) return &g;
  }
  return nullptr;
}

// The expected interfaces of one GPIO: name -> data type.
using Expected = std::vector<std::pair<std::string, std::string>>;

Expected wide(const auto& fields) {
  Expected e;
  for (const std::string_view f : fields) {
    e.emplace_back(std::string(f) + "_lo", "uint32");
    e.emplace_back(std::string(f) + "_hi", "uint32");
  }
  return e;
}

bool check_gpio(const hi::HardwareInfo& info, std::string_view name, const Expected& expected,
                bool command, std::string* error) {
  const hi::ComponentInfo* g = find_gpio(info, name);
  if (g == nullptr) {
    *error = "URDF: no gpio '" + std::string(name) + "'";
    return false;
  }
  const std::vector<hi::InterfaceInfo>& got = command ? g->command_interfaces : g->state_interfaces;
  const std::vector<hi::InterfaceInfo>& other =
      command ? g->state_interfaces : g->command_interfaces;
  if (!other.empty() || got.size() != expected.size()) {
    *error = "URDF: gpio '" + std::string(name) + "' must have exactly " +
             std::to_string(expected.size()) + (command ? " command" : " state") +
             " interfaces and no others";
    return false;
  }
  for (const auto& [iface, type] : expected) {
    const hi::InterfaceInfo* found = nullptr;
    for (const hi::InterfaceInfo& i : got) {
      if (i.name == iface) found = &i;
    }
    if (found == nullptr || found->data_type != type) {
      *error = "URDF: gpio '" + std::string(name) + "' needs interface '";
      *error += iface;
      *error += "' of data_type ";
      *error += type;
      return false;
    }
    // C12: an integer command handle without an initial value starts at its maximum.
    if (command && found->initial_value != "0") {
      *error = "URDF: " + full(name, iface) + " needs an explicit initial_value of 0 (C12)";
      return false;
    }
  }
  return true;
}

std::optional<double> finite_number(const std::string& text) {
  try {
    std::size_t used = 0;
    const double v = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(v)) return std::nullopt;
    return v;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace

// The derived destructors have already dropped the cycle (their forget_backend ran there).
ArmSystem::~ArmSystem() { hal_.reset(); }

rc::MonoTime ArmSystem::now() { return rc::SystemMonotonicClock{}.now(); }

ArmSystem::CallbackReturn ArmSystem::fail(const std::string& reason, CallbackReturn result) {
  last_error_ = reason;
  RCLCPP_ERROR(get_logger(), "%s: %s", info_.name.c_str(), reason.c_str());
  return result;
}

void ArmSystem::drop_cycle() {
  forget_backend();
  hal_.reset();
}

ArmSystem::CallbackReturn ArmSystem::on_init(const hi::HardwareComponentInterfaceParams& params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return fail("the base SystemInterface refused the hardware info", CallbackReturn::ERROR);
  }
  std::string error;
  if (!parse_params(&error)) return fail(error, CallbackReturn::ERROR);
  last_error_.clear();
  return CallbackReturn::SUCCESS;
}

bool ArmSystem::parse_params(std::string* error) {
  if (info_.joints.size() != hc::kJoints) {
    *error = "URDF: expected six joints, got " + std::to_string(info_.joints.size());
    return false;
  }
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    const hi::ComponentInfo& j = info_.joints.at(i);
    if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
        j.command_interfaces[0].data_type != "double") {
      *error = "URDF: joint '" + j.name + "' needs exactly one command interface, position";
      return false;
    }
    if (j.state_interfaces.size() != 2 || j.state_interfaces[0].name != "position" ||
        j.state_interfaces[1].name != "velocity" || j.state_interfaces[0].data_type != "double" ||
        j.state_interfaces[1].data_type != "double") {
      *error = "URDF: joint '" + j.name + "' needs exactly the state interfaces position, velocity";
      return false;
    }
    const std::optional<double> lo = finite_number(j.command_interfaces[0].min);
    const std::optional<double> hi_ = finite_number(j.command_interfaces[0].max);
    if (!lo || !hi_ || *lo >= *hi_) {
      *error = "URDF: joint '" + j.name + "' position command needs finite min < max";
      return false;
    }
    params_.limits.pos_min.at(i) = *lo;
    params_.limits.pos_max.at(i) = *hi_;
    params_.limits.vel_max.at(i) = 1.0;   // v_max (E3a)
    params_.limits.acc_max.at(i) = 10.0;  // a_stop (E3a)
  }
  if (info_.gpios.size() != 4) {
    *error = "URDF: expected exactly the gpios arm, permit, setpoint, hal";
    return false;
  }
  Expected permit = wide(kPermitFields);
  permit.emplace_back(std::string(kPermitPresent), "uint8");
  Expected hal = {{std::string(kVerdict), "uint8"}, {std::string(kArmState), "uint8"}};
  for (const std::string_view f : {kArmedRequestId, kAppliedSeq}) {
    hal.emplace_back(std::string(f) + "_lo", "uint32");
    hal.emplace_back(std::string(f) + "_hi", "uint32");
  }
  hal.emplace_back(std::string(kFaults), "uint32");
  for (std::size_t i = 1; i <= hc::kJoints; ++i) {
    hal.emplace_back(std::string(kStopStatus) + std::to_string(i), "uint8");
  }
  if (!check_gpio(info_, kArmGpio, wide(kArmFields), true, error) ||
      !check_gpio(info_, kPermitGpio, permit, true, error) ||
      !check_gpio(info_, kSetpointGpio, wide(kSetpointFields), true, error) ||
      !check_gpio(info_, kHalGpio, hal, false, error)) {
    return false;
  }

  const auto& p = info_.hardware_parameters;
  const auto text = [&](const char* key) -> std::optional<std::string> {
    const auto it = p.find(key);
    if (it == p.end()) return std::nullopt;
    return it->second;
  };
  const auto required = [&](const char* key) -> std::optional<std::string> {
    std::optional<std::string> v = text(key);
    if (!v) *error = std::string("missing hardware parameter '") + key + "'";
    return v;
  };
  const auto integer = [&](const char* key, std::int64_t& out) {
    const std::optional<std::string> v = required(key);
    if (!v) return false;
    const std::optional<std::int64_t> n = parse_integer(*v, error);
    if (!n) {
      *error = std::string(key) + ": " + *error;
      return false;
    }
    out = *n;
    return true;
  };
  if (!integer("bus_period_ms", params_.timing.bus_period_ms) ||
      !integer("lease_ms", params_.timing.lease_ms) ||
      !integer("max_feedback_age_ms", params_.timing.max_feedback_age_ms)) {
    return false;
  }
  const std::optional<std::string> req = required("required_capabilities");
  if (!req) return false;
  const std::optional<rc::Requirements> r = parse_requirements(*req, error);
  if (!r) return false;
  params_.required = *r;
  const auto limit = parse_profile_limit(text("backend_profile_limit").value_or("none"), error);
  if (!limit) return false;
  params_.profile_limit = *limit;
  if (needs_bus()) {
    const std::optional<std::string> iface = required("can_interface");
    if (!iface) return false;
    if (iface->empty()) {
      *error = "can_interface is empty";
      return false;
    }
    params_.can_interface = *iface;
    const std::optional<std::string> ids = required("node_ids");
    if (!ids) return false;
    const auto parsed = parse_node_ids(*ids, error);
    if (!parsed) return false;
    params_.node_ids = *parsed;
  }
  return true;
}

bool ArmSystem::bind_handles(std::string* error) {
  if (bound_) return true;
  try {
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      const std::string& j = info_.joints.at(i).name;
      joint_cmd_.at(i) = get_command_interface_handle(full(j, "position"));
      joint_pos_.at(i) = get_state_interface_handle(full(j, "position"));
      joint_vel_.at(i) = get_state_interface_handle(full(j, "velocity"));
    }
    const auto wide_cmd = [&](std::string_view gpio, std::string_view field) {
      return Wide{get_command_interface_handle(full(gpio, std::string(field) + "_lo")),
                  get_command_interface_handle(full(gpio, std::string(field) + "_hi"))};
    };
    for (std::size_t i = 0; i < kArmFields.size(); ++i)
      arm_.at(i) = wide_cmd(kArmGpio, kArmFields.at(i));
    for (std::size_t i = 0; i < kPermitFields.size(); ++i) {
      permit_.at(i) = wide_cmd(kPermitGpio, kPermitFields.at(i));
    }
    permit_present_ = get_command_interface_handle(full(kPermitGpio, kPermitPresent));
    for (std::size_t i = 0; i < kSetpointFields.size(); ++i) {
      setpoint_.at(i) = wide_cmd(kSetpointGpio, kSetpointFields.at(i));
    }
    const auto state = [&](std::string_view name) {
      return get_state_interface_handle(full(kHalGpio, name));
    };
    verdict_ = state(kVerdict);
    arm_state_ = state(kArmState);
    armed_request_id_ = WideState{state(std::string(kArmedRequestId) + "_lo"),
                                  state(std::string(kArmedRequestId) + "_hi")};
    applied_seq_ =
        WideState{state(std::string(kAppliedSeq) + "_lo"), state(std::string(kAppliedSeq) + "_hi")};
    faults_ = state(kFaults);
    for (std::size_t i = 0; i < hc::kJoints; ++i) {
      stop_.at(i) = state(std::string(kStopStatus) + std::to_string(i + 1));
    }
  } catch (const std::exception& e) {
    *error = std::string("interfaces not exported: ") + e.what();
    return false;
  }
  all_gpio_commands_.clear();
  for (const Wide& w : arm_) all_gpio_commands_.insert(all_gpio_commands_.end(), {w.lo, w.hi});
  for (const Wide& w : permit_) all_gpio_commands_.insert(all_gpio_commands_.end(), {w.lo, w.hi});
  for (const Wide& w : setpoint_) all_gpio_commands_.insert(all_gpio_commands_.end(), {w.lo, w.hi});
  for (const Command& c : all_gpio_commands_) {
    if (!c) {
      *error = "a gpio command handle is null";
      return false;
    }
  }
  if (!permit_present_ || !verdict_ || !arm_state_ || !faults_) {
    *error = "a gpio handle is null";
    return false;
  }
  bound_ = true;
  return true;
}

void ArmSystem::reset_commands() {
  for (const Command& c : all_gpio_commands_) {
    std::ignore = c->set_value(std::uint32_t{0}, true);
  }
  std::ignore = permit_present_->set_value(std::uint8_t{0}, true);
  last_request_id_ = 0;
}

ArmSystem::CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State& /*previous*/) {
  if (active_) {
    return fail("deactivate before configure: a configure cycle's Hal is never reconfigured");
  }
  if (const std::string t = check_timing(params_.timing); !t.empty()) return fail(t);
  if (hal_) {
    if (backend_running()) {
      return fail(
          "the previous configure cycle's worker is still running (shutting down); configure "
          "again once it has exited");
    }
    drop_cycle();
  }
  std::string error;
  if (!bind_handles(&error)) return fail(error);
  set_verdict(hc::HalVerdict::kHardwareNotActive);
  put8(arm_state_, static_cast<std::uint8_t>(ArmCode::kIdle));
  put(armed_request_id_, 0);
  put(applied_seq_, 0);
  put32(faults_, 0);
  for (const State& s : stop_) put8(s, static_cast<std::uint8_t>(StopCode::kNone));

  std::unique_ptr<hc::Backend> backend = make_backend(params_, &error);
  if (!backend) return fail("cannot build the backend: " + error);
  try {
    hal_ = std::make_unique<hc::Hal>(
        std::move(backend), params_.required,
        rc::GuardConfig{rc::milliseconds(params_.timing.bus_period_ms),
                        rc::milliseconds(params_.timing.max_feedback_age_ms)},
        params_.limits, rc::milliseconds(params_.timing.lease_ms));
  } catch (const std::exception& e) {
    forget_backend();
    return fail(std::string("cannot build the Hal: ") + e.what());
  }
  error_reported_ = false;
  const hc::OpHandle op = hal_->configure(now());
  const std::optional<hc::OpResult> r = await(op, with_margin(hc::kConfigureTimeout));
  if (!r || !r->ok) {
    const std::string why = r ? r->error : std::string("configure timed out");
    // The cycle failed: its worker goes now if it can; otherwise the next configure waits for it.
    if (stop_backend(kJoinTimeout)) drop_cycle();
    return fail("configure failed: " + why);
  }
  last_error_.clear();
  RCLCPP_INFO(get_logger(), "%s: configured", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

ArmSystem::CallbackReturn ArmSystem::on_activate(const rclcpp_lifecycle::State& /*previous*/) {
  if (!hal_) return fail("activate without a configured cycle");
  reset_commands();
  const hc::OpHandle op = hal_->activate(now());
  const std::optional<hc::OpResult> r = await(op, with_margin(hc::kActivateTimeout));
  if (!r || !r->ok) {
    std::string why = r ? r->error : std::string("activate timed out");
    if (r && !r->missing.empty()) {
      why += " (missing:";
      for (const rc::Missing& m : r->missing) {
        why += std::string(" ") + rc::to_string(m.capability) + "=" + rc::to_string(m.support);
      }
      why += ")";
    }
    return fail("activation refused: " + why);
  }
  active_ = true;
  set_verdict(hc::HalVerdict::kNoPermit);
  last_error_.clear();
  RCLCPP_INFO(get_logger(), "%s: active", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

ArmSystem::CallbackReturn ArmSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous*/) {
  active_ = false;
  if (bound_) reset_commands();
  if (!hal_) return CallbackReturn::SUCCESS;
  set_verdict(hc::HalVerdict::kHardwareNotActive);
  const hc::OpHandle op = hal_->deactivate(now());
  const std::optional<hc::OpResult> r = await(op, with_margin(hc::kDeactivateTimeout));
  if (!r || !r->ok) {
    // The stop of an armed group was not confirmed: latch the inhibit and let on_error follow.
    hal_->on_error_nonblocking();
    return fail("deactivate failed: " + (r ? r->error : std::string("timed out")),
                CallbackReturn::ERROR);
  }
  last_error_.clear();
  return CallbackReturn::SUCCESS;
}

ArmSystem::CallbackReturn ArmSystem::on_cleanup(const rclcpp_lifecycle::State& /*previous*/) {
  if (!hal_) return CallbackReturn::SUCCESS;
  if (!stop_backend(kJoinTimeout)) {
    return fail("the worker did not exit within 1 s; cleanup again");
  }
  drop_cycle();
  last_error_.clear();
  return CallbackReturn::SUCCESS;
}

ArmSystem::CallbackReturn ArmSystem::on_shutdown(const rclcpp_lifecycle::State& previous) {
  active_ = false;
  return on_cleanup(previous);
}

ArmSystem::CallbackReturn ArmSystem::on_error(const rclcpp_lifecycle::State& /*previous*/) {
  active_ = false;
  if (hal_) hal_->on_error_nonblocking();
  return CallbackReturn::SUCCESS;
}

std::optional<hc::OpResult> ArmSystem::await(hc::OpHandle op, rc::Duration timeout) {
  return hal_->wait(op, timeout);
}

// ---- control path ----

bool ArmSystem::get(const Wide& w, std::uint64_t& out) {
  Halves h;
  if (!w.lo->get_value(h.lo, false) || !w.hi->get_value(h.hi, false)) return false;
  out = join(h);
  return true;
}

bool ArmSystem::get8(const Command& c, std::uint8_t& out) { return c->get_value(out, false); }

void ArmSystem::put(const WideState& w, std::uint64_t value) {
  const Halves h = split(value);
  std::ignore = w.lo->set_value(h.lo, false);
  std::ignore = w.hi->set_value(h.hi, false);
}

void ArmSystem::put8(const State& s, std::uint8_t value) {
  std::ignore = s->set_value(value, false);
}

void ArmSystem::put32(const State& s, std::uint32_t value) {
  std::ignore = s->set_value(value, false);
}

void ArmSystem::set_verdict(hc::HalVerdict v) { put8(verdict_, static_cast<std::uint8_t>(v)); }

void ArmSystem::publish(const hc::Feedback& fb) {
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    std::ignore = joint_pos_.at(i)->set_value(fb.position.at(i), false);
    std::ignore = joint_vel_.at(i)->set_value(fb.velocity.at(i), false);
    put8(stop_.at(i), static_cast<std::uint8_t>(stop_code(fb.stop.at(i))));
  }
  put8(arm_state_, static_cast<std::uint8_t>(arm_code(hal_->arm_state())));
  put(armed_request_id_, hal_->armed_request_id());
  put(applied_seq_, fb.group_applied_source_seq.value_or(0));
  put32(faults_, fb.faults.bits());
}

hi::return_type ArmSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!hal_ || !bound_) return hi::return_type::OK;
  const rc::MonoTime t = now();
  advance(t);
  const hc::Feedback fb = hal_->read(t);
  publish(fb);
  if (is_loss(fb.faults) && !error_reported_) {
    error_reported_ = true;
    return hi::return_type::ERROR;
  }
  return hi::return_type::OK;
}

namespace {

rc::MonoTime to_time(std::uint64_t ns) {
  // A value past the int64 range is no instant of CLOCK_MONOTONIC: it reads as 0 (long past).
  return rc::MonoTime{ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
                          ? 0
                          : static_cast<std::int64_t>(ns)};
}

}  // namespace

hi::return_type ArmSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!hal_ || !bound_) return hi::return_type::OK;
  const rc::MonoTime t = now();
  // A logic-time backend catches up to t first.
  // Otherwise a step dated before t, replayed by the next read(), would take this write's command
  // before it was approved.
  advance(t);

  // arm: a new non-zero request id posts one arm op, never waited for.
  std::array<std::uint64_t, kArmFields.size()> a{};
  bool arm_read = true;
  for (std::size_t i = 0; i < a.size(); ++i) arm_read = get(arm_.at(i), a.at(i)) && arm_read;
  const std::uint64_t request_id = a.at(4);
  if (arm_read && request_id != 0 && request_id != last_request_id_) {
    last_request_id_ = request_id;
    const hc::AuthorityId id{rc::ManagerInstance{a.at(0)}, rc::SessionId{a.at(1)},
                             rc::Generation{a.at(2)}};
    std::ignore = hal_->arm(hc::ArmRequest{id, to_time(a.at(3)), request_id}, t);
  }

  // permit: present only when permit/present == 1 and every half was read.
  std::array<std::uint64_t, kPermitFields.size()> p{};
  std::uint8_t present = 0;
  bool permit_read = get8(permit_present_, present);
  for (std::size_t i = 0; i < p.size(); ++i)
    permit_read = get(permit_.at(i), p.at(i)) && permit_read;
  const bool has_permit = permit_read && present == kPresent;
  const rc::MotionPermit permit{rc::ManagerInstance{p.at(0)}, rc::SessionId{p.at(1)},
                                rc::Generation{p.at(2)}, to_time(p.at(3))};

  std::array<std::uint64_t, kSetpointFields.size()> s{};
  for (std::size_t i = 0; i < s.size(); ++i) std::ignore = get(setpoint_.at(i), s.at(i));
  hc::HalSetpoint sp{};
  sp.id = hc::AuthorityId{rc::ManagerInstance{s.at(0)}, rc::SessionId{s.at(1)},
                          rc::Generation{s.at(2)}};
  sp.snapshot.generation = sp.id.generation;
  sp.snapshot.source_seq = s.at(3);
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    double q = std::numeric_limits<double>::quiet_NaN();
    if (!joint_cmd_.at(i)->get_value(q, false)) q = std::numeric_limits<double>::quiet_NaN();
    sp.snapshot.position.at(i) = q;
  }

  hc::HalVerdict v = hal_->write(has_permit ? &permit : nullptr, sp, t);
  if (!has_permit && v != hc::HalVerdict::kInhibited) v = hc::HalVerdict::kNoPermit;
  set_verdict(v);
  return hi::return_type::OK;
}

}  // namespace arm_hw
