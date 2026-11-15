// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/capability.hpp"

// The ros2_control interface of the arm (C12) and the plugin parameters (C2, C20), without ROS
// types: names, the 32-bit halves of 64-bit values, the state codes and the parameter parsers.
//
// Per joint: state `position`, `velocity`; command `position` (double, with `min`/`max`, which are
// the guard's position limits). GPIOs, every integer interface uint32 unless noted:
//   arm      (command)  manager_lo/hi session_lo/hi generation_lo/hi hold_until_ns_lo/hi
//                       request_id_lo/hi
//   permit   (command)  manager_lo/hi session_lo/hi generation_lo/hi expires_ns_lo/hi
//                       present (uint8)
//   setpoint (command)  manager_lo/hi session_lo/hi generation_lo/hi seq_lo/hi
//   hal      (state)    verdict (uint8, HalVerdict) arm_state (uint8, ArmCode)
//                       armed_request_id_lo/hi applied_seq_lo/hi faults (FaultSet bits)
//                       stop_status_1..6 (uint8, StopCode, joint order)
// HANDLE_DATATYPE of hardware_interface 4.48.1 has no 64-bit integer, so 64-bit values travel as
// exact 32-bit halves (a double would lose 2^53 + 1). Every GPIO command interface must have an
// explicit initial_value of 0 in the URDF: ros2_control would otherwise start an integer handle
// at its maximum, which would read as a request id or a permit.
namespace arm_hw {

struct Halves {
  std::uint32_t lo{0};
  std::uint32_t hi{0};
  bool operator==(const Halves&) const = default;
};

constexpr Halves split(std::uint64_t value) {
  return Halves{static_cast<std::uint32_t>(value & 0xFFFF'FFFFULL),
                static_cast<std::uint32_t>(value >> 32U)};
}

constexpr std::uint64_t join(Halves h) {
  return (static_cast<std::uint64_t>(h.hi) << 32U) | static_cast<std::uint64_t>(h.lo);
}

inline constexpr std::string_view kArmGpio = "arm";
inline constexpr std::string_view kPermitGpio = "permit";
inline constexpr std::string_view kSetpointGpio = "setpoint";
inline constexpr std::string_view kHalGpio = "hal";

// The 64-bit fields of each command GPIO, in order; each is `<field>_lo` and `<field>_hi`.
inline constexpr std::array<std::string_view, 5> kArmFields{"manager", "session", "generation",
                                                            "hold_until_ns", "request_id"};
inline constexpr std::array<std::string_view, 4> kPermitFields{"manager", "session", "generation",
                                                               "expires_ns"};
inline constexpr std::string_view kPermitPresent = "present";
inline constexpr std::array<std::string_view, 4> kSetpointFields{"manager", "session", "generation",
                                                                 "seq"};
// The hal state interfaces besides stop_status_<n>.
inline constexpr std::string_view kVerdict = "verdict";
inline constexpr std::string_view kArmState = "arm_state";
inline constexpr std::string_view kArmedRequestId = "armed_request_id";
inline constexpr std::string_view kAppliedSeq = "applied_seq";
inline constexpr std::string_view kFaults = "faults";
inline constexpr std::string_view kStopStatus = "stop_status_";

// hal/arm_state.
enum class ArmCode : std::uint8_t { kIdle = 0, kArming = 1, kArmed = 2, kFailed = 3 };
ArmCode arm_code(hal_contract::ArmState state);

// hal/stop_status_<n>. kNotEnabled (nothing to stop) reads as kNone.
enum class StopCode : std::uint8_t { kNone = 0, kStopping = 1, kConfirmed = 2, kUnconfirmed = 3 };
StopCode stop_code(hal_contract::StopStatus status);

// permit/present must equal exactly this; any other value is no permit.
inline constexpr std::uint8_t kPresent = 1;

// Faults that make read() return ERROR (once per configure cycle): the loss of the transport or of
// a drive (spec §8.2). Policy events (expiry, refusals, policy stops, the inhibit) never do.
bool is_loss(hal_contract::FaultSet faults);

// ---- parameters ----

// The C20 timing profile: bus_period_ms = 10, lease_ms in (0, 48], max_feedback_age_ms in
// [20, 30]. Outside it on_configure fails with kTimingProfileAllowed in its message.
inline constexpr std::string_view kTimingProfileAllowed =
    "allowed: bus_period_ms = 10, lease_ms in (0, 48], max_feedback_age_ms in [20, 30] (C20)";

struct Timing {
  std::int64_t bus_period_ms{10};
  std::int64_t lease_ms{48};
  std::int64_t max_feedback_age_ms{20};
};

// Empty when `t` is inside the C20 profile, else the reason with the allowed values.
std::string check_timing(const Timing& t);

// Each parser returns std::nullopt and sets `error` on a value it does not accept.
// A decimal integer.
std::optional<std::int64_t> parse_integer(std::string_view text, std::string* error);
// "none", "all", or a comma list of device_side_expiry, execution_generation, local_stop,
// apply_ack, source_timestamp.
std::optional<runtime_core::Requirements> parse_requirements(std::string_view text,
                                                             std::string* error);
// backend_profile_limit (C2): "none" (no limit: outer nullopt is an error, inner nullopt no
// limit), or the name of a profile: virtual_canopen, generic_cia402, direct, modbus_io.
std::optional<std::optional<runtime_core::DeviceCapabilities>> parse_profile_limit(
    std::string_view text, std::string* error);
// Six distinct node IDs in 1..127, comma separated, in joint order.
std::optional<std::array<std::uint8_t, hal_contract::kJoints>> parse_node_ids(std::string_view text,
                                                                              std::string* error);

}  // namespace arm_hw
