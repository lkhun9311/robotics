// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "runtime_core/time.hpp"

// The virtual drive's one-joint model (contract D10, Global Constraints). Integer nanoradians
// and nanoradians per second, so the quick-stop ramp is achieved exactly: with the project
// values every step is an exact integer and no rounding accumulates over a stop.
namespace drive_core {

// Project-declared values (E1; not industry values).
inline constexpr std::int64_t kVMaxNradS = 1'000'000'000;        // v_max 1.0 rad/s
inline constexpr std::int64_t kAStopNradS2 = 10'000'000'000;     // a_stop 10.0 rad/s^2
inline constexpr runtime_core::Duration kTickPeriod{2'000'000};  // W 2 ms

struct JointState {
  std::int64_t position_nrad{};
  std::int64_t velocity_nrad_s{};

  friend bool operator==(const JointState&, const JointState&) = default;
};

// One call is one W step. Every step sets the velocity first and then integrates the position
// with the new velocity (semi-implicit), so braking travel stays <= v^2 / (2 a_stop).
class JointModel {
 public:
  explicit JointModel(runtime_core::Duration step = kTickPeriod, std::int64_t v_max = kVMaxNradS,
                      std::int64_t a_stop = kAStopNradS2);

  // Tracks a position target at no more than v_max: the velocity is the one that reaches the
  // target in this step, clamped to +/-v_max.
  void track(std::int64_t target_nrad);
  // Quick-stop ramp: v <- v - sign(v) * min(|v|, a_stop * W), then x <- x + v * W.
  void brake();
  // No actuator force (loss of drive authority): the velocity is kept, the position integrates.
  void coast();

  // Test injection: a stuck joint ignores the brake (its velocity stays as it is).
  void set_stuck(bool stuck) { stuck_ = stuck; }

  const JointState& state() const { return state_; }

 private:
  void integrate();

  std::int64_t step_ns_;
  std::int64_t v_max_;
  std::int64_t dv_per_step_;  // a_stop * W
  std::int64_t max_travel_;   // v_max * W, the farthest one step goes
  bool stuck_{false};
  JointState state_;
};

}  // namespace drive_core
