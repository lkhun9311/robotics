// SPDX-License-Identifier: Apache-2.0
#include "drive_core/joint.hpp"

namespace drive_core {

namespace {

constexpr std::int64_t kNsPerS = 1'000'000'000;

}  // namespace

JointModel::JointModel(runtime_core::Duration step, std::int64_t v_max, std::int64_t a_stop)
    : step_ns_(step.ns),
      v_max_(v_max),
      dv_per_step_(a_stop * step.ns / kNsPerS),
      max_travel_(v_max * step.ns / kNsPerS) {}

void JointModel::integrate() {
  state_.position_nrad += state_.velocity_nrad_s * step_ns_ / kNsPerS;
}

void JointModel::track(std::int64_t target_nrad) {
  const std::int64_t error = target_nrad - state_.position_nrad;
  if (error > max_travel_) {
    state_.velocity_nrad_s = v_max_;
  } else if (error < -max_travel_) {
    state_.velocity_nrad_s = -v_max_;
  } else {
    // Within one step: land on the target exactly (|error| <= v_max * W, so no overflow).
    state_.velocity_nrad_s = error * kNsPerS / step_ns_;
    state_.position_nrad = target_nrad;
    return;
  }
  integrate();
}

void JointModel::brake() {
  if (!stuck_) {
    const std::int64_t v = state_.velocity_nrad_s;
    const std::int64_t magnitude = v < 0 ? -v : v;
    const std::int64_t dv = magnitude < dv_per_step_ ? magnitude : dv_per_step_;
    state_.velocity_nrad_s = v < 0 ? v + dv : v - dv;
  }
  integrate();
}

void JointModel::coast() { integrate(); }

}  // namespace drive_core
