// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "runtime_core/deadline.hpp"
#include "runtime_core/ids.hpp"
#include "runtime_core/permit.hpp"
#include "runtime_core/time.hpp"

namespace runtime_core {

enum class GuardVerdict {
  kSend,
  kHardwareNotActive,
  kNoPermit,
  kPermitExpired,
  kManagerMismatch,
  kSessionMismatch,
  kGenerationMismatch,
  kFeedbackStale,
  kNonFinite,
  kPositionLimit,
  kVelocityLimit,
  kAccelerationLimit,
};

const char* to_string(GuardVerdict verdict);

template <std::size_t N>
struct JointLimits {
  std::array<double, N> pos_min{};
  std::array<double, N> pos_max{};
  std::array<double, N> vel_max{};
  std::array<double, N> acc_max{};
};

template <std::size_t N>
struct SetpointSnapshot {
  Generation generation{};
  std::uint64_t source_seq{0};
  std::array<double, N> position{};
};

struct GuardConfig {
  Duration period{};
  Duration max_feedback_age{};
};

using ExpiryPredicate = bool (*)(MonoTime now, MonoTime valid_until);

template <std::size_t N>
struct GuardInput {
  bool hardware_active{false};
  const MotionPermit* permit{nullptr};
  ManagerInstance current_manager{};
  SessionId current_session{};
  SetpointSnapshot<N> setpoint{};
  MonoTime feedback_sampled_at{};
  std::array<double, N> feedback_position{};
};

// Decides once per control period whether a setpoint may go out as a normal trajectory
// command (spec §6.2). Every check must pass; blocking is the default.
// After any blocked cycle, the next send is measured against feedback at rest.
template <std::size_t N>
class HalGuard {
 public:
  // Throws std::invalid_argument on a configuration that would make a check meaningless
  // (zero period gives NaN velocities, and NaN passes every `>` comparison).
  HalGuard(GuardConfig config, JointLimits<N> limits, ExpiryPredicate expired = &is_expired)
      : config_(config), limits_(limits), expired_(expired) {
    if (config_.period.ns <= 0) throw std::invalid_argument("HalGuard: period must be positive");
    if (config_.max_feedback_age.ns < 0) {
      throw std::invalid_argument("HalGuard: max_feedback_age must not be negative");
    }
    if (expired_ == nullptr) throw std::invalid_argument("HalGuard: expiry predicate is null");
    for (std::size_t i = 0; i < N; ++i) {
      const bool finite = std::isfinite(limits_.pos_min[i]) && std::isfinite(limits_.pos_max[i]) &&
                          std::isfinite(limits_.vel_max[i]) && std::isfinite(limits_.acc_max[i]);
      if (!finite || limits_.pos_min[i] > limits_.pos_max[i] || limits_.vel_max[i] <= 0.0 ||
          limits_.acc_max[i] <= 0.0) {
        throw std::invalid_argument("HalGuard: limits must be finite and consistent");
      }
    }
  }

  GuardVerdict check(const GuardInput<N>& in, MonoTime now) {
    const GuardVerdict verdict = evaluate(in, now);
    if (verdict == GuardVerdict::kSend) {
      last_position_ = in.setpoint.position;
      last_velocity_ = candidate_velocity_;
      has_reference_ = true;
    } else {
      has_reference_ = false;
    }
    return verdict;
  }

  // Called at ARMING so the first send is measured against the actual position at rest.
  void reset_reference() { has_reference_ = false; }

 private:
  static constexpr double kEps = 1e-9;

  GuardVerdict evaluate(const GuardInput<N>& in, MonoTime now) {
    if (!in.hardware_active) return GuardVerdict::kHardwareNotActive;
    if (in.permit == nullptr) return GuardVerdict::kNoPermit;
    if (expired_(now, in.permit->expires_at)) return GuardVerdict::kPermitExpired;
    if (in.permit->manager != in.current_manager) return GuardVerdict::kManagerMismatch;
    if (in.permit->session != in.current_session) return GuardVerdict::kSessionMismatch;
    if (in.permit->generation != in.setpoint.generation) {
      return GuardVerdict::kGenerationMismatch;
    }
    const Duration age = now - in.feedback_sampled_at;
    if (age.ns < 0 || age > config_.max_feedback_age) return GuardVerdict::kFeedbackStale;
    for (std::size_t i = 0; i < N; ++i) {
      if (!std::isfinite(in.setpoint.position[i]) || !std::isfinite(in.feedback_position[i])) {
        return GuardVerdict::kNonFinite;
      }
    }
    for (std::size_t i = 0; i < N; ++i) {
      const double q = in.setpoint.position[i];
      if (q < limits_.pos_min[i] || q > limits_.pos_max[i]) return GuardVerdict::kPositionLimit;
    }
    const double dt = static_cast<double>(config_.period.ns) * 1e-9;
    const std::array<double, N>& ref_q = has_reference_ ? last_position_ : in.feedback_position;
    std::array<double, N> ref_v{};
    if (has_reference_) ref_v = last_velocity_;
    for (std::size_t i = 0; i < N; ++i) {
      const double v = (in.setpoint.position[i] - ref_q[i]) / dt;
      if (!std::isfinite(v)) return GuardVerdict::kNonFinite;
      if (std::abs(v) > limits_.vel_max[i] + kEps) return GuardVerdict::kVelocityLimit;
      const double a = (v - ref_v[i]) / dt;
      if (!std::isfinite(a)) return GuardVerdict::kNonFinite;
      if (std::abs(a) > limits_.acc_max[i] + kEps) return GuardVerdict::kAccelerationLimit;
      candidate_velocity_[i] = v;
    }
    return GuardVerdict::kSend;
  }

  GuardConfig config_;
  JointLimits<N> limits_;
  ExpiryPredicate expired_;
  bool has_reference_{false};
  std::array<double, N> last_position_{};
  std::array<double, N> last_velocity_{};
  std::array<double, N> candidate_velocity_{};
};

}  // namespace runtime_core
