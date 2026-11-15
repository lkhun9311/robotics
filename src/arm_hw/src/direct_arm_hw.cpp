// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "arm_hw/arm_system.hpp"
#include "canopen_backend/canopen_backend.hpp"
#include "direct_backend/direct_backend.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hw {

namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

// The DirectBackend with a backend_profile_limit (C2): the reported capabilities are the weaker,
// per capability, of the backend's and the limit; everything else is forwarded.
class LimitedBackend final : public hc::Backend {
 public:
  LimitedBackend(std::unique_ptr<hc::Backend> inner, rc::DeviceCapabilities limit)
      : inner_(std::move(inner)), limit_(limit) {}

  void configure(const hc::OpTicket& t, rc::MonoTime now) override { inner_->configure(t, now); }
  rc::DeviceCapabilities capabilities() const override {
    return canopen_backend::weaker(inner_->capabilities(), limit_);
  }
  void activate(const hc::OpTicket& t, rc::MonoTime now) override { inner_->activate(t, now); }
  void arm(const hc::OpTicket& t, const hc::ArmRequest& r, rc::MonoTime now) override {
    inner_->arm(t, r, now);
  }
  void submit(const hc::ApprovedCommand& c, rc::MonoTime now) override { inner_->submit(c, now); }
  hc::Feedback poll(rc::MonoTime now) override { return inner_->poll(now); }
  void deactivate(const hc::OpTicket& t, rc::MonoTime now) override { inner_->deactivate(t, now); }
  void on_error_nonblocking() override { inner_->on_error_nonblocking(); }
  hc::OpState op_state(std::uint64_t id) const override { return inner_->op_state(id); }
  std::optional<hc::OpResult> wait(std::uint64_t id, rc::Duration timeout) override {
    return inner_->wait(id, timeout);
  }
  bool inhibited() const override { return inner_->inhibited(); }
  bool can_apply(rc::MonoTime valid_until, rc::MonoTime now) const override {
    return inner_->can_apply(valid_until, now);
  }

 private:
  std::unique_ptr<hc::Backend> inner_;
  rc::DeviceCapabilities limit_;
};

}  // namespace

DirectArmHw::~DirectArmHw() { drop_cycle(); }

std::unique_ptr<hc::Backend> DirectArmHw::make_backend(const Params& params, std::string* error) {
  direct_backend::DirectConfig config;
  config.max_feedback_age = rc::milliseconds(params.timing.max_feedback_age_ms);
  std::unique_ptr<direct_backend::DirectBackend> direct;
  try {
    direct = std::make_unique<direct_backend::DirectBackend>(config);
  } catch (const std::exception& e) {
    *error = e.what();
    return nullptr;
  }
  direct_ = direct.get();
  next_step_ = now();
  if (!params.profile_limit) return direct;
  return std::make_unique<LimitedBackend>(std::move(direct), *params.profile_limit);
}

void DirectArmHw::advance(rc::MonoTime t) {
  if (direct_ == nullptr || t < next_step_) return;
  const std::int64_t due = (t - next_step_).ns / direct_backend::kStep.ns + 1;
  if (due > kMaxCatchUp) {
    // Not replayed: jump to the last grid point at or before t.
    next_step_ = next_step_ + rc::Duration{(due - 1) * direct_backend::kStep.ns};
  }
  while (next_step_ <= t) {
    direct_->step(next_step_);
    next_step_ = next_step_ + direct_backend::kStep;
  }
}

std::optional<hc::OpResult> DirectArmHw::await(hc::OpHandle op, rc::Duration timeout) {
  const rc::MonoTime deadline = now() + timeout;
  while (true) {
    advance(now());
    if (hal_->op_state(op) != hc::OpState::kPending || now() >= deadline) {
      return hal_->wait(op, rc::Duration{});
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(direct_backend::kStep.ns));
  }
}

}  // namespace arm_hw

PLUGINLIB_EXPORT_CLASS(arm_hw::DirectArmHw, hardware_interface::SystemInterface)
