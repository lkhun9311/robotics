// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "arm_hw/interfaces.hpp"
#include "canopen_backend/transport.hpp"
#include "hal_contract/backend.hpp"
#include "hal_contract/hal.hpp"
#include "hal_contract/records.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "runtime_core/capability.hpp"
#include "runtime_core/time.hpp"

namespace canopen_backend {
class Worker;
}
namespace direct_backend {
class DirectBackend;
}

// The ros2_control SystemInterface plugins of the arm (C12, C13): thin shells over one
// hal_contract::Hal per configure cycle.
//
// LIFECYCLE (C13)
//   on_init      parses the URDF interfaces (interfaces.hpp) and the parameters; ERROR on any
//                mismatch.
//   on_configure refuses (FAILURE) while active ("deactivate before configure": Hal::configure
//                clears the Hal's state before a backend can refuse a configure while armed, so the
//                Hal of a cycle is never configured twice), outside the C20 profile (with the
//                allowed values), and while the previous cycle's backend still runs (its worker
//                thread); otherwise drops the previous cycle, builds a new backend and Hal and runs
//                the configure op (FAILURE on an unreachable node or the 3 s timeout).
//   on_activate  resets the GPIO command metadata to 0, then the capability gate and the activate
//                op; FAILURE with the missing list (logged) when the gate refuses.
//   on_deactivate resets the command metadata to 0, then the deactivate op (stop confirmed);
//                ERROR if the stop is not confirmed, after latching the inhibit.
//   on_cleanup / on_shutdown  end the backend's thread (group stop if armed, then exit), joining
//                for at most 1 s; FAILURE (retryable) while it still runs.
//   on_error     Hal::on_error_nonblocking() and SUCCESS at once: ros2_control 4.48.1 calls it
//                synchronously from read() and then goes to UNCONFIGURED without on_cleanup; the
//                worker finishes the group stop and releases the socket by itself, and the next
//                on_configure waits for that (FAILURE until it has exited).
//
// CONTROL PATH (read/write: no socket I/O, no wait, no allocation, no logging)
//   read()   advances the backend (DirectArmHw only), Hal::read, publishes the joint states and
//            the hal GPIO; ERROR once per configure cycle on the loss of the transport or of a
//            drive (is_loss), OK otherwise.
//   write()  a new non-zero arm/request_id posts an arm op (never waited for; a repeat or 0 posts
//            nothing); then Hal::write with the permit (only when permit/present == 1) and the
//            setpoint; hal/verdict is the Hal's verdict, except that without a permit it is
//            kNoPermit unless the group inhibit is latched (kInhibited): the Hal itself answers
//            kNotArmed first, and the operator-facing reason is the missing permit. Always OK
//            (spec §8.2: a refusal is a verdict, never an error).
namespace arm_hw {

struct Params {
  Timing timing{};
  runtime_core::Requirements required{};
  std::optional<runtime_core::DeviceCapabilities> profile_limit{};
  // CanopenArmHw only.
  std::string can_interface{};
  std::array<std::uint8_t, hal_contract::kJoints> node_ids{1, 2, 3, 4, 5, 6};
  // From each joint's position command interface min/max.
  runtime_core::JointLimits<hal_contract::kJoints> limits{};
};

class ArmSystem : public hardware_interface::SystemInterface {
 public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  ArmSystem() = default;
  ArmSystem(const ArmSystem&) = delete;
  ArmSystem& operator=(const ArmSystem&) = delete;
  ~ArmSystem() override;

  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  // ---- observation (tests) ----
  // The reason of the last failed transition; empty after a successful one.
  const std::string& last_error() const { return last_error_; }
  // The current cycle's Hal; nullptr before the first configure and after cleanup.
  hal_contract::Hal* hal() { return hal_.get(); }
  const Params& params() const { return params_; }

 protected:
  // The backend of a new configure cycle; nullptr with `error` set when it cannot be built.
  virtual std::unique_ptr<hal_contract::Backend> make_backend(const Params& params,
                                                              std::string* error) = 0;
  // True while the current cycle's backend runs a thread of its own.
  virtual bool backend_running() const { return false; }
  // Ends the backend's thread within `timeout`; true once none runs.
  virtual bool stop_backend(runtime_core::Duration /*timeout*/) { return true; }
  // The Hal (and with it the backend) is about to be destroyed.
  virtual void forget_backend() {}
  // Time passes to `now` before the Hal reads (DirectArmHw steps its backend).
  virtual void advance(runtime_core::MonoTime /*now*/) {}
  // Blocks up to `timeout` for a lifecycle op; std::nullopt while it is still pending.
  virtual std::optional<hal_contract::OpResult> await(hal_contract::OpHandle op,
                                                      runtime_core::Duration timeout);
  // Which parameters the plugin needs besides the common ones.
  virtual bool needs_bus() const { return false; }

  static runtime_core::MonoTime now();
  CallbackReturn fail(const std::string& reason, CallbackReturn result = CallbackReturn::FAILURE);
  // Drops the Hal (the backend with it) after forget_backend().
  void drop_cycle();

  std::unique_ptr<hal_contract::Hal> hal_;

 private:
  using Command = hardware_interface::CommandInterface::SharedPtr;
  using State = hardware_interface::StateInterface::SharedPtr;
  struct Wide {
    Command lo;
    Command hi;
  };
  struct WideState {
    State lo;
    State hi;
  };

  bool parse_params(std::string* error);
  bool bind_handles(std::string* error);
  void reset_commands();
  void publish(const hal_contract::Feedback& fb);
  void set_verdict(hal_contract::HalVerdict v);
  // Reads a wide command; false if a handle was busy.
  static bool get(const Wide& w, std::uint64_t& out);
  static bool get8(const Command& c, std::uint8_t& out);
  static void put(const WideState& w, std::uint64_t value);
  static void put8(const State& s, std::uint8_t value);
  static void put32(const State& s, std::uint32_t value);

  Params params_{};
  std::string last_error_;
  bool bound_{false};
  bool active_{false};
  bool error_reported_{false};
  std::uint64_t last_request_id_{0};

  std::array<Command, hal_contract::kJoints> joint_cmd_{};
  std::array<State, hal_contract::kJoints> joint_pos_{};
  std::array<State, hal_contract::kJoints> joint_vel_{};
  std::array<Wide, kArmFields.size()> arm_{};
  std::array<Wide, kPermitFields.size()> permit_{};
  Command permit_present_{};
  std::array<Wide, kSetpointFields.size()> setpoint_{};
  std::vector<Command> all_gpio_commands_{};
  State verdict_{};
  State arm_state_{};
  WideState armed_request_id_{};
  WideState applied_seq_{};
  State faults_{};
  std::array<State, hal_contract::kJoints> stop_{};
};

// The arm over the CANopen backend on its socket I/O worker (canopen_backend::Worker), one Worker
// per configure cycle (a Worker cannot be restarted after its shutdown). Parameters besides the
// common ones: can_interface, node_ids.
//
// PLUGIN LIBRARY AND A DETACHED WORKER: the worker's code is linked into this plugin library, so
// the library must not be unmapped while a worker thread runs. The destructor ends the worker if
// it can (Worker::~Worker joins within 1 s); if a worker still runs when the destructor starts, it
// first marks this library RTLD_NODELETE, so a later dlclose (class_loader unloading the plugin)
// leaves it mapped and the detached thread can finish. The library then stays loaded until the
// process exits.
class CanopenArmHw : public ArmSystem {
 public:
  CanopenArmHw() = default;
  ~CanopenArmHw() override;

  // True once a destructor had to keep this library loaded for a running worker.
  static bool library_pinned();

 protected:
  std::unique_ptr<hal_contract::Backend> make_backend(const Params& params,
                                                      std::string* error) override;
  bool backend_running() const override;
  bool stop_backend(runtime_core::Duration timeout) override;
  void forget_backend() override { worker_ = nullptr; }
  bool needs_bus() const override { return true; }

 private:
  canopen_backend::Worker* worker_{nullptr};  // Owned by hal_.
};

// The arm over the in-process DirectBackend. No real-time worker drives it: read() steps it in
// W (2 ms) steps up to the read instant (CLOCK_MONOTONIC), so it advances at the controller rate
// in whole device periods, and a lifecycle op is stepped the same way while it is awaited. A gap
// longer than kMaxCatchUp (a stalled controller) is not replayed: the backend jumps to the grid
// point before the read, so its feedback ages as the hardware's would.
class DirectArmHw : public ArmSystem {
 public:
  static constexpr int kMaxCatchUp = 500;  // steps, 1 s

  DirectArmHw() = default;
  ~DirectArmHw() override;

  // The backend of the current cycle, for tests; nullptr between cycles.
  direct_backend::DirectBackend* backend() { return direct_; }

 protected:
  std::unique_ptr<hal_contract::Backend> make_backend(const Params& params,
                                                      std::string* error) override;
  void forget_backend() override { direct_ = nullptr; }
  void advance(runtime_core::MonoTime now) override;
  std::optional<hal_contract::OpResult> await(hal_contract::OpHandle op,
                                              runtime_core::Duration timeout) override;

 private:
  direct_backend::DirectBackend* direct_{nullptr};  // Owned by hal_.
  runtime_core::MonoTime next_step_{};
};

}  // namespace arm_hw

// ---- test seam (CanopenArmHw) ----
namespace arm_hw::testing {
// Builds the transport of a configure cycle instead of a SocketCAN socket on can_interface;
// nullptr (the default) restores SocketCAN. A factory returning nullptr fails the configure with
// `error`.
using TransportFactory = std::unique_ptr<canopen_backend::Transport> (*)(
    const std::string& can_interface, std::string* error);
void set_transport_factory(TransportFactory factory);
}  // namespace arm_hw::testing

// The same seam with C linkage, for a test that loads the plugin library with dlopen and finds it
// by name (dlsym).
extern "C" void arm_hw_testing_set_transport_factory(arm_hw::testing::TransportFactory factory);
