// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "canopen_backend/transport.hpp"
#include "canopen_codec/codec.hpp"
#include "canopen_master/testing/drive_bus.hpp"
#include "drive_core/joint.hpp"
#include "runtime_core/time.hpp"

// A wall-clock in-memory bus for the worker thread of CanopenArmHw: the six drive_core drives of
// canopen_master's DriveBus, ticked every W up to CLOCK_MONOTONIC whenever the worker sends or
// polls. One bus outlives the transports of several configure cycles. Hooks: hold() blocks the
// worker inside its next poll until released (a stalled worker), lose() makes the transport report
// a receive error (the worker's transport loss), and the bus counts releases and destructions.
namespace arm_hw::test {

namespace cc = canopen_codec;
namespace cmt = canopen_master::testing;
namespace rc = runtime_core;

class DriveBusSim {
 public:
  DriveBusSim() : last_tick_(clock()), bus_(last_tick_) { route(bus_.power_on(last_tick_)); }

  static rc::MonoTime clock() { return rc::SystemMonotonicClock{}.now(); }

  void send(const cc::Frame& f) {
    std::lock_guard<std::mutex> lock(mu_);
    const rc::MonoTime t = catch_up();
    route(bus_.deliver(f, t));
  }

  std::vector<cc::Frame> poll() {
    std::unique_lock<std::mutex> lock(mu_);
    ++polls_;
    cv_.notify_all();
    if (hold_) {
      blocked_ = true;
      cv_.notify_all();
      cv_.wait(lock, [&] { return !hold_; });
      blocked_ = false;
    }
    catch_up();
    std::vector<cc::Frame> out;
    out.swap(rx_);
    return out;
  }

  // Of transport number `id` (1 for the first created).
  canopen_backend::TransportCounts counts(int id) {
    std::lock_guard<std::mutex> lock(mu_);
    canopen_backend::TransportCounts c;
    if (id == lost_) {
      c.rx_error = 1;
      c.last_rx_errno = 100;  // ENETDOWN
    }
    return c;
  }

  void released() {
    std::lock_guard<std::mutex> lock(mu_);
    ++releases_;
    cv_.notify_all();
  }
  // A new transport is a new socket: it sees only frames sent after it opened.
  // Returns the new transport's number.
  int created() {
    std::lock_guard<std::mutex> lock(mu_);
    catch_up();
    rx_.clear();
    return ++created_;
  }
  void destroyed() {
    std::lock_guard<std::mutex> lock(mu_);
    ++destroyed_;
    cv_.notify_all();
  }

  // ---- hooks (test thread) ----
  void hold() {
    std::lock_guard<std::mutex> lock(mu_);
    hold_ = true;
  }
  void unhold() {
    std::lock_guard<std::mutex> lock(mu_);
    hold_ = false;
    cv_.notify_all();
  }
  // The newest transport reports a receive error from now on (a later one does not).
  void lose() {
    std::lock_guard<std::mutex> lock(mu_);
    lost_ = created_;
  }
  bool wait_blocked(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock, timeout, [&] { return blocked_; });
  }
  // Waits until the worker has polled `n` more times than now (each poll is one wake).
  bool wait_polls(std::uint64_t n, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    const std::uint64_t until = polls_ + n;
    return cv_.wait_for(lock, timeout, [&] { return polls_ >= until; });
  }
  bool wait_destroyed(int n, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock, timeout, [&] { return destroyed_ >= n; });
  }
  int releases() {
    std::lock_guard<std::mutex> lock(mu_);
    return releases_;
  }
  int created_count() {
    std::lock_guard<std::mutex> lock(mu_);
    return created_;
  }
  int destroyed_count() {
    std::lock_guard<std::mutex> lock(mu_);
    return destroyed_;
  }
  // The joints as they truly are (rad).
  std::array<double, cmt::kBusNodes> position() {
    std::lock_guard<std::mutex> lock(mu_);
    std::array<double, cmt::kBusNodes> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
      out.at(i) = static_cast<double>(bus_.drives.at(i)->joint().position_nrad) / 1e9;
    }
    return out;
  }

 private:
  // Ticks the drives every W up to now; returns now. Caller holds mu_.
  rc::MonoTime catch_up() {
    const rc::MonoTime t = clock();
    while (last_tick_ + drive_core::kTickPeriod <= t) {
      last_tick_ = last_tick_ + drive_core::kTickPeriod;
      route(bus_.tick(last_tick_));
    }
    return t;
  }
  void route(const std::vector<cmt::DriveFrame>& frames) {
    for (const cmt::DriveFrame& d : frames) rx_.push_back(d.frame);
  }

  std::mutex mu_;
  std::condition_variable cv_;
  rc::MonoTime last_tick_;
  cmt::DriveBus bus_;
  std::vector<cc::Frame> rx_;
  bool hold_{false};
  bool blocked_{false};
  int lost_{0};  // the number of the lost transport, 0 for none
  std::uint64_t polls_{0};
  int releases_{0};
  int created_{0};
  int destroyed_{0};
};

class DriveBusTransport final : public canopen_backend::Transport {
 public:
  explicit DriveBusTransport(std::shared_ptr<DriveBusSim> sim)
      : sim_(std::move(sim)), id_(sim_->created()) {}
  ~DriveBusTransport() override { sim_->destroyed(); }
  DriveBusTransport(const DriveBusTransport&) = delete;
  DriveBusTransport& operator=(const DriveBusTransport&) = delete;

  void send(const cc::Frame& frame) override {
    if (!released_) sim_->send(frame);
  }
  std::vector<cc::Frame> poll() override {
    if (released_) return {};
    return sim_->poll();
  }
  canopen_backend::TransportCounts counts() const override { return sim_->counts(id_); }
  void release() override {
    if (!released_) sim_->released();
    released_ = true;
  }

 private:
  std::shared_ptr<DriveBusSim> sim_;
  int id_;
  bool released_{false};
};

}  // namespace arm_hw::test
