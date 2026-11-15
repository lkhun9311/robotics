// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "canopen_codec/od.hpp"
#include "hal_contract/records.hpp"
#include "runtime_core/time.hpp"

// The per-node feedback join of the vdrive session (C4). A vdrive answers every SYNC with TPDO1,
// TPDO2 and TPDO3, all three carrying that node's sample_seq (D13). Each node counts its own
// sample_seq, so the counters differ across nodes: samples are joined per node, by that node's
// sample_seq, never across nodes by arrival or by SYNC.
//
// A node's sample is complete when all three TPDOs of one sample_seq arrived, in any order. The
// node's newest complete sample (seq16 order) is the one the robot feedback uses. Its source time
// is TPDO3 sample_time_us, a 32-bit microsecond count of the shared CLOCK_MONOTONIC domain (D5),
// unwrapped against the host time at reception; a receive time is never used as a sample time.
//
// Within one boot only: a boot-up of the node (on_boot) drops everything joined for it.
//
// Pure logic, no allocation: fixed arrays only. Not thread-safe.
namespace canopen_master {

inline constexpr std::size_t kSessionNodes = hal_contract::kJoints;

// One complete sample of one node.
struct NodeSample {
  std::uint16_t sample_seq{};
  std::uint16_t statusword{};
  std::int32_t position_urad{};
  std::int32_t velocity_urad_s{};
  std::uint16_t applied_seq{};
  std::uint16_t applied_tag{};
  std::uint32_t sample_time_us{};
  // sample_time_us unwrapped: the host instant whose floor(ns / 1000) mod 2^32 equals it, nearest
  // to the reception time of the TPDO3 (within +/- 2^31 us).
  runtime_core::MonoTime sampled_at{};
};

// The host instant of a 32-bit microsecond stamp, nearest to `reference`.
runtime_core::MonoTime unwrap_us(std::uint32_t stamp_us, runtime_core::MonoTime reference);

class FeedbackJoin {
 public:
  // Partial samples held per node; a fourth sample_seq evicts the oldest partial one.
  static constexpr std::size_t kPartials = 4;

  // `skew_limit`: a snapshot whose newest and oldest node source times differ by more than
  // this is marked skewed (C4: T_bus + W).
  explicit FeedbackJoin(runtime_core::Duration skew_limit);

  // Each returns the node's sample this TPDO completed, when it is the node's newest complete
  // one; std::nullopt otherwise (still partial, or older than the newest). `node` is 0..5;
  // another value is ignored. `now` is the reception time (used by TPDO3 to unwrap its stamp).
  std::optional<NodeSample> on_tpdo(std::size_t node, const canopen_codec::od::Tpdo1& pdo,
                                    runtime_core::MonoTime now);
  std::optional<NodeSample> on_tpdo(std::size_t node, const canopen_codec::od::Tpdo2& pdo,
                                    runtime_core::MonoTime now);
  std::optional<NodeSample> on_tpdo(std::size_t node, const canopen_codec::od::Tpdo3& pdo,
                                    runtime_core::MonoTime now);

  // The node booted (a boot-up message): its partial and complete samples are dropped.
  void on_boot(std::size_t node);
  // Drops everything.
  void reset();

  // The node's newest complete sample.
  const std::optional<NodeSample>& node(std::size_t node) const;

  // The robot snapshot from each node's newest complete sample: position and velocity (rad,
  // rad/s), node_sampled_at, sampled_at = the oldest of the six source times, and skewed.
  // std::nullopt until every node has a complete sample. The other Feedback fields are left at
  // their defaults; the session fills them.
  std::optional<hal_contract::Feedback> latest() const;

 private:
  struct Partial {
    bool used{false};
    std::uint8_t have{0};  // Bit 0: TPDO1, bit 1: TPDO2, bit 2: TPDO3.
    std::uint64_t order{0};
    NodeSample sample{};
  };
  struct Node {
    std::array<Partial, kPartials> partials{};
    std::optional<NodeSample> latest{};
  };

  Partial& slot(Node& n, std::uint16_t seq);
  std::optional<NodeSample> complete(Node& n, Partial& p);

  runtime_core::Duration skew_limit_;
  std::array<Node, kSessionNodes> nodes_{};
  std::uint64_t order_{0};
};

}  // namespace canopen_master
