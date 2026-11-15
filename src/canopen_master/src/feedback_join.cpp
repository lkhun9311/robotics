// SPDX-License-Identifier: Apache-2.0
#include "canopen_master/feedback_join.hpp"

#include <algorithm>

#include "runtime_core/seq.hpp"

namespace canopen_master {

namespace od = canopen_codec::od;
using runtime_core::Duration;
using runtime_core::MonoTime;

namespace {

constexpr std::uint8_t kHave1 = 1U << 0U;
constexpr std::uint8_t kHave2 = 1U << 1U;
constexpr std::uint8_t kHave3 = 1U << 2U;
constexpr std::uint8_t kHaveAll = kHave1 | kHave2 | kHave3;
constexpr double kMicro = 1e-6;

}  // namespace

std::int64_t floor_us(MonoTime t) {
  std::int64_t us = t.ns / 1000;
  if (t.ns % 1000 < 0) {
    --us;
  }
  return us;
}

MonoTime unwrap_us(std::uint32_t stamp_us, MonoTime reference) {
  const std::int64_t ref_us = floor_us(reference);
  const auto ref_low = static_cast<std::uint32_t>(static_cast<std::uint64_t>(ref_us));
  // The signed 32-bit distance from the reference to the stamp: nearest in either direction.
  const auto delta = static_cast<std::int32_t>(stamp_us - ref_low);
  return MonoTime{(ref_us + delta) * 1000};
}

FeedbackJoin::FeedbackJoin(Duration skew_limit) : skew_limit_(skew_limit) {}

FeedbackJoin::Partial& FeedbackJoin::slot(Node& n, std::uint16_t seq) {
  Partial* free = nullptr;
  Partial* oldest = &n.partials.front();
  for (Partial& p : n.partials) {
    if (p.used && p.sample.sample_seq == seq) {
      return p;
    }
    if (!p.used && free == nullptr) {
      free = &p;
    }
    if (p.order < oldest->order) {
      oldest = &p;
    }
  }
  Partial& p = free != nullptr ? *free : *oldest;
  p = Partial{};
  p.used = true;
  p.order = ++order_;
  p.sample.sample_seq = seq;
  return p;
}

std::optional<NodeSample> FeedbackJoin::complete(Node& n, Partial& p) {
  if (p.have != kHaveAll) {
    return std::nullopt;
  }
  const NodeSample sample = p.sample;
  p = Partial{};
  if (n.latest && !runtime_core::seq16_newer(sample.sample_seq, n.latest->sample_seq)) {
    return std::nullopt;  // Completed late: an older sample than the one already published.
  }
  n.latest = sample;
  return sample;
}

std::optional<NodeSample> FeedbackJoin::on_tpdo(std::size_t node, const od::Tpdo1& pdo,
                                                MonoTime /*now*/) {
  if (node >= kSessionNodes) {
    return std::nullopt;
  }
  Node& n = nodes_[node];
  Partial& p = slot(n, pdo.sample_seq);
  p.sample.statusword = pdo.statusword;
  p.sample.position_urad = pdo.position_actual_value;
  p.have = static_cast<std::uint8_t>(p.have | kHave1);
  return complete(n, p);
}

std::optional<NodeSample> FeedbackJoin::on_tpdo(std::size_t node, const od::Tpdo2& pdo,
                                                MonoTime /*now*/) {
  if (node >= kSessionNodes) {
    return std::nullopt;
  }
  Node& n = nodes_[node];
  Partial& p = slot(n, pdo.sample_seq);
  p.sample.velocity_urad_s = pdo.velocity_actual_value;
  p.sample.applied_seq = pdo.applied_seq;
  p.have = static_cast<std::uint8_t>(p.have | kHave2);
  return complete(n, p);
}

std::optional<NodeSample> FeedbackJoin::on_tpdo(std::size_t node, const od::Tpdo3& pdo,
                                                MonoTime now) {
  if (node >= kSessionNodes) {
    return std::nullopt;
  }
  Node& n = nodes_[node];
  Partial& p = slot(n, pdo.sample_seq);
  p.sample.sample_time_us = pdo.sample_time_us;
  p.sample.sampled_at = unwrap_us(pdo.sample_time_us, now);
  p.sample.applied_tag = pdo.applied_tag;
  p.have = static_cast<std::uint8_t>(p.have | kHave3);
  return complete(n, p);
}

void FeedbackJoin::on_boot(std::size_t node) {
  if (node < kSessionNodes) {
    nodes_[node] = Node{};
  }
}

void FeedbackJoin::reset() { nodes_ = {}; }

const std::optional<NodeSample>& FeedbackJoin::node(std::size_t node) const {
  return nodes_.at(node).latest;
}

std::optional<hal_contract::Feedback> FeedbackJoin::latest() const {
  hal_contract::Feedback f{};
  MonoTime oldest{};
  MonoTime newest{};
  for (std::size_t i = 0; i < kSessionNodes; ++i) {
    const std::optional<NodeSample>& s = nodes_[i].latest;
    if (!s) {
      return std::nullopt;
    }
    f.position[i] = static_cast<double>(s->position_urad) * kMicro;
    f.velocity[i] = static_cast<double>(s->velocity_urad_s) * kMicro;
    f.node_sampled_at[i] = s->sampled_at;
    oldest = i == 0 ? s->sampled_at : std::min(oldest, s->sampled_at);
    newest = i == 0 ? s->sampled_at : std::max(newest, s->sampled_at);
  }
  f.sampled_at = oldest;
  f.skewed = newest - oldest > skew_limit_;
  return f;
}

}  // namespace canopen_master
